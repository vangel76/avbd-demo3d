/*
 * avbd_bench : profiling benchmark for the AVBD solver core.
 *
 * Builds a heavy scene and reports where Solver::step() spends its time. For
 * each scene, two parts:
 *   Part 1 - per-phase breakdown (broadphase / colouring / primal / dual /
 *            other) at the default thread count, via Solver::profile.
 *   Part 2 - thread scaling: total step time at 1/2/4/8/hw threads, showing
 *            whether the solver is compute-bound or limited by thread-pool
 *            fork/join synchronisation overhead.
 *
 * Three scenes, each stressing a different code path:
 *   wall  - 1186 rigid OABB boxes (brick wall): box-box narrow phase.
 *   hull  - ~980 convex-hull cubes (pile)      : convex-convex narrow phase.
 *   cloth - 2500-particle triangle-FEM cloth   : particle + FEM/bend path.
 *
 * This is a measurement tool, not a correctness test (see avbd_test for that).
 * Run it before changing the solver, then again after, to confirm a change
 * actually helped.
 *
 * Usage: avbd_bench [scene] [steps]
 *        scene  = wall | hull | cloth | all   (default: all)
 *        steps  = timed steps per run         (default: 400)
 */

#include "solver.h"
#include "parallel.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

// --- Scene builders. Each disables sleeping so every body stays active and the
//     per-step cost is roughly constant across the run. Each returns the body
//     count. --------------------------------------------------------------

// Staggered (running-bond) brick wall of rigid OABB boxes on a static ground:
// many interlocked bodies exercising the box-box narrow phase.
static int buildWall(Solver &solver)
{
    solver.sleepEnabled = false;
    const int width = 40, height = 30;
    const float bx = 1.0f, by = 2.0f, bz = 0.5f;

    new Rigid(&solver, float3{width * bx + 8.0f, 20.0f, 1.0f}, 0.0f, 0.6f, float3{0, 0, 0});
    for (int row = 0; row < height; ++row)
    {
        float offset = (row & 1) ? bx * 0.5f : 0.0f;
        int count = (row & 1) ? width - 1 : width;
        for (int col = 0; col < count; ++col)
        {
            float x = (col - width / 2.0f) * bx + offset;
            float z = 0.5f + bz * 0.5f + row * bz;
            new Rigid(&solver, float3{bx, by, bz}, 1.0f, 0.6f, float3{x, 0, z});
        }
    }

    int n = 0;
    for (Body *b = solver.bodies; b != 0; b = b->next)
        ++n;
    return n;
}

// A grid of convex-hull unit cubes settling into a pile on a static ground:
// drives the convex-convex (SAT + clipping) narrow phase rather than the tuned
// box-box path.
static int buildHull(Solver &solver)
{
    solver.sleepEnabled = false;
    new Rigid(&solver, float3{60.0f, 60.0f, 1.0f}, 0.0f, 0.5f, float3{0, 0, 0});

    const int nx = 14, ny = 14, nz = 5; // 980 hull cubes
    const float s = 1.06f;
    for (int z = 0; z < nz; ++z)
        for (int y = 0; y < ny; ++y)
            for (int x = 0; x < nx; ++x)
            {
                float3 pos = {(x - nx / 2.0f) * s, (y - ny / 2.0f) * s, 1.0f + z * s};
                new Rigid(&solver, ConvexHull::createBox(float3{1, 1, 1}), 1.0f, 0.5f, pos);
            }

    int n = 0;
    for (Body *b = solver.bodies; b != 0; b = b->next)
        ++n;
    return n;
}

// A large triangle-FEM cloth dropped onto a static ground box: exercises the
// 3-DOF particle path, FEMTriangle membrane elements, BendEdge bending, and
// particle-rigid contact.
static int buildCloth(Solver &solver)
{
    solver.sleepEnabled = false;
    new Rigid(&solver, float3{30.0f, 30.0f, 1.0f}, 0.0f, 0.5f, float3{0, 0, 0}); // ground top z=0.5

    const int N = 50; // 2500 particles
    const float s = 0.12f;
    const float z = 4.0f;

    std::vector<float3> verts;
    verts.reserve(N * N);
    for (int y = 0; y < N; ++y)
        for (int x = 0; x < N; ++x)
            verts.push_back(float3{x * s - (N - 1) * s * 0.5f, y * s - (N - 1) * s * 0.5f, z});

    std::vector<int> tris;
    tris.reserve((N - 1) * (N - 1) * 6);
    for (int y = 0; y < N - 1; ++y)
        for (int x = 0; x < N - 1; ++x)
        {
            int i = y * N + x;
            tris.push_back(i);     tris.push_back(i + 1);     tris.push_back(i + N);
            tris.push_back(i + 1); tris.push_back(i + N + 1); tris.push_back(i + N);
        }

    // The Cloth struct is intentionally leaked: this is a short-lived benchmark
    // process. The particles/forces it creates are owned by the solver.
    new Cloth(&solver, verts.data(), (int)verts.size(), tris.data(), (int)tris.size() / 3,
              1.0f, 0.01f, 1000.0f, 0.3f, 0.5f, 0.05f, 0.5f);

    int n = 0;
    for (Body *b = solver.bodies; b != 0; b = b->next)
        ++n;
    return n;
}

// --- Memory-locality probe ------------------------------------------------
// Does laying contact forces out contiguously (a memory pool) speed up the
// per-body force traversal that dominates the primal phase? This mirrors that
// traversal on dummy manifolds in two layouts: one contiguous block vs.
// individually new'd and heap-fragmented.

struct LocManifold
{
    float3 basis[3];
    struct { float3 rA, rB, C0, penalty, lambda; } contacts[8];
    int numContacts;
    float friction;
    char pad[244]; // pad toward the real Manifold footprint (~770 bytes)
};

static double localityPass(const std::vector<std::vector<LocManifold *>> &bodyForces,
                           int iterations)
{
    auto t0 = std::chrono::high_resolution_clock::now();
    float3 sink{0, 0, 0};
    for (int it = 0; it < iterations; ++it)
        for (const std::vector<LocManifold *> &fl : bodyForces)
        {
            float3 acc{0, 0, 0};
            for (LocManifold *m : fl)
                for (int c = 0; c < m->numContacts; ++c)
                {
                    acc = acc + m->contacts[c].rA + m->contacts[c].C0 + m->contacts[c].penalty;
                    m->contacts[c].lambda = acc; // write-back, as updateDual does
                }
            sink = sink + acc;
        }
    auto t1 = std::chrono::high_resolution_clock::now();
    volatile float keep = sink.x + sink.y + sink.z;
    (void)keep;
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

static void benchLocality()
{
    const int N = 15000;  // contact manifolds (a dense rigid scene)
    const int M = 2300;   // bodies
    const int K = 13;     // forces per body (~2*N/M)
    const int iters = 10; // solver iterations per step

    unsigned rng = 12345u;
    auto rnd = [&]() { rng = rng * 1664525u + 1013904223u; return rng; };

    std::vector<std::vector<int>> idx(M);
    for (int b = 0; b < M; ++b)
        for (int k = 0; k < K; ++k)
            idx[b].push_back((int)(rnd() % (unsigned)N));

    // Layout A: one contiguous block. Layout B: individually allocated, with
    // junk allocations interleaved so the live manifolds span a wider range.
    LocManifold *block = new LocManifold[N];
    std::vector<LocManifold *> scattered(N);
    std::vector<void *> junk;
    for (int i = 0; i < N; ++i)
    {
        scattered[i] = new LocManifold();
        if ((i & 1) == 0)
            junk.push_back(malloc(64 + (rnd() % 512)));
    }
    for (int i = 0; i < N; ++i)
    {
        int nc = 4 + (int)(rnd() % 5);
        block[i].numContacts = nc;
        scattered[i]->numContacts = nc;
    }

    std::vector<std::vector<LocManifold *>> contig(M), scat(M);
    for (int b = 0; b < M; ++b)
        for (int j : idx[b])
        {
            contig[b].push_back(&block[j]);
            scat[b].push_back(scattered[j]);
        }

    localityPass(contig, iters); // warm up
    localityPass(scat, iters);
    double cMs = 1e30, sMs = 1e30;
    for (int r = 0; r < 7; ++r)
    {
        double c = localityPass(contig, iters);
        double s = localityPass(scat, iters);
        if (c < cMs) cMs = c;
        if (s < sMs) sMs = s;
    }

    printf("Memory-locality probe\n");
    printf("  %d manifolds (%zu KB working set), %d bodies x %d forces, %d iterations\n",
           N, (size_t)N * sizeof(LocManifold) / 1024, M, K, iters);
    printf("  %-22s %9.3f ms / step-equivalent\n", "contiguous (pool)", cMs);
    printf("  %-22s %9.3f ms / step-equivalent\n", "scattered (new)", sMs);
    printf("  -> a manifold pool would be %.2fx on the force traversal\n",
           cMs > 0.0 ? sMs / cMs : 0.0);

    delete[] block;
    for (LocManifold *m : scattered)
        delete m;
    for (void *p : junk)
        free(p);
}

enum Scene { SCENE_WALL, SCENE_HULL, SCENE_CLOTH };

static int buildScene(Solver &solver, Scene scene)
{
    switch (scene)
    {
    case SCENE_HULL:  return buildHull(solver);
    case SCENE_CLOTH: return buildCloth(solver);
    default:          return buildWall(solver);
    }
}

static const char *sceneName(Scene scene)
{
    switch (scene)
    {
    case SCENE_HULL:  return "hull (convex-hull cube pile)";
    case SCENE_CLOTH: return "cloth (triangle-FEM)";
    default:          return "wall (rigid box brick wall)";
    }
}

// Runs `steps` solver steps and returns the elapsed wall-clock milliseconds.
static double runTimed(Solver &solver, int steps)
{
    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < steps; ++i)
        solver.step();
    auto t1 = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

static void runScene(Scene scene, int steps, int warmup)
{
    int bodies;
    {
        Solver probe;
        bodies = buildScene(probe, scene);
    }

    printf("==================================================================\n");
    printf("Scene: %s\n", sceneName(scene));
    printf("  %d bodies | %d timed steps (+ %d warmup)\n\n", bodies, steps, warmup);

    // --- Part 1: per-phase breakdown -----------------------------------------
    {
        Solver solver;
        buildScene(solver, scene);
        for (int i = 0; i < warmup; ++i)
            solver.step();

        solver.profileEnabled = true;
        solver.profile.reset();
        double wallMs = runTimed(solver, steps);
        const SolverProfile &p = solver.profile;
        double total = p.totalMs();

        printf("  Part 1 - per-phase breakdown (default thread count)\n");
        printf("    %-14s %12s %12s %9s  %s\n", "phase", "total ms", "ms / step", "share", "exec");
        struct Row { const char *name; double ms; const char *kind; };
        Row rows[] = {
            {"broadphase",      p.broadphaseMs,  "parallel"},
            {"force-gather",    p.forceGatherMs, "SERIAL"},
            {"force-init",      p.initMs,        "par+serial"},
            {"warmstart",       p.warmstartMs,   "parallel"},
            {"colouring",       p.coloringMs,    "SERIAL"},
            {"primal",          p.primalMs,      "parallel"},
            {"dual",            p.dualMs,        "parallel"},
            {"other",           p.otherMs,       "mixed"},
        };
        for (const Row &r : rows)
            printf("    %-14s %12.1f %12.4f %8.1f%%  %s\n", r.name, r.ms,
                   r.ms / steps, total > 0.0 ? 100.0 * r.ms / total : 0.0, r.kind);
        printf("    %-14s %12.1f %12.4f %8.1f%%\n", "TOTAL (sum)", total,
               total / steps, 100.0);
        printf("    wall-clock (incl. profiling) : %.1f ms total, %.4f ms/step\n\n",
               wallMs, wallMs / steps);
    }

    // --- Part 2: thread scaling ----------------------------------------------
    {
        const int threadCounts[] = {1, 2, 4, 8, 0};
        printf("  Part 2 - thread scaling (0 = hardware concurrency)\n");
        printf("    %-12s %12s %12s %10s %10s\n",
               "threads", "total ms", "ms / step", "speedup", "efficiency");
        double singleMs = 0;
        for (int ti = 0; ti < (int)(sizeof(threadCounts) / sizeof(int)); ++ti)
        {
            int t = threadCounts[ti];
            Solver solver;
            solver.setThreads(t);
            buildScene(solver, scene);
            for (int i = 0; i < warmup; ++i)
                solver.step();
            double ms = runTimed(solver, steps);
            if (ti == 0)
                singleMs = ms;

            int lanes = solver.pool->laneCount();
            double speedup = ms > 0.0 ? singleMs / ms : 0.0;
            char label[24];
            snprintf(label, sizeof(label), t == 0 ? "%d->%d (hw)" : "%d", t, lanes);
            printf("    %-12s %12.1f %12.4f %9.2fx %9.0f%%\n", label, ms, ms / steps,
                   speedup, lanes > 0 ? 100.0 * speedup / lanes : 0.0);
        }
        printf("\n");
    }
}

int main(int argc, char **argv)
{
    Scene scenes[3] = {SCENE_WALL, SCENE_HULL, SCENE_CLOTH};
    int numScenes = 3;
    int steps = 400;
    const int warmup = 30;

    if (argc > 1)
    {
        if (strcmp(argv[1], "locality") == 0)   { benchLocality(); return 0; }
        else if (strcmp(argv[1], "wall") == 0)  { scenes[0] = SCENE_WALL;  numScenes = 1; }
        else if (strcmp(argv[1], "hull") == 0)  { scenes[0] = SCENE_HULL;  numScenes = 1; }
        else if (strcmp(argv[1], "cloth") == 0) { scenes[0] = SCENE_CLOTH; numScenes = 1; }
        else if (strcmp(argv[1], "all") != 0)
        {
            printf("unknown scene '%s' (expected wall | hull | cloth | locality | all)\n", argv[1]);
            return 1;
        }
    }
    if (argc > 2)
        steps = atoi(argv[2]);

    printf("AVBD solver profiling benchmark\n\n");
    for (int i = 0; i < numScenes; ++i)
        runScene(scenes[i], steps, warmup);

    return 0;
}

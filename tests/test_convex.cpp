/*
 * Smoke tests for the AVBD core: convex-hull collision and the OBB path.
 * Built as the `avbd_test` target. Exits non-zero on failure.
 */

#include "solver.h"
#include <cmath>
#include <cstdio>
#include <vector>

static int failures = 0;

static void check(bool ok, const char *name, float value, float expected, float tol)
{
    bool pass = ok && fabsf(value - expected) <= tol;
    printf("[%s] %-34s value=%.4f expected=%.4f tol=%.3f\n",
           pass ? "PASS" : "FAIL", name, value, expected, tol);
    if (!pass)
        ++failures;
}

// Drops a body and returns its resting Z height after stepping.
static float settleZ(Solver &solver, Rigid *body, int steps)
{
    for (int i = 0; i < steps; ++i)
        solver.step();
    return body->positionLin.z;
}

// A unit cube as a convex hull dropped onto a static box ground (box-vs-convex path).
static void testConvexCubeOnGround()
{
    Solver solver;
    new Rigid(&solver, float3{100, 100, 1}, 0.0f, 0.5f, float3{0, 0, 0}); // ground, top at z=0.5
    Rigid *cube = new Rigid(&solver, ConvexHull::createBox(float3{1, 1, 1}), 1.0f, 0.5f, float3{0, 0, 4});
    float z = settleZ(solver, cube, 240);
    check(true, "convex cube rests on ground", z, 1.0f, 0.1f);
}

// Two stacked convex cubes on a static ground (convex-vs-convex path).
static void testConvexStack()
{
    Solver solver;
    new Rigid(&solver, float3{100, 100, 1}, 0.0f, 0.5f, float3{0, 0, 0});
    Rigid *lower = new Rigid(&solver, ConvexHull::createBox(float3{1, 1, 1}), 1.0f, 0.5f, float3{0, 0, 1.2f});
    Rigid *upper = new Rigid(&solver, ConvexHull::createBox(float3{1, 1, 1}), 1.0f, 0.5f, float3{0, 0, 2.6f});
    for (int i = 0; i < 300; ++i)
        solver.step();
    check(true, "convex stack lower box", lower->positionLin.z, 1.0f, 0.12f);
    check(true, "convex stack upper box", upper->positionLin.z, 2.0f, 0.18f);
}

// Reference: identical scene with plain box bodies (OBB path) for comparison.
static void testBoxStackReference()
{
    Solver solver;
    new Rigid(&solver, float3{100, 100, 1}, 0.0f, 0.5f, float3{0, 0, 0});
    Rigid *lower = new Rigid(&solver, float3{1, 1, 1}, 1.0f, 0.5f, float3{0, 0, 1.2f});
    Rigid *upper = new Rigid(&solver, float3{1, 1, 1}, 1.0f, 0.5f, float3{0, 0, 2.6f});
    for (int i = 0; i < 300; ++i)
        solver.step();
    check(true, "box stack lower box (OBB path)", lower->positionLin.z, 1.0f, 0.12f);
    check(true, "box stack upper box (OBB path)", upper->positionLin.z, 2.0f, 0.18f);
}

// Convex hull mass properties: a unit cube of density 1 has mass 1 and
// moment 1/6 about each axis.
static void testHullMassProperties()
{
    ConvexHull *hull = ConvexHull::createBox(float3{1, 1, 1});
    float mass;
    float3 com, moment;
    computeHullMassProperties(hull, 1.0f, mass, com, moment);
    check(true, "unit cube hull mass", mass, 1.0f, 1e-3f);
    check(true, "unit cube hull moment.x", moment.x, 1.0f / 6.0f, 1e-3f);
    check(true, "unit cube hull com magnitude", length(com), 0.0f, 1e-4f);
    delete hull;
}

// Builds a small pyramid scene and returns the resting Z of the top body.
static float pyramidTopZ(int threadCount)
{
    Solver solver;
    solver.setThreads(threadCount);
    new Rigid(&solver, float3{100, 100, 1}, 0.0f, 0.5f, float3{0, 0, 0});
    Rigid *top = 0;
    const int size = 6;
    for (int y = 0; y < size; ++y)
        for (int x = 0; x < size - y; ++x)
            top = new Rigid(&solver, float3{1, 0.5f, 0.5f}, 1.0f, 0.5f,
                            float3{x * 1.01f + y * 0.5f - size / 2.0f, 0.0f, y * 0.85f + 0.5f});
    for (int i = 0; i < 200; ++i)
        solver.step();
    return top->positionLin.z;
}

// The colored Gauss-Seidel solver must be deterministic: a fixed colouring gives
// the same result regardless of how many threads process each colour.
static void testThreadDeterminism()
{
    float single = pyramidTopZ(1);
    float multi = pyramidTopZ(8);
    check(true, "single vs multi-thread settle Z", multi, single, 1e-3f);
}

// A tall single-column box stack: isolates the spatial-hash broadphase by
// checking that vertically stacked contacts are all found (no sinking).
static void testTallStack()
{
    Solver solver;
    new Rigid(&solver, float3{100, 100, 1}, 0.0f, 0.5f, float3{0, 0, 0});
    Rigid *top = 0;
    for (int i = 0; i < 12; ++i)
        top = new Rigid(&solver, float3{1, 1, 1}, 1.0f, 0.5f, float3{0, 0, 1.0f + i * 1.05f});
    for (int i = 0; i < 400; ++i)
        solver.step();
    // Ground top 0.5 + 12 boxes of height 1 => top centre near 12.0.
    check(true, "tall stack top box (broadphase)", top->positionLin.z, 12.0f, 0.5f);
}

// A staggered (running-bond) brick wall on a solid ground: exercises the
// spatial-hash broadphase and contact solver with a few hundred interlocked
// bodies. Checks that the wall as a whole stays standing.
static void testBrickWall()
{
    const int width = 24;
    const int height = 15;
    const float bx = 1.0f, by = 2.0f, bz = 0.5f;

    Solver solver;
    new Rigid(&solver, float3{80, 20, 1}, 0.0f, 0.6f, float3{0, 0, 0}); // ground top z=0.5

    std::vector<int> rowOf;
    std::vector<Rigid *> bricks;
    for (int row = 0; row < height; ++row)
    {
        float offset = (row & 1) ? bx * 0.5f : 0.0f;
        int count = (row & 1) ? width - 1 : width;
        for (int col = 0; col < count; ++col)
        {
            float x = (col - width / 2.0f) * bx + offset;
            float z = 0.5f + bz * 0.5f + row * bz;
            bricks.push_back(new Rigid(&solver, float3{bx, by, bz}, 1.0f, 0.6f, float3{x, 0, z}));
            rowOf.push_back(row);
        }
    }

    for (int i = 0; i < 300; ++i)
        solver.step();

    // Count bricks that dropped well below their expected row height.
    int fallen = 0;
    for (size_t i = 0; i < bricks.size(); ++i)
    {
        float expected = 0.5f + bz * 0.5f + rowOf[i] * bz;
        if (!isfinite(bricks[i]->positionLin.z) || bricks[i]->positionLin.z < expected - 0.3f)
            ++fallen;
    }
    float fallenPct = 100.0f * fallen / (float)bricks.size();
    printf("    [brick wall] %d bricks, %d fell (%.1f%%)\n",
           (int)bricks.size(), fallen, fallenPct);
    check(true, "brick wall stays standing", fallenPct, 0.0f, 2.0f);
}

// Body sleeping: a settled body freezes, stays frozen, and wakes on impact.
static void testSleeping()
{
    Solver solver; // sleeping is enabled by default
    new Rigid(&solver, float3{100, 100, 1}, 0.0f, 0.5f, float3{0, 0, 0});
    Rigid *box = new Rigid(&solver, float3{1, 1, 1}, 1.0f, 0.5f, float3{0, 0, 2});

    for (int i = 0; i < 240; ++i)
        solver.step();
    check(box->asleep, "box sleeps after settling", box->asleep ? 1.0f : 0.0f, 1.0f, 0.5f);

    // A sleeping body must not drift at all.
    float restZ = box->positionLin.z;
    for (int i = 0; i < 60; ++i)
        solver.step();
    check(box->asleep, "sleeping box stays frozen", box->positionLin.z, restZ, 1.0e-6f);

    // Dropping another box on top must wake the sleeper.
    new Rigid(&solver, float3{1, 1, 1}, 1.0f, 0.5f, float3{0, 0, 5});
    bool woke = false;
    for (int i = 0; i < 120; ++i)
    {
        solver.step();
        if (!box->asleep)
            woke = true;
    }
    check(woke, "sleeping box wakes on impact", woke ? 1.0f : 0.0f, 1.0f, 0.5f);
}

int main()
{
    testHullMassProperties();
    testConvexCubeOnGround();
    testConvexStack();
    testBoxStackReference();
    testThreadDeterminism();
    testTallStack();
    testBrickWall();
    testSleeping();

    printf("\n%s (%d failure%s)\n", failures ? "TESTS FAILED" : "ALL TESTS PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}

/*
 * Copyright (c) 2026 Chris Giles
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Chris Giles makes no representations about the suitability
 * of this software for any purpose.
 * It is provided "as is" without express or implied warranty.
 */

#include "solver.h"
#include "parallel.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

Solver::Solver()
    : bodies(0), forces(0), threads(0), pool(0), profileEnabled(false)
{
    defaultParams();
    profile.reset();
    pool = new ThreadPool(threads);
}

Solver::~Solver()
{
    clear();
    delete pool;
}

void Solver::setThreads(int n)
{
    threads = n;
    delete pool;
    pool = new ThreadPool(n);
}

Rigid *Solver::pick(float3 origin, float3 dir, float3 &local)
{
    const float epsilon = 1.0e-6f;
    float bestT = INFINITY;
    Rigid *bestBody = 0;
    float3 bestLocal = {0, 0, 0};

    // Ray-cast against each rigid body's oriented box (AABB extents for hulls)
    // by transforming the ray into body local space.
    for (Body *b = bodies; b != 0; b = b->next)
    {
        if (b->kind != BODY_RIGID || b->mass <= 0.0f)
            continue;
        Rigid *body = (Rigid *)b;

        quat invRot = conjugate(body->positionAng);
        float3 o = rotate(invRot, origin - body->positionLin);
        float3 d = rotate(invRot, dir);
        float3 half = body->size * 0.5f;

        float tEnter = 0.0f;
        float tExit = INFINITY;
        bool hit = true;

        for (int i = 0; i < 3; ++i)
        {
            if (fabsf(d[i]) < epsilon)
            {
                if (o[i] < -half[i] || o[i] > half[i])
                {
                    hit = false;
                    break;
                }
                continue;
            }

            float invD = 1.0f / d[i];
            float t0 = (-half[i] - o[i]) * invD;
            float t1 = (half[i] - o[i]) * invD;
            if (t0 > t1)
            {
                float tmp = t0;
                t0 = t1;
                t1 = tmp;
            }

            tEnter = max(tEnter, t0);
            tExit = min(tExit, t1);
            if (tEnter > tExit)
            {
                hit = false;
                break;
            }
        }

        if (!hit)
            continue;

        float tHit = tEnter >= 0.0f ? tEnter : tExit;
        if (tHit < 0.0f)
            continue;

        if (tHit < bestT)
        {
            bestT = tHit;
            bestBody = body;
            bestLocal = o + d * tHit;
        }
    }

    if (!bestBody)
        return 0;

    local = bestLocal;
    return bestBody;
}

void Solver::clear()
{
    while (forces)
        delete forces;

    while (bodies)
        delete bodies;
}

void Solver::defaultParams()
{
    dt = 1.0f / 60.0f;
    gravity = -10.0f;
    iterations = 10;

    // Note: in the paper, beta is suggested to be [1, 1000]. Technically, the best choice will
    // depend on the length, mass, and constraint function scales (ie units) of your simulation,
    // along with your strategy for incrementing the penalty parameters.
    betaLin = 10000.0f;
    betaAng = 100.0f;

    // Alpha controls how much stabilization is applied. Higher values give slower and smoother
    // error correction, and lower values are more responsive and energetic.
    alpha = 0.99f;

    // Gamma controls how much the penalty and lambda values are decayed each step during warmstarting.
    gamma = 0.999f;

    // 0 selects the hardware concurrency for the thread pool.
    threads = 0;

    // Sleeping is enabled by default.
    sleepEnabled = true;
    sleepThresholdLin = 0.05f;
    sleepThresholdAng = 0.05f;
    sleepTime = 0.5f;
}

namespace
{
// Walks the force list of `body` (each body threads its own list through the
// matching Force::bodyNext slot).
inline Force *nextForce(Force *f, const Body *body)
{
    return f->bodyNext[f->slotOf(body)];
}

// Primal update for a 6-DOF rigid body: assemble and solve the 6x6 SPD system.
void updateRigidPrimal(Solver *solver, Rigid *body)
{
    float dt = solver->dt;

    float3x3 MLin = diagonal(body->mass, body->mass, body->mass);
    float3x3 MAng = diagonal(body->moment.x, body->moment.y, body->moment.z);

    float3x3 lhsLin = MLin / (dt * dt);
    float3x3 lhsAng = MAng / (dt * dt);
    float3x3 lhsCross = float3x3{0, 0, 0, 0, 0, 0, 0, 0, 0};

    float3 rhsLin = MLin / (dt * dt) * (body->positionLin - body->inertialLin);
    float3 rhsAng = MAng / (dt * dt) * (body->positionAng - body->inertialAng);

    for (Force *f = body->forces; f != 0; f = nextForce(f, body))
        f->updatePrimal(body, solver->alpha, lhsLin, lhsAng, lhsCross, rhsLin, rhsAng);

    float3 dxLin, dxAng;
    solve(lhsLin, lhsAng, lhsCross, -rhsLin, -rhsAng, dxLin, dxAng);
    body->positionLin = body->positionLin + dxLin;
    body->positionAng = body->positionAng + dxAng;
}

// Primal update for a 3-DOF particle: assemble and solve the 3x3 SPD system.
// Particle-aware forces stamp only the linear blocks.
void updateParticlePrimal(Solver *solver, Particle *body)
{
    float dt = solver->dt;
    float m = body->mass;

    float3x3 lhsLin = diagonal(m, m, m) / (dt * dt);
    float3x3 lhsAng = float3x3{0, 0, 0, 0, 0, 0, 0, 0, 0};
    float3x3 lhsCross = lhsAng;
    float3 rhsLin = lhsLin * (body->positionLin - body->inertialLin);
    float3 rhsAng = float3{0, 0, 0};

    for (Force *f = body->forces; f != 0; f = nextForce(f, body))
        f->updatePrimal(body, solver->alpha, lhsLin, lhsAng, lhsCross, rhsLin, rhsAng);

    body->positionLin = body->positionLin + solve3(lhsLin, -rhsLin);
}

inline uint64_t packCell(int x, int y, int z)
{
    uint64_t ux = (uint64_t)((unsigned)(x + 1048576)) & 0x1FFFFFu;
    uint64_t uy = (uint64_t)((unsigned)(y + 1048576)) & 0x1FFFFFu;
    uint64_t uz = (uint64_t)((unsigned)(z + 1048576)) & 0x1FFFFFu;
    return ux | (uy << 21) | (uz << 42);
}

// Broadphase collision detection using a uniform spatial hash. Only rigid-rigid
// pairs are emitted here (particle collision is handled separately). Each body
// writes its colliding partners into a private bucket, so the pass is lock-free.
void broadphase(const std::vector<Body *> &bodies, ThreadPool *pool,
                std::vector<std::vector<Body *>> &pairs)
{
    int n = (int)bodies.size();
    if (n < 2)
        return;

    std::vector<float> radii(n);
    for (int i = 0; i < n; ++i)
        radii[i] = bodies[i]->radius;
    std::vector<float> sorted = radii;
    std::nth_element(sorted.begin(), sorted.begin() + n / 2, sorted.end());
    float cell = sorted[n / 2] * 2.0f;
    if (!(cell > 1.0e-6f))
        cell = 1.0f;
    float invCell = 1.0f / cell;
    float largeRadius = cell * 2.0f;

    std::vector<int> small, large;
    float maxSmallRadius = 0.0f;
    for (int i = 0; i < n; ++i)
    {
        if (radii[i] > largeRadius)
            large.push_back(i);
        else
        {
            small.push_back(i);
            maxSmallRadius = max(maxSmallRadius, radii[i]);
        }
    }

    auto coord = [invCell](float v) { return (int)floorf(v * invCell); };

    // Open-addressed uniform-grid hash. The small bodies are bucketed into grid
    // cells by a counting sort into flat arrays (cellKey / cellStart / cellCount
    // plus a cell-grouped cellBody index list), so the per-body neighbourhood
    // scan does cache-friendly O(1) cell lookups. This replaces a node-based
    // std::unordered_map, whose per-lookup pointer chase dominated dense
    // particle scenes (cloth).
    int numSmall = (int)small.size();
    int cap = 1;
    while (cap < numSmall * 2 + 1)
        cap <<= 1;
    int mask = cap - 1;
    const uint64_t EMPTY_CELL = ~0ull; // packCell never sets bit 63, so this is free

    std::vector<uint64_t> cellKey(cap, EMPTY_CELL);
    std::vector<int> cellStart(cap, 0);
    std::vector<int> cellCount(cap, 0);
    std::vector<int> cellBody(numSmall);
    std::vector<uint64_t> keyOf(numSmall);

    // splitmix64 finaliser, then a linear probe to the slot holding `k` (or to
    // the empty slot where it belongs). Read-only after the build below, so it
    // is safe to call concurrently from the scan.
    auto slotOf = [&](uint64_t k) {
        uint64_t h = k;
        h ^= h >> 30;
        h *= 0xbf58476d1ce4e5b9ull;
        h ^= h >> 27;
        h *= 0x94d049bb133111ebull;
        h ^= h >> 31;
        int s = (int)(h & (uint64_t)mask);
        while (cellKey[s] != EMPTY_CELL && cellKey[s] != k)
            s = (s + 1) & mask;
        return s;
    };

    for (int si = 0; si < numSmall; ++si)
    {
        float3 p = bodies[small[si]]->positionLin;
        keyOf[si] = packCell(coord(p.x), coord(p.y), coord(p.z));
    }
    // Pass 1: register each occupied cell and count its occupants.
    for (int si = 0; si < numSmall; ++si)
    {
        int s = slotOf(keyOf[si]);
        cellKey[s] = keyOf[si];
        ++cellCount[s];
    }
    // Prefix sum -> per-cell start offset into cellBody.
    int acc = 0;
    for (int s = 0; s < cap; ++s)
    {
        cellStart[s] = acc;
        acc += cellCount[s];
    }
    // Pass 2: scatter body indices, grouped by cell (small-body order within a
    // cell, so the emitted pairs are deterministic).
    std::vector<int> cursor = cellStart;
    for (int si = 0; si < numSmall; ++si)
        cellBody[cursor[slotOf(keyOf[si])]++] = small[si];

    pool->parallelFor(numSmall, [&](int si) {
        int i = small[si];
        Body *a = bodies[i];
        float3 p = a->positionLin;
        int cx = coord(p.x), cy = coord(p.y), cz = coord(p.z);
        int range = (int)ceilf((a->radius + maxSmallRadius) * invCell) + 1;
        for (int dz = -range; dz <= range; ++dz)
            for (int dy = -range; dy <= range; ++dy)
                for (int dx = -range; dx <= range; ++dx)
                {
                    int s = slotOf(packCell(cx + dx, cy + dy, cz + dz));
                    if (cellKey[s] == EMPTY_CELL)
                        continue;
                    int start = cellStart[s], cnt = cellCount[s];
                    for (int t = 0; t < cnt; ++t)
                    {
                        int j = cellBody[start + t];
                        if (j <= i)
                            continue;
                        Body *b = bodies[j];
                        if (a->kind == BODY_PARTICLE && b->kind == BODY_PARTICLE)
                            continue; // no particle self-collision
                        float3 dp = p - b->positionLin;
                        float r = a->radius + b->radius;
                        if (dot(dp, dp) <= r * r && !a->constrainedTo(b))
                            pairs[i].push_back(b);
                    }
                }
    });

    pool->parallelFor((int)large.size(), [&](int li) {
        int i = large[li];
        Body *a = bodies[i];
        for (int j = 0; j < n; ++j)
        {
            if (j == i)
                continue;
            Body *b = bodies[j];
            if (a->kind == BODY_PARTICLE && b->kind == BODY_PARTICLE)
                continue; // no particle self-collision
            bool bLarge = b->radius > largeRadius;
            if (bLarge && j <= i)
                continue;
            float3 dp = a->positionLin - b->positionLin;
            float r = a->radius + b->radius;
            // constrainedTo walks the body's own force list, so query from `b`:
            // the large body `a` accumulates contacts from everything that lands
            // on it (thousands for a ground plane), while `b` keeps a short list.
            if (dot(dp, dp) <= r * r && !b->constrainedTo(a))
                pairs[i].push_back(b);
        }
    });
}
} // namespace

void Solver::step()
{
    // Per-phase profiling. When profileEnabled is off, lap() is a no-op and no
    // clock is read, so this has zero cost on the normal solver path.
    using profileClock = std::chrono::high_resolution_clock;
    profileClock::time_point profileTic;
    if (profileEnabled)
        profileTic = profileClock::now();
    auto lap = [&](double &acc) {
        if (!profileEnabled)
            return;
        profileClock::time_point now = profileClock::now();
        acc += std::chrono::duration<double, std::milli>(now - profileTic).count();
        profileTic = now;
    };

    // Gather all bodies into a contiguous array for indexed parallel access.
    std::vector<Body *> bodyList;
    for (Body *body = bodies; body != 0; body = body->next)
    {
        body->index = (int)bodyList.size();
        body->color = -1;
        bodyList.push_back(body);
    }
    int numBodies = (int)bodyList.size();

    // Cache each rigid body's world-space box axes once per step. collideOBB
    // (run per contact pair during force init) reads these instead of rotating
    // the basis vectors itself, which it otherwise repeats once per contact.
    pool->parallelFor(numBodies, [&](int i) {
        Body *body = bodyList[i];
        if (body->kind != BODY_RIGID)
            return;
        Rigid *r = (Rigid *)body;
        r->worldAxis[0] = rotate(r->positionAng, float3{1, 0, 0});
        r->worldAxis[1] = rotate(r->positionAng, float3{0, 1, 0});
        r->worldAxis[2] = rotate(r->positionAng, float3{0, 0, 1});
    });
    lap(profile.otherMs);

    // Broadphase collision detection; create the matching contact force for
    // each pair (rigid-rigid Manifold or particle-rigid ParticleContact).
    std::vector<std::vector<Body *>> pairs(numBodies);
    broadphase(bodyList, pool, pairs);
    for (int i = 0; i < numBodies; ++i)
        for (Body *bodyB : pairs[i])
        {
            Body *a = bodyList[i];
            if (a->kind == BODY_RIGID && bodyB->kind == BODY_RIGID)
                new Manifold(this, (Rigid *)a, (Rigid *)bodyB);
            else
            {
                Particle *p = (Particle *)(a->kind == BODY_PARTICLE ? a : bodyB);
                Rigid *r = (Rigid *)(a->kind == BODY_RIGID ? a : bodyB);
                new ParticleContact(this, p, r);
            }
        }
    lap(profile.broadphaseMs);

    // Initialize forces; an inactive force returns false. initialize() is the
    // narrow-phase collision recompute for contacts and is by far the heaviest
    // pre-solve cost, but each force only reads its own bodies (positions are
    // not mutated here) and writes its own state -- so run initialize() in
    // parallel over every force, then do the cheap linked-list surgery serially.
    std::vector<Force *> allForces;
    for (Force *force = forces; force != 0; force = force->next)
        allForces.push_back(force);
    int numAllForces = (int)allForces.size();
    lap(profile.forceGatherMs);

    // char, not vector<bool>: bit-packed bools would make adjacent parallel
    // writes share a byte and race.
    std::vector<char> forceActive(numAllForces);
    pool->parallelFor(numAllForces, [&](int i) {
        forceActive[i] = allForces[i]->initialize() ? 1 : 0;
    });

    // Drop inactive forces (serial: ~Force() unthreads the solver + per-body
    // lists). forceList ends up holding exactly the surviving forces.
    std::vector<Force *> forceList;
    forceList.reserve(numAllForces);
    for (int i = 0; i < numAllForces; ++i)
    {
        if (forceActive[i])
            forceList.push_back(allForces[i]);
        else
            delete allForces[i];
    }
    int numForces = (int)forceList.size();

    // Wake any sleeping body that shares a force with an awake dynamic body.
    if (sleepEnabled)
    {
        for (Force *force : forceList)
        {
            bool anyAwake = false;
            for (int k = 0; k < force->numBodies; ++k)
            {
                Body *b = force->bodies[k];
                if (b && b->mass > 0 && !b->asleep)
                    anyAwake = true;
            }
            if (!anyAwake)
                continue;
            for (int k = 0; k < force->numBodies; ++k)
            {
                Body *b = force->bodies[k];
                if (b && b->mass > 0 && b->asleep)
                {
                    b->asleep = false;
                    b->sleepTimer = 0.0f;
                }
            }
        }
    }

    lap(profile.initMs);

    // Initialize and warmstart bodies (primal variables).
    pool->parallelFor(numBodies, [&](int i) {
        Body *body = bodyList[i];
        if (body->asleep)
            return;

        // Inertial position (Eq. 2) + adaptive warmstart (original VBD paper).
        body->inertialLin = body->positionLin + body->velocityLin * dt;
        if (body->mass > 0)
            body->inertialLin += float3{0, 0, gravity} * (dt * dt);

        float3 accel = (body->velocityLin - body->prevVelocityLin) / dt;
        float accelWeight = clamp(accel.z * sign(gravity) / abs(gravity), 0.0f, 1.0f);
        if (!isfinite(accelWeight))
            accelWeight = 0.0f;

        body->initialLin = body->positionLin;
        if (body->mass > 0)
            body->positionLin = body->positionLin + body->velocityLin * dt + float3{0, 0, gravity} * (accelWeight * dt * dt);

        if (body->kind == BODY_RIGID)
        {
            Rigid *r = (Rigid *)body;
            r->inertialAng = r->positionAng + r->velocityAng * dt;
            r->initialAng = r->positionAng;
            if (r->mass > 0)
                r->positionAng = r->positionAng + r->velocityAng * dt;
        }
    });
    lap(profile.warmstartMs);

    // Graph-colour the dynamic bodies (colored Gauss-Seidel, AVBD Algorithm 1).
    std::vector<std::vector<Body *>> colorBuckets;
    for (Body *body : bodyList)
    {
        if (body->mass <= 0 || body->asleep)
            continue;

        std::vector<bool> used;
        for (Force *f = body->forces; f != 0; f = nextForce(f, body))
            for (int k = 0; k < f->numBodies; ++k)
            {
                Body *other = f->bodies[k];
                if (other && other != body && other->mass > 0 && other->color >= 0)
                {
                    if ((int)used.size() <= other->color)
                        used.resize(other->color + 1, false);
                    used[other->color] = true;
                }
            }

        int c = 0;
        while (c < (int)used.size() && used[c])
            ++c;
        body->color = c;
        if (c >= (int)colorBuckets.size())
            colorBuckets.resize(c + 1);
        colorBuckets[c].push_back(body);
    }
    lap(profile.coloringMs);

    // Main solver loop
    for (int it = 0; it < iterations; it++)
    {
        // Primal update: one colour at a time, bodies within a colour in parallel.
        for (std::vector<Body *> &bucket : colorBuckets)
        {
            int n = (int)bucket.size();
            pool->parallelFor(n, [&](int i) {
                Body *body = bucket[i];
                if (body->kind == BODY_RIGID)
                    updateRigidPrimal(this, (Rigid *)body);
                else
                    updateParticlePrimal(this, (Particle *)body);
            });
        }
        lap(profile.primalMs);

        // Dual update: every force is independent.
        pool->parallelFor(numForces, [&](int i) {
            Force *f = forceList[i];
            bool anyActive = false;
            for (int k = 0; k < f->numBodies; ++k)
            {
                Body *b = f->bodies[k];
                if (b && b->mass > 0 && !b->asleep)
                    anyActive = true;
            }
            if (anyActive)
                f->updateDual(alpha);
        });
        lap(profile.dualMs);
    }

    // Compute velocities (BDF1) after the final iteration.
    pool->parallelFor(numBodies, [&](int i) {
        Body *body = bodyList[i];
        if (body->asleep)
            return;
        body->prevVelocityLin = body->velocityLin;
        if (body->mass > 0)
        {
            body->velocityLin = (body->positionLin - body->initialLin) / dt;
            if (body->kind == BODY_RIGID)
            {
                Rigid *r = (Rigid *)body;
                r->velocityAng = (r->positionAng - r->initialAng) / dt;
            }
        }
    });

    // Sleep pass: freeze dynamic bodies that stayed at rest for sleepTime.
    if (sleepEnabled)
    {
        pool->parallelFor(numBodies, [&](int i) {
            Body *body = bodyList[i];
            if (body->mass <= 0 || body->asleep)
                return;
            float angSpeed = body->kind == BODY_RIGID ? length(((Rigid *)body)->velocityAng) : 0.0f;
            if (length(body->velocityLin) < sleepThresholdLin && angSpeed < sleepThresholdAng)
            {
                body->sleepTimer += dt;
                if (body->sleepTimer >= sleepTime)
                {
                    body->asleep = true;
                    body->velocityLin = float3{0, 0, 0};
                    body->prevVelocityLin = float3{0, 0, 0};
                    if (body->kind == BODY_RIGID)
                        ((Rigid *)body)->velocityAng = float3{0, 0, 0};
                }
            }
            else
            {
                body->sleepTimer = 0.0f;
            }
        });
    }

    lap(profile.otherMs);
    if (profileEnabled)
        ++profile.steps;
}

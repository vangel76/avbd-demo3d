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
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

Solver::Solver()
    : bodies(0), forces(0), threads(0), pool(0)
{
    defaultParams();
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

    // Ray-cast against each body's oriented box (the AABB extents for hull bodies)
    // by transforming the ray into body local space.
    for (Rigid *body = bodies; body != 0; body = body->next)
    {
        if (body->mass <= 0.0f)
            continue;

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
    // If the value is not in the right range, you may see slower convergance for complex scenes.
    // A minor upgrade from the paper is using separate betas for constraints of different units (eg linear vs angular).
    betaLin = 10000.0f;
    betaAng = 100.0f;

    // Alpha controls how much stabilization is applied. Higher values give slower and smoother
    // error correction, and lower values are more responsive and energetic. Tune this depending
    // on your desired constraint error response.
    alpha = 0.99f;

    // Gamma controls how much the penalty and lambda values are decayed each step during warmstarting.
    // This should always be < 1 so that the penalty values can decrease (unless you use a different
    // penalty parameter strategy which does not require decay).
    gamma = 0.999f;

    // 0 selects the hardware concurrency for the thread pool.
    threads = 0;

    // Sleeping is enabled by default. The thresholds are in simulation units
    // (length / second and radians / second); tune them via setSleeping.
    sleepEnabled = true;
    sleepThresholdLin = 0.05f;
    sleepThresholdAng = 0.05f;
    sleepTime = 0.5f;
}

namespace
{
// Solves the primal linear system for a single dynamic body and applies the
// position update (Eqs. 4-6). Shared by the serial and parallel paths so the
// solver behaviour is identical regardless of thread count.
void updateBodyPrimal(Solver *solver, Rigid *body)
{
    float dt = solver->dt;

    // Initialize left and right hand sides of the linear system (Eqs. 5, 6)
    float3x3 MLin = diagonal(body->mass, body->mass, body->mass);
    float3x3 MAng = diagonal(body->moment.x, body->moment.y, body->moment.z);

    float3x3 lhsLin = MLin / (dt * dt);
    float3x3 lhsAng = MAng / (dt * dt);
    float3x3 lhsCross = float3x3{0, 0, 0, 0, 0, 0, 0, 0, 0};

    float3 rhsLin = MLin / (dt * dt) * (body->positionLin - body->inertialLin);
    float3 rhsAng = MAng / (dt * dt) * (body->positionAng - body->inertialAng);

    // Iterate over all forces acting on the body
    for (Force *force = body->forces; force != 0; force = (force->bodyA == body) ? force->nextA : force->nextB)
        force->updatePrimal(body, solver->alpha, lhsLin, lhsAng, lhsCross, rhsLin, rhsAng);

    // Solve the SPD linear system using LDL and apply the update (Eq. 4)
    float3 dxLin, dxAng;
    solve(lhsLin, lhsAng, lhsCross, -rhsLin, -rhsAng, dxLin, dxAng);
    body->positionLin = body->positionLin + dxLin;
    body->positionAng = body->positionAng + dxAng;
}

// Packs integer cell coordinates into a 64-bit spatial-hash key (21 bits/axis).
inline uint64_t packCell(int x, int y, int z)
{
    uint64_t ux = (uint64_t)((unsigned)(x + 1048576)) & 0x1FFFFFu;
    uint64_t uy = (uint64_t)((unsigned)(y + 1048576)) & 0x1FFFFFu;
    uint64_t uz = (uint64_t)((unsigned)(z + 1048576)) & 0x1FFFFFu;
    return ux | (uy << 21) | (uz << 42);
}

// Broadphase collision detection using a uniform spatial hash. The cell size is
// derived from the median body radius. Bodies far larger than a cell (eg ground
// planes) would smear across the whole grid, so they are split into a separate
// list and tested against everything. Each body writes its colliding partners
// into a private bucket, so the whole pass is lock-free and parallelizable.
void broadphase(const std::vector<Rigid *> &bodies, ThreadPool *pool,
                std::vector<std::vector<Rigid *>> &pairs)
{
    int n = (int)bodies.size();
    if (n < 2)
        return;

    // Cell size from the median radius.
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

    // Split bodies into small (hashed) and large (tested against all).
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

    // Hash the small bodies by the cell containing their centre.
    std::unordered_map<uint64_t, std::vector<int>> grid;
    grid.reserve(small.size() * 2 + 1);
    for (int idx : small)
    {
        float3 p = bodies[idx]->positionLin;
        grid[packCell(coord(p.x), coord(p.y), coord(p.z))].push_back(idx);
    }

    // Small vs small: query the cells within reach of each small body.
    pool->parallelFor((int)small.size(), [&](int si) {
        int i = small[si];
        Rigid *a = bodies[i];
        float3 p = a->positionLin;
        int cx = coord(p.x), cy = coord(p.y), cz = coord(p.z);
        int range = (int)ceilf((a->radius + maxSmallRadius) * invCell) + 1;
        for (int dz = -range; dz <= range; ++dz)
            for (int dy = -range; dy <= range; ++dy)
                for (int dx = -range; dx <= range; ++dx)
                {
                    auto it = grid.find(packCell(cx + dx, cy + dy, cz + dz));
                    if (it == grid.end())
                        continue;
                    for (int j : it->second)
                    {
                        if (j <= i)
                            continue;
                        Rigid *b = bodies[j];
                        float3 dp = p - b->positionLin;
                        float r = a->radius + b->radius;
                        if (dot(dp, dp) <= r * r && !a->constrainedTo(b))
                            pairs[i].push_back(b);
                    }
                }
    });

    // Large bodies: test against every other body. A large/large pair is emitted
    // by the lower index; a large/small pair is emitted here (the small body's
    // hash query never sees large bodies).
    pool->parallelFor((int)large.size(), [&](int li) {
        int i = large[li];
        Rigid *a = bodies[i];
        for (int j = 0; j < n; ++j)
        {
            if (j == i)
                continue;
            Rigid *b = bodies[j];
            bool bLarge = b->radius > largeRadius;
            if (bLarge && j <= i)
                continue;
            float3 dp = a->positionLin - b->positionLin;
            float r = a->radius + b->radius;
            if (dot(dp, dp) <= r * r && !a->constrainedTo(b))
                pairs[i].push_back(b);
        }
    });
}
} // namespace

void Solver::step()
{
    // Gather the bodies into a contiguous array for indexed parallel access.
    std::vector<Rigid *> bodyList;
    for (Rigid *body = bodies; body != 0; body = body->next)
    {
        body->index = (int)bodyList.size();
        body->color = -1;
        bodyList.push_back(body);
    }
    int numBodies = (int)bodyList.size();

    // Broadphase collision detection via a spatial hash. Each body collects its
    // colliding partners into a private bucket (lock-free, parallelized), then
    // manifolds are created serially because they mutate the force linked lists.
    std::vector<std::vector<Rigid *>> pairs(numBodies);
    broadphase(bodyList, pool, pairs);
    for (int i = 0; i < numBodies; ++i)
        for (Rigid *bodyB : pairs[i])
            new Manifold(this, bodyList[i], bodyB);

    // Initialize and warmstart forces. Initialization can cache anything constant
    // over the step; a force returning false is inactive and removed.
    for (Force *force = forces; force != 0;)
    {
        if (!force->initialize())
        {
            Force *next = force->next;
            delete force;
            force = next;
        }
        else
            force = force->next;
    }

    // Gather the (post-initialization) forces into a contiguous array.
    std::vector<Force *> forceList;
    for (Force *force = forces; force != 0; force = force->next)
        forceList.push_back(force);
    int numForces = (int)forceList.size();

    // Wake any sleeping body that shares a constraint with an awake dynamic
    // body, so a disturbance propagates into resting regions over a few frames.
    if (sleepEnabled)
    {
        for (Force *force : forceList)
        {
            Rigid *a = force->bodyA;
            Rigid *b = force->bodyB;
            if (!a || !b)
                continue;
            if (a->mass > 0 && !a->asleep && b->mass > 0 && b->asleep)
            {
                b->asleep = false;
                b->sleepTimer = 0.0f;
            }
            if (b->mass > 0 && !b->asleep && a->mass > 0 && a->asleep)
            {
                a->asleep = false;
                a->sleepTimer = 0.0f;
            }
        }
    }

    // Initialize and warmstart bodies (ie primal variables). Each body is
    // independent here, so this runs in parallel.
    pool->parallelFor(numBodies, [&](int i) {
        Rigid *body = bodyList[i];
        if (body->asleep)
            return; // frozen bodies are not warmstarted

        // Compute inertial position (Eq 2)
        body->inertialLin = body->positionLin + body->velocityLin * dt;
        if (body->mass > 0)
            body->inertialLin += float3{0, 0, gravity} * (dt * dt);
        body->inertialAng = body->positionAng + body->velocityAng * dt;

        // Adaptive warmstart (See original VBD paper)
        float3 accel = (body->velocityLin - body->prevVelocityLin) / dt;
        float accelExt = accel.z * sign(gravity);
        float accelWeight = clamp(accelExt / abs(gravity), 0.0f, 1.0f);
        if (!isfinite(accelWeight))
            accelWeight = 0.0f;

        // Save initial position (x-) and compute warmstarted position (See original VBD paper)
        body->initialLin = body->positionLin;
        body->initialAng = body->positionAng;
        if (body->mass > 0)
        {
            body->positionLin = body->positionLin + body->velocityLin * dt + float3{0, 0, gravity} * (accelWeight * dt * dt);
            body->positionAng = body->positionAng + body->velocityAng * dt;
        }
    });

    // Graph-colour the dynamic bodies so that bodies sharing a constraint never
    // receive the same colour. Bodies of one colour can then be solved in
    // parallel (colored Gauss-Seidel, as in the AVBD paper, Algorithm 1).
    std::vector<std::vector<Rigid *>> colorBuckets;
    for (Rigid *body : bodyList)
    {
        if (body->mass <= 0 || body->asleep)
            continue; // static and sleeping bodies are not degrees of freedom

        // Mark the colours used by constrained neighbours.
        std::vector<bool> used;
        for (Force *f = body->forces; f != 0; f = (f->bodyA == body) ? f->nextA : f->nextB)
        {
            Rigid *other = (f->bodyA == body) ? f->bodyB : f->bodyA;
            if (other && other->mass > 0 && other->color >= 0)
            {
                if ((int)used.size() <= other->color)
                    used.resize(other->color + 1, false);
                used[other->color] = true;
            }
        }

        // Pick the lowest unused colour.
        int c = 0;
        while (c < (int)used.size() && used[c])
            ++c;
        body->color = c;
        if (c >= (int)colorBuckets.size())
            colorBuckets.resize(c + 1);
        colorBuckets[c].push_back(body);
    }

    // Main solver loop
    for (int it = 0; it < iterations; it++)
    {
        // Primal update: one colour at a time, the bodies within a colour in parallel.
        for (std::vector<Rigid *> &bucket : colorBuckets)
        {
            int n = (int)bucket.size();
            pool->parallelFor(n, [&](int i) { updateBodyPrimal(this, bucket[i]); });
        }

        // Dual update: every force is independent, so this runs fully in
        // parallel. Forces between only static/sleeping bodies are skipped.
        pool->parallelFor(numForces, [&](int i) {
            Force *f = forceList[i];
            Rigid *a = f->bodyA;
            Rigid *b = f->bodyB;
            bool aActive = a && a->mass > 0 && !a->asleep;
            bool bActive = b && b->mass > 0 && !b->asleep;
            if (aActive || bActive)
                f->updateDual(alpha);
        });
    }

    // Compute velocities (BDF1) after the final iteration.
    pool->parallelFor(numBodies, [&](int i) {
        Rigid *body = bodyList[i];
        if (body->asleep)
            return; // frozen bodies keep zero velocity
        body->prevVelocityLin = body->velocityLin;
        if (body->mass > 0)
        {
            body->velocityLin = (body->positionLin - body->initialLin) / dt;
            body->velocityAng = (body->positionAng - body->initialAng) / dt;
        }
    });

    // Sleep pass: freeze dynamic bodies that stayed below the rest velocity
    // thresholds for sleepTime seconds, eliminating resting jitter.
    if (sleepEnabled)
    {
        pool->parallelFor(numBodies, [&](int i) {
            Rigid *body = bodyList[i];
            if (body->mass <= 0 || body->asleep)
                return;
            if (length(body->velocityLin) < sleepThresholdLin &&
                length(body->velocityAng) < sleepThresholdAng)
            {
                body->sleepTimer += dt;
                if (body->sleepTimer >= sleepTime)
                {
                    body->asleep = true;
                    body->velocityLin = float3{0, 0, 0};
                    body->velocityAng = float3{0, 0, 0};
                    body->prevVelocityLin = float3{0, 0, 0};
                }
            }
            else
            {
                body->sleepTimer = 0.0f;
            }
        });
    }
}

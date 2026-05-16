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

// Particle-vs-rigid frictional contact. The rigid body is treated as its
// oriented bounding box for the closest-point query; this is exact for box
// bodies and an approximation for convex hulls. The contact is solved with the
// same augmented-Lagrangian + friction-cone model as Manifold, for one point.

#include "solver.h"
#include <cmath>

namespace
{
// Closest point on a rigid body's oriented box to a world point. Returns the
// closest point and the outward unit normal (both world space), and the signed
// distance (negative when the query point is inside the box).
void closestOnBox(const Rigid* r, float3 query, float3& closestWorld, float3& normalWorld, float& signedDist)
{
    quat invRot = conjugate(r->positionAng);
    float3 local = rotate(invRot, query - r->positionLin);
    float3 half = r->size * 0.5f;

    bool inside = fabsf(local.x) < half.x && fabsf(local.y) < half.y && fabsf(local.z) < half.z;
    float3 closestLocal;
    float3 normalLocal;

    if (inside)
    {
        // Push out along the axis of least penetration.
        float pen[3] = {half.x - fabsf(local.x), half.y - fabsf(local.y), half.z - fabsf(local.z)};
        int k = (pen[0] < pen[1]) ? (pen[0] < pen[2] ? 0 : 2) : (pen[1] < pen[2] ? 1 : 2);
        closestLocal = local;
        closestLocal[k] = local[k] >= 0.0f ? half[k] : -half[k];
        normalLocal = float3{0, 0, 0};
        normalLocal[k] = local[k] >= 0.0f ? 1.0f : -1.0f;
        signedDist = -pen[k];
    }
    else
    {
        closestLocal = clamp(local, -1.0f, 1.0f); // placeholder, overwritten below
        closestLocal = float3{clamp(local.x, -half.x, half.x),
                              clamp(local.y, -half.y, half.y),
                              clamp(local.z, -half.z, half.z)};
        float3 delta = local - closestLocal;
        float d = length(delta);
        normalLocal = d > 1.0e-9f ? delta / d : float3{0, 0, 1};
        signedDist = d;
    }

    closestWorld = r->positionLin + rotate(r->positionAng, closestLocal);
    normalWorld = rotate(r->positionAng, normalLocal);
}
} // namespace

ParticleContact::ParticleContact(Solver* solver, Particle* particle, Rigid* rigid)
    : Force(solver, particle, rigid), rB{0, 0, 0}, C0{0, 0, 0},
    penalty{0, 0, 0}, lambda{0, 0, 0}, friction(0.5f), active(false)
{
    basis = float3x3{1, 0, 0, 0, 1, 0, 0, 0, 1};
}

bool ParticleContact::initialize()
{
    Particle* p = (Particle*)bodies[0];
    Rigid* r = (Rigid*)bodies[1];

    friction = sqrtf(p->friction * r->friction);

    float3 closestWorld, normalWorld;
    float signedDist;
    closestOnBox(r, p->positionLin, closestWorld, normalWorld, signedDist);

    // Engage the contact once the particle's sphere is within the margin.
    float gap = signedDist - p->radius;
    active = gap < COLLISION_MARGIN;
    if (!active)
        return false;

    basis = orthonormal(normalWorld);
    rB = rotate(conjugate(r->positionAng), closestWorld - r->positionLin);

    // Constraint error at the start of the step (Sec. 4 Taylor approximation).
    C0 = basis * (p->positionLin - closestWorld) + float3{COLLISION_MARGIN - p->radius, 0, 0};

    // Warmstart the dual variables and penalty parameters (Eq. 19).
    lambda = lambda * solver->alpha * solver->gamma;
    penalty = clamp(penalty * solver->gamma, PENALTY_MIN, PENALTY_MAX);
    return true;
}

void ParticleContact::updatePrimal(Body* body, float alpha, float3x3& lhsLin, float3x3& lhsAng, float3x3& lhsCross, float3& rhsLin, float3& rhsAng)
{
    Particle* p = (Particle*)bodies[0];
    Rigid* r = (Rigid*)bodies[1];

    float3 dqPLin = p->positionLin - p->initialLin;
    float3 dqRLin = r->positionLin - r->initialLin;
    float3 dqRAng = r->positionAng - r->initialAng;

    float3 rBWorld = rotate(r->positionAng, rB);

    // Jacobians: particle is A (point, no angular), rigid is B.
    float3x3 jPLin = basis;
    float3x3 jRLin = -basis;
    float3x3 jRAng = float3x3{cross(rBWorld, jRLin[0]), cross(rBWorld, jRLin[1]), cross(rBWorld, jRLin[2])};

    float3x3 K = diagonal(penalty.x, penalty.y, penalty.z);
    float3 C = C0 * (1 - alpha) + jPLin * dqPLin + jRLin * dqRLin + jRAng * dqRAng;
    float3 F = K * C + lambda;

    // Clamp the normal force (no pulling) and the friction cone.
    F[0] = min(F[0], 0.0f);
    float bounds = fabsf(F[0]) * friction;
    float frictionScale = length(float2{F[1], F[2]});
    if (frictionScale > bounds && frictionScale > 0)
    {
        F[1] *= bounds / frictionScale;
        F[2] *= bounds / frictionScale;
    }

    if (body == bodies[0])
    {
        // Particle side: linear only.
        float3x3 jLinT = transpose(jPLin);
        lhsLin += jLinT * K * jPLin;
        rhsLin += jLinT * F;
    }
    else
    {
        // Rigid side: linear + angular.
        float3x3 jLinT = transpose(jRLin);
        float3x3 jAngT = transpose(jRAng);
        float3x3 jAngTk = jAngT * K;
        lhsLin += jLinT * K * jRLin;
        lhsAng += jAngTk * jRAng;
        lhsCross += jAngTk * jRLin;
        rhsLin += jLinT * F;
        rhsAng += jAngT * F;
    }
}

void ParticleContact::updateDual(float alpha)
{
    Particle* p = (Particle*)bodies[0];
    Rigid* r = (Rigid*)bodies[1];

    float3 dqPLin = p->positionLin - p->initialLin;
    float3 dqRLin = r->positionLin - r->initialLin;
    float3 dqRAng = r->positionAng - r->initialAng;
    float3 rBWorld = rotate(r->positionAng, rB);

    float3x3 jPLin = basis;
    float3x3 jRLin = -basis;
    float3x3 jRAng = float3x3{cross(rBWorld, jRLin[0]), cross(rBWorld, jRLin[1]), cross(rBWorld, jRLin[2])};

    float3x3 K = diagonal(penalty.x, penalty.y, penalty.z);
    float3 C = C0 * (1 - alpha) + jPLin * dqPLin + jRLin * dqRLin + jRAng * dqRAng;
    float3 F = K * C + lambda;

    F[0] = min(F[0], 0.0f);
    float bounds = fabsf(F[0]) * friction;
    float frictionScale = length(float2{F[1], F[2]});
    if (frictionScale > bounds && frictionScale > 0)
    {
        F[1] *= bounds / frictionScale;
        F[2] *= bounds / frictionScale;
    }

    lambda = F;

    // Ramp the penalty parameters (Eq. 12) while within the force bounds.
    if (F[0] < 0)
        penalty[0] = min(penalty[0] + solver->betaLin * fabsf(C[0]), PENALTY_MAX);
    if (frictionScale <= bounds)
    {
        penalty[1] = min(penalty[1] + solver->betaLin * fabsf(C[1]), PENALTY_MAX);
        penalty[2] = min(penalty[2] + solver->betaLin * fabsf(C[2]), PENALTY_MAX);
    }
}

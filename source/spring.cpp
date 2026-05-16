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

Spring::Spring(Solver* solver, Rigid* bodyA, Rigid* bodyB, float3 rA, float3 rB, float stiffness, float rest)
    : Force(solver, bodyA, bodyB), rA(rA), rB(rB), rest(rest), stiffness(stiffness), penalty(0.0f)
{
    if (this->rest < 0.0f)
    {
        float3 pA = transform(bodyA->positionLin, bodyA->positionAng, this->rA);
        float3 pB = transform(bodyB->positionLin, bodyB->positionAng, this->rB);
        this->rest = length(pA - pB);
    }
}

bool Spring::initialize()
{
    // Warmstart the penalty parameter (Eq. 19), decayed so it can shrink again,
    // and capped at the material stiffness. A spring is a finite-stiffness force,
    // so it has no dual variable: the ramped penalty is used directly (Sec. 3.4).
    penalty = clamp(penalty * solver->gamma, PENALTY_MIN, PENALTY_MAX);
    penalty = min(penalty, stiffness);
    return true;
}

void Spring::updatePrimal(Body* body, float alpha, float3x3& lhsLin, float3x3& lhsAng, float3x3& lhsCross, float3& rhsLin, float3& rhsAng)
{
    (void)alpha;
    Rigid* bodyA = (Rigid*)bodies[0];
    Rigid* bodyB = (Rigid*)bodies[1];

    float3 pA = transform(bodyA->positionLin, bodyA->positionAng, rA);
    float3 pB = transform(bodyB->positionLin, bodyB->positionAng, rB);
    float3 d = pA - pB;
    float dLen = length(d);
    if (dLen <= 1.0e-6f)
        return;

    float3 n = d / dLen;
    float C = dLen - rest;

    // Force uses the ramped penalty stiffness, not the full material stiffness.
    float f = penalty * C;

    float3 rWorld;
    float3 jLin;
    float3 jAng;
    if (body == bodyA)
    {
        rWorld = rotate(bodyA->positionAng, rA);
        jLin = n;
        jAng = cross(rWorld, n);
    }
    else
    {
        rWorld = rotate(bodyB->positionAng, rB);
        jLin = -n;
        jAng = -cross(rWorld, n);
    }

    float3 F = jLin * f;
    float3 Tau = jAng * f;

    // Material (Gauss-Newton) term of the Hessian.
    float3x3 Kll = outer(jLin, jLin) * penalty;
    float3x3 Kla = outer(jAng, jLin) * penalty;
    float3x3 Kaa = outer(jAng, jAng) * penalty;

    // Geometric stiffness of the linear block (Sec. 3.5): the spring direction
    // rotates as it deflects. Only added when in tension so the block stays SPD.
    float geo = f > 0.0f ? f / dLen : 0.0f;
    if (geo > 0.0f)
        Kll += (diagonal(1, 1, 1) - outer(n, n)) * geo;

    lhsLin += Kll;
    lhsAng += Kaa;
    lhsCross += Kla;
    rhsLin += F;
    rhsAng += Tau;
}

void Spring::updateDual(float alpha)
{
    (void)alpha;
    Rigid* bodyA = (Rigid*)bodies[0];
    Rigid* bodyB = (Rigid*)bodies[1];

    // Ramp the penalty stiffness toward the material stiffness based on the
    // current constraint violation (Eq. 16).
    float3 pA = transform(bodyA->positionLin, bodyA->positionAng, rA);
    float3 pB = transform(bodyB->positionLin, bodyB->positionAng, rB);
    float C = length(pA - pB) - rest;

    penalty = min(penalty + solver->betaLin * fabsf(C), min(stiffness, PENALTY_MAX));
}

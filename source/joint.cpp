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

inline float3x3 geometricStiffnessBallSocket(int k, float3 v)
{
    float3x3 m = diagonal(-v[k], -v[k], -v[k]);

    m[0][k] += v[0];
    m[1][k] += v[1];
    m[2][k] += v[2];

    return m;
}

Joint::Joint(Solver* solver, Rigid* bodyA, Rigid* bodyB, float3 rA, float3 rB, float stiffnessLin, float stiffnessAng, float fracture)
    : Force(solver, bodyA, bodyB), rA(rA), rB(rB), stiffnessLin(stiffnessLin), stiffnessAng(stiffnessAng), fracture(fracture), broken(false)
{
    this->penaltyLin = this->penaltyAng = float3{ 0, 0, 0 };
    this->lambdaLin = this->lambdaAng = float3{ 0, 0, 0 };
    this->torqueArm = lengthSq((bodyA ? bodyA->size : float3{ 0, 0, 0 }) + bodyB->size);
}

bool Joint::initialize()
{
    Rigid* bodyA = (Rigid*)bodies[0];
    Rigid* bodyB = (Rigid*)bodies[1];

    // Store constraint function at beginnning of timestep C(x-)
    // Note: if bodyA is null, it is assumed that the joint connects a body to the world space position rA
    C0Lin = (bodyA ? transform(bodyA->positionLin, bodyA->positionAng, rA) : rA) - transform(bodyB->positionLin, bodyB->positionAng, rB);
    C0Ang = ((bodyA ? bodyA->positionAng : quat{ 0, 0, 0, 1 }) - bodyB->positionAng) * torqueArm;

    // Warmstart the dual variables and penalty parameters (Eq. 19)
    // Penalty is safely clamped to a minimum and maximum value
    lambdaLin = lambdaLin * solver->alpha * solver->gamma;
    lambdaAng = lambdaAng * solver->alpha * solver->gamma;
    penaltyLin = clamp(penaltyLin * solver->gamma, PENALTY_MIN, PENALTY_MAX);
    penaltyAng = clamp(penaltyAng * solver->gamma, PENALTY_MIN, PENALTY_MAX);

    // Clamp penalty to material stiffness
    penaltyLin = min(penaltyLin, stiffnessLin);
    penaltyAng = min(penaltyAng, stiffnessAng);

    return !broken;
}

void Joint::updatePrimal(Body* body, float alpha, float3x3& lhsLin, float3x3& lhsAng, float3x3& lhsCross, float3& rhsLin, float3& rhsAng)
{
    Rigid* bodyA = (Rigid*)bodies[0];
    Rigid* bodyB = (Rigid*)bodies[1];

    // Linear constraint
    if (lengthSq(penaltyLin) > 0)
    {
        // Compute constraint and jacobians
        float3x3 K = diagonal(penaltyLin.x, penaltyLin.y, penaltyLin.z);
        float3 C = (bodyA ? transform(bodyA->positionLin, bodyA->positionAng, rA) : rA) - transform(bodyB->positionLin, bodyB->positionAng, rB);
        
        // Stabilization
        if (isinf(stiffnessLin))
            C -= C0Lin * alpha;

        // Compute force
        float3 F = K * C + lambdaLin;

        // Choose jacobian depending on input body
        float3x3 jLin = body == bodyA ? float3x3{ 1, 0, 0, 0, 1, 0, 0, 0, 1 } : float3x3{ -1, 0, 0, 0, -1, 0, 0, 0, -1 };
        float3x3 jAng = body == bodyA ? skew(-rotate(bodyA->positionAng, rA)) : skew(rotate(bodyB->positionAng, rB));

        // Stamp into LHS
        float3x3 jLinT = transpose(jLin);
        float3x3 jAngT = transpose(jAng);
        float3x3 jAngTk = jAngT * K;

        lhsLin += jLinT * K * jLin;
        lhsAng += jAngTk * jAng;
        lhsCross += jAngTk * jLin;

        // Diagonal approximation for higher order terms
        float3 r = body == bodyA ? rotate(bodyA->positionAng, rA) : -rotate(bodyB->positionAng, rB);
        float3x3 H = 
            geometricStiffnessBallSocket(0, r) * F[0] +
            geometricStiffnessBallSocket(1, r) * F[1] +
            geometricStiffnessBallSocket(2, r) * F[2];
        lhsAng += diagonalize(H);

        // Stamp into RHS
        rhsLin += jLinT * F;
        rhsAng += jAngT * F;
    }

    // Angular constraint
    if (lengthSq(penaltyAng) > 0)
    {
        // Compute constraint and jacobians
        float3x3 K = diagonal(penaltyAng.x, penaltyAng.y, penaltyAng.z);
        float3 C = ((bodyA ? bodyA->positionAng : quat{ 0, 0, 0, 1 }) - bodyB->positionAng) * torqueArm;

        // Stabilization
        if (isinf(stiffnessAng))
            C -= C0Ang * alpha;

        // Compute force
        float3 F = K * C + lambdaAng;

        // Choose jacobian depending on input body
        float3x3 jAng = (body == bodyA ? float3x3{ 1, 0, 0, 0, 1, 0, 0, 0, 1 } : float3x3{ -1, 0, 0, 0, -1, 0, 0, 0, -1 }) * torqueArm;

        // Stamp into LHS
        lhsAng += transpose(jAng) * K * jAng;

        // Stamp into RHS
        rhsAng += transpose(jAng) * F;
    }
}

void Joint::updateDual(float alpha)
{
    Rigid* bodyA = (Rigid*)bodies[0];
    Rigid* bodyB = (Rigid*)bodies[1];

    // Linear constraint
    if (lengthSq(penaltyLin) > 0)
    {
        // Compute constraint and jacobians
        float3x3 K = diagonal(penaltyLin.x, penaltyLin.y, penaltyLin.z);
        float3 C = (bodyA ? transform(bodyA->positionLin, bodyA->positionAng, rA) : rA) - transform(bodyB->positionLin, bodyB->positionAng, rB);

        if (isinf(stiffnessLin))
        {
            // Stabilization
            C -= C0Lin * alpha;

            // Compute force
            float3 F = K * C + lambdaLin;

            // Store updated force
            lambdaLin = F;
        }

        // Update the penalty parameter and clamp to material stiffness if we are within the force bounds (Eq. 16)
        penaltyLin = min(penaltyLin + abs(C) * solver->betaLin, min(stiffnessLin, PENALTY_MAX));
    }

    // Angular constraint
    if (lengthSq(penaltyAng) > 0)
    {
        // Compute constraint and jacobians
        float3x3 K = diagonal(penaltyAng.x, penaltyAng.y, penaltyAng.z);
        float3 C = ((bodyA ? bodyA->positionAng : quat{ 0, 0, 0, 1 }) - bodyB->positionAng) * torqueArm;

        if (isinf(stiffnessAng))
        {
            // Stabilization
            C -= C0Ang * alpha;

            // Compute force
            float3 F = K * C + lambdaAng;

            // Store updated force
            lambdaAng = F;
        }

        // Update the penalty parameter and clamp to material stiffness if we are within the force bounds (Eq. 16)
        penaltyAng = min(penaltyAng + abs(C) * solver->betaAng, min(stiffnessAng, PENALTY_MAX));
    }

    // Fracture test
    if (lengthSq(lambdaAng) > fracture * fracture)
    {
        penaltyLin = { 0, 0, 0 };
        penaltyAng = { 0, 0, 0 };
        lambdaLin = { 0, 0, 0 };
        lambdaAng = { 0, 0, 0 };
        broken = true;
    }
}

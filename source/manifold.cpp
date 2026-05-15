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

Manifold::Manifold(Solver *solver, Rigid *bodyA, Rigid *bodyB)
    : Force(solver, bodyA, bodyB), numContacts(0)
{
}

int Manifold::collide(Rigid *bodyA, Rigid *bodyB, Contact *contacts, float3x3 &basis)
{
    // Box-vs-box uses the fast tuned OBB path; anything involving a convex hull
    // goes through the general separating-axis convex routine.
    if (bodyA->hull || bodyB->hull)
        return collideConvex(bodyA, bodyB, contacts, basis);
    return collideOBB(bodyA, bodyB, contacts, basis);
}

bool Manifold::initialize()
{
    // Compute friction
    friction = sqrtf(bodyA->friction * bodyB->friction);

    // Compute new contacts
    Contact newContacts[8] = {0};
    int newNumContacts = collide(bodyA, bodyB, newContacts, basis);

    // Merge old contact data with new contacts
    for (int i = 0; i < newNumContacts; i++)
    {
        for (int j = 0; j < numContacts; j++)
        {
            if (newContacts[i].feature.key == contacts[j].feature.key)
            {
                float3 newRA = newContacts[i].rA;
                float3 newRB = newContacts[i].rB;
                newContacts[i] = contacts[j];

                // If no static friction in last frame, use the new contact point locations
                if (!contacts[j].stick)
                {
                    newContacts[i].rA = newRA;
                    newContacts[i].rB = newRB;
                }
                break;
            }
        }
    }

    // Copy new contacts to the manifold
    numContacts = newNumContacts;
    for (int i = 0; i < numContacts; i++)
        contacts[i] = newContacts[i];

    // Compute error at q- and update penalty and lambdas
    for (int i = 0; i < numContacts; i++)
    {
        // Error at q-
        float3 xA = transform(bodyA->positionLin, bodyA->positionAng, contacts[i].rA);
        float3 xB = transform(bodyB->positionLin, bodyB->positionAng, contacts[i].rB);
        contacts[i].C0 = basis * (xA - xB) + float3{COLLISION_MARGIN, 0, 0};

        // Warmstart the dual variables and penalty parameters (Eq. 19)
        // Penalty is safely clamped to a minimum and maximum value
        contacts[i].lambda = contacts[i].lambda * solver->alpha * solver->gamma;
        contacts[i].penalty = clamp(contacts[i].penalty * solver->gamma, PENALTY_MIN, PENALTY_MAX);
    }

    return numContacts > 0;
}

void Manifold::updatePrimal(Rigid *body, float alpha, float3x3 &lhsLin, float3x3 &lhsAng, float3x3 &lhsCross, float3 &rhsLin, float3 &rhsAng)
{
    float3 dqALin = bodyA->positionLin - bodyA->initialLin;
    float3 dqAAng = bodyA->positionAng - bodyA->initialAng;
    float3 dqBLin = bodyB->positionLin - bodyB->initialLin;
    float3 dqBAng = bodyB->positionAng - bodyB->initialAng;

    for (int i = 0; i < numContacts; i++)
    {
        float3 rAWorld = rotate(bodyA->positionAng, contacts[i].rA);
        float3 rBWorld = rotate(bodyB->positionAng, contacts[i].rB);

        // Compute the Taylor series approximation of the constraint function C(x) (Sec 4)
        float3x3 jALin = basis;
        float3x3 jBLin = -basis;
        float3x3 jAAng = float3x3{cross(rAWorld, jALin[0]), cross(rAWorld, jALin[1]), cross(rAWorld, jALin[2])};
        float3x3 jBAng = float3x3{cross(rBWorld, jBLin[0]), cross(rBWorld, jBLin[1]), cross(rBWorld, jBLin[2])};

        float3x3 K = diagonal(contacts[i].penalty.x, contacts[i].penalty.y, contacts[i].penalty.z);
        float3 C = contacts[i].C0 * (1 - alpha) + jALin * dqALin + jBLin * dqBLin + jAAng * dqAAng + jBAng * dqBAng;

        // Compute force before clamping (lambda+ in Eq. 13)
        float3 Fpre = K * C + contacts[i].lambda;
        float3 F = Fpre;

        // Clamp normal force
        F[0] = min(F[0], 0.0f);

        // Clamp norm of friction forces to achieve a friction cone
        float bounds = fabsf(F[0]) * friction;
        float frictionScale = length(float2{F[1], F[2]});
        if (frictionScale > bounds && frictionScale > 0)
        {
            F[1] *= bounds / frictionScale;
            F[2] *= bounds / frictionScale;
        }

        // Stiffness rescaling for the Hessian (Eq. 14). When the normal force is
        // clamped (the contact wants to separate, lambda_max = 0), the true
        // Hessian of the clamped force is zero. Using the full penalty stiffness
        // there over-stiffens boundary contacts and destabilizes tall stacks;
        // instead we rescale it so k * C matches the clamped force. Friction
        // keeps the simple unclamped Hessian, as in the AVBD paper.
        float3 Klhs = contacts[i].penalty;
        if (Fpre[0] > 0.0f && fabsf(C[0]) > 1.0e-9f)
        {
            float kRescaled = -contacts[i].lambda[0] / C[0];
            Klhs[0] = clamp(kRescaled, PENALTY_MIN, contacts[i].penalty[0]);
        }
        float3x3 Kh = diagonal(Klhs.x, Klhs.y, Klhs.z);

        // Choose jacobian depending on input body
        float3x3 jLin = body == bodyA ? jALin : jBLin;
        float3x3 jAng = body == bodyA ? jAAng : jBAng;

        // Stamp into LHS (using the rescaled stiffness, Eq. 14)
        float3x3 jLinT = transpose(jLin);
        float3x3 jAngT = transpose(jAng);
        float3x3 jAngTk = jAngT * Kh;

        lhsLin += jLinT * Kh * jLin;
        lhsAng += jAngTk * jAng;
        lhsCross += jAngTk * jLin;

        // Stamp into RHS
        rhsLin += jLinT * F;
        rhsAng += jAngT * F;
    }
}

void Manifold::updateDual(float alpha)
{
    float3 dqALin = bodyA->positionLin - bodyA->initialLin;
    float3 dqAAng = bodyA->positionAng - bodyA->initialAng;
    float3 dqBLin = bodyB->positionLin - bodyB->initialLin;
    float3 dqBAng = bodyB->positionAng - bodyB->initialAng;

    for (int i = 0; i < numContacts; i++)
    {
        float3 rAWorld = rotate(bodyA->positionAng, contacts[i].rA);
        float3 rBWorld = rotate(bodyB->positionAng, contacts[i].rB);

        // Compute the Taylor series approximation of the constraint function C(x) (Sec 4)
        float3x3 jALin = basis;
        float3x3 jBLin = -basis;
        float3x3 jAAng = float3x3{cross(rAWorld, jALin[0]), cross(rAWorld, jALin[1]), cross(rAWorld, jALin[2])};
        float3x3 jBAng = float3x3{cross(rBWorld, jBLin[0]), cross(rBWorld, jBLin[1]), cross(rBWorld, jBLin[2])};

        float3x3 K = diagonal(contacts[i].penalty.x, contacts[i].penalty.y, contacts[i].penalty.z);
        float3 C = contacts[i].C0 * (1 - alpha) + jALin * dqALin + jBLin * dqBLin + jAAng * dqAAng + jBAng * dqBAng;

        // Compute force
        float3 F = K * C + contacts[i].lambda;

        // Clamp normal force
        F[0] = min(F[0], 0.0f);

        // Clamp norm of friction forces to achieve a friction cone
        float bounds = fabsf(F[0]) * friction;
        float frictionScale = length(float2{F[1], F[2]});
        if (frictionScale > bounds && frictionScale > 0)
        {
            F[1] *= bounds / frictionScale;
            F[2] *= bounds / frictionScale;
        }

        // Store updated force
        contacts[i].lambda = F;

        // Update the penalty parameter and clamp to material stiffness if we are within the force bounds (Eq. 16)
        if (F[0] < 0)
            contacts[i].penalty[0] = min(contacts[i].penalty[0] + solver->betaLin * fabsf(C[0]), PENALTY_MAX);
        if (frictionScale <= bounds)
        {
            contacts[i].penalty[1] = min(contacts[i].penalty[1] + solver->betaLin * fabsf(C[1]), PENALTY_MAX);
            contacts[i].penalty[2] = min(contacts[i].penalty[2] + solver->betaLin * fabsf(C[2]), PENALTY_MAX);
            contacts[i].stick = length(float2{C[1], C[2]}) < STICK_THRESH;
        }
    }
}

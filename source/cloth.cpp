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

// Triangle-FEM cloth for AVBD: a St-Venant-Kirchhoff membrane element per
// triangle (FEMTriangle) plus a quadratic bending element per interior edge
// (BendEdge). Both are AVBD force elements over 3-DOF particles; their force
// and a diagonal SPD Hessian (Sec. 3.5) are stamped into the particle solve.

#include "solver.h"
#include <cmath>
#include <map>
#include <vector>

// ---------------------------------------------------------------------------
// FEMTriangle - St-Venant-Kirchhoff membrane element
// ---------------------------------------------------------------------------

FEMTriangle::FEMTriangle(Solver* solver, Particle* p0, Particle* p1, Particle* p2, float mu, float lambda)
    : Force(solver, p0, p1, p2), mu(mu), lambda(lambda)
{
    // Flatten the rest triangle into its own 2D plane.
    float3 e1 = p1->positionLin - p0->positionLin;
    float3 e2 = p2->positionLin - p0->positionLin;
    float3 normal = cross(e1, e2);
    float area2 = length(normal);
    restArea = 0.5f * area2;

    float3 ax = lengthSq(e1) > 1.0e-20f ? normalize(e1) : float3{ 1, 0, 0 };
    float3 az = area2 > 1.0e-20f ? normal / area2 : float3{ 0, 0, 1 };
    float3 ay = cross(az, ax);

    // Rest edge vectors in 2D (u0 is the origin).
    float2 u1{ dot(e1, ax), dot(e1, ay) };
    float2 u2{ dot(e2, ax), dot(e2, ay) };

    // Rest shape matrix Dm has the rest edges as columns; invert it.
    float det = u1.x * u2.y - u2.x * u1.y;
    float invDet = fabsf(det) > 1.0e-20f ? 1.0f / det : 0.0f;
    dmInv[0] = float2{ u2.y * invDet, -u2.x * invDet };
    dmInv[1] = float2{ -u1.y * invDet, u1.x * invDet };

    // Shape-function gradients: g1/g2 are the rows of dmInv, g0 closes the sum.
    grad[1] = dmInv[0];
    grad[2] = dmInv[1];
    grad[0] = float2{ -(grad[1].x + grad[2].x), -(grad[1].y + grad[2].y) };
}

void FEMTriangle::updatePrimal(Body* body, float alpha, float3x3& lhsLin, float3x3& lhsAng, float3x3& lhsCross, float3& rhsLin, float3& rhsAng)
{
    (void)alpha; (void)lhsAng; (void)lhsCross; (void)rhsAng;
    int s = slotOf(body);
    if (s < 0)
        return;

    // Deformation gradient F (3x2) from the current edge vectors.
    float3 d1 = bodies[1]->positionLin - bodies[0]->positionLin;
    float3 d2 = bodies[2]->positionLin - bodies[0]->positionLin;
    float3 Fc0 = d1 * dmInv[0][0] + d2 * dmInv[1][0];
    float3 Fc1 = d1 * dmInv[0][1] + d2 * dmInv[1][1];

    // Green strain G = 0.5 (F^T F - I), 2x2 symmetric.
    float g00 = 0.5f * (dot(Fc0, Fc0) - 1.0f);
    float g01 = 0.5f * dot(Fc0, Fc1);
    float g11 = 0.5f * (dot(Fc1, Fc1) - 1.0f);
    float trG = g00 + g11;

    // Second Piola-Kirchhoff stress S = 2 mu G + lambda tr(G) I.
    float s00 = 2.0f * mu * g00 + lambda * trG;
    float s01 = 2.0f * mu * g01;
    float s11 = 2.0f * mu * g11 + lambda * trG;

    // First Piola stress P = F S (3x2).
    float3 Pc0 = Fc0 * s00 + Fc1 * s01;
    float3 Pc1 = Fc0 * s01 + Fc1 * s11;

    // Energy gradient w.r.t. the vertex being solved: dE/dx_s = A * P * grad_s.
    float3 gradient = (Pc0 * grad[s].x + Pc1 * grad[s].y) * restArea;
    rhsLin += gradient;

    // Diagonal SPD Hessian approximation (Sec. 3.5).
    float k = restArea * (2.0f * mu + lambda) * dot(grad[s], grad[s]);
    lhsLin += diagonal(k, k, k);
}

// ---------------------------------------------------------------------------
// BendEdge - quadratic hinge bending element
// ---------------------------------------------------------------------------

namespace
{
// Stencil weights: the bending vector is x2 + x3 - x0 - x1.
const float kBendWeight[4] = { -1.0f, -1.0f, 1.0f, 1.0f };
}

BendEdge::BendEdge(Solver* solver, Particle* p0, Particle* p1, Particle* p2, Particle* p3, float stiffness)
    : Force(solver, p0, p1, p2, p3), stiffness(stiffness)
{
    rest = p2->positionLin + p3->positionLin - p0->positionLin - p1->positionLin;
}

void BendEdge::updatePrimal(Body* body, float alpha, float3x3& lhsLin, float3x3& lhsAng, float3x3& lhsCross, float3& rhsLin, float3& rhsAng)
{
    (void)alpha; (void)lhsAng; (void)lhsCross; (void)rhsAng;
    int s = slotOf(body);
    if (s < 0)
        return;

    float3 current = bodies[2]->positionLin + bodies[3]->positionLin
                   - bodies[0]->positionLin - bodies[1]->positionLin;
    float3 violation = current - rest;

    // E = 0.5 k |violation|^2 ; dE/dx_s = k w_s violation ; Hessian = k w_s^2 I.
    float w = kBendWeight[s];
    rhsLin += violation * (stiffness * w);
    float k = stiffness * w * w;
    lhsLin += diagonal(k, k, k);
}

// ---------------------------------------------------------------------------
// Cloth - builds particles and elements from a triangle mesh
// ---------------------------------------------------------------------------

Cloth::Cloth(Solver* solver, const float3* verts, int numVerts,
             const int* triangles, int numTriangles,
             float density, float thickness, float youngsModulus, float poisson,
             float bendStiffness, float particleRadius, float friction)
    : solver(solver), particles(0), numParticles(numVerts)
{
    // Plane-stress Lame parameters from Young's modulus and Poisson's ratio.
    float nu = clamp(poisson, 0.0f, 0.49f);
    float mu = youngsModulus / (2.0f * (1.0f + nu));
    float lambda = youngsModulus * nu / (1.0f - nu * nu);

    // Lump triangle mass onto vertices (one third to each corner).
    std::vector<float> mass(numVerts, 0.0f);
    for (int t = 0; t < numTriangles; ++t)
    {
        int a = triangles[t * 3 + 0], b = triangles[t * 3 + 1], c = triangles[t * 3 + 2];
        float3 area = cross(verts[b] - verts[a], verts[c] - verts[a]);
        float triMass = 0.5f * length(area) * thickness * density;
        mass[a] += triMass / 3.0f;
        mass[b] += triMass / 3.0f;
        mass[c] += triMass / 3.0f;
    }

    // Create the particles.
    particles = new Particle*[numVerts];
    for (int i = 0; i < numVerts; ++i)
        particles[i] = new Particle(solver, verts[i], mass[i], particleRadius, friction);

    // Membrane element per triangle.
    for (int t = 0; t < numTriangles; ++t)
    {
        int a = triangles[t * 3 + 0], b = triangles[t * 3 + 1], c = triangles[t * 3 + 2];
        new FEMTriangle(solver, particles[a], particles[b], particles[c], mu, lambda);
    }

    // Bending element per interior edge (an edge shared by exactly two triangles).
    // Map each undirected edge to the opposite vertex of its first triangle.
    std::map<std::pair<int, int>, int> edgeOpposite;
    for (int t = 0; t < numTriangles; ++t)
    {
        int v[3] = { triangles[t * 3 + 0], triangles[t * 3 + 1], triangles[t * 3 + 2] };
        for (int e = 0; e < 3; ++e)
        {
            int a = v[e], b = v[(e + 1) % 3], opp = v[(e + 2) % 3];
            std::pair<int, int> key(a < b ? a : b, a < b ? b : a);
            auto it = edgeOpposite.find(key);
            if (it == edgeOpposite.end())
            {
                edgeOpposite[key] = opp;
            }
            else
            {
                // Second triangle on this edge: create the bending element.
                new BendEdge(solver, particles[key.first], particles[key.second],
                             particles[it->second], particles[opp], bendStiffness);
                it->second = -1; // mark consumed (ignore non-manifold extra triangles)
            }
        }
    }
}

Cloth::~Cloth()
{
    // The particles themselves are owned by the solver; free only the array.
    delete[] particles;
}

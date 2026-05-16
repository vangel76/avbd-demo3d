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
#include <cfloat>

Rigid::Rigid(Solver* solver, float3 size, float density, float friction, float3 position, float3 velocity)
    : Body(BODY_RIGID, solver),
    positionAng{ 0, 0, 0, 1 }, initialAng{ 0, 0, 0, 1 }, inertialAng{ 0, 0, 0, 1 },
    velocityAng{ 0, 0, 0 }, size(size), hull(0)
{
    positionLin = position;
    velocityLin = velocity;
    prevVelocityLin = velocity;
    this->friction = friction;

    // Compute mass properties and bounding radius
    mass = size.x * size.y * size.z * density;
    moment = float3 {
        (size.y * size.y + size.z * size.z) / 12.0f * mass,
        (size.x * size.x + size.z * size.z) / 12.0f * mass,
        (size.x * size.x + size.y * size.y) / 12.0f * mass
    };
    radius = length(size * 0.5f);
}

Rigid::Rigid(Solver* solver, ConvexHull* hull, float density, float friction, float3 position, float3 velocity)
    : Body(BODY_RIGID, solver),
    positionAng{ 0, 0, 0, 1 }, initialAng{ 0, 0, 0, 1 }, inertialAng{ 0, 0, 0, 1 },
    velocityAng{ 0, 0, 0 }, size{ 0, 0, 0 }, hull(hull)
{
    positionLin = position;
    velocityLin = velocity;
    prevVelocityLin = velocity;
    this->friction = friction;

    // Integrate mass properties over the polyhedron, then shift the hull so its
    // centre of mass sits at the body origin (AVBD tracks the centre of mass).
    float3 com;
    computeHullMassProperties(hull, density, mass, com, moment);
    for (int i = 0; i < hull->numVerts; i++)
        hull->verts[i] -= com;

    // Recompute bounds about the (now centred) origin.
    hull->radius = 0.0f;
    float3 lo{ FLT_MAX, FLT_MAX, FLT_MAX }, hi{ -FLT_MAX, -FLT_MAX, -FLT_MAX };
    for (int i = 0; i < hull->numVerts; i++)
    {
        hull->radius = max(hull->radius, length(hull->verts[i]));
        lo.x = min(lo.x, hull->verts[i].x); hi.x = max(hi.x, hull->verts[i].x);
        lo.y = min(lo.y, hull->verts[i].y); hi.y = max(hi.y, hull->verts[i].y);
        lo.z = min(lo.z, hull->verts[i].z); hi.z = max(hi.z, hull->verts[i].z);
    }
    hull->aabbHalf = float3{ max(fabsf(lo.x), fabsf(hi.x)), max(fabsf(lo.y), fabsf(hi.y)), max(fabsf(lo.z), fabsf(hi.z)) };
    size = hull->aabbHalf * 2.0f;
    radius = hull->radius;
}

Rigid::~Rigid()
{
    // Free the collision hull (null for box bodies). The Body destructor removes
    // this body from the solver list.
    delete hull;
}

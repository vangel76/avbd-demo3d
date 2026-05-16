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

Body::Body(BodyKind kind, Solver* solver)
    : kind(kind), solver(solver), forces(0),
    positionLin{0, 0, 0}, initialLin{0, 0, 0}, inertialLin{0, 0, 0},
    velocityLin{0, 0, 0}, prevVelocityLin{0, 0, 0},
    mass(0.0f), radius(0.0f), friction(0.5f),
    index(0), color(-1), asleep(false), sleepTimer(0.0f)
{
    // Add to the solver body list
    next = solver->bodies;
    solver->bodies = this;
}

Body::~Body()
{
    // Remove from the solver body list
    Body** p = &solver->bodies;
    while (*p != this)
        p = &(*p)->next;
    *p = next;
}

bool Body::constrainedTo(const Body* other) const
{
    for (const Force* f = forces; f != 0; f = f->bodyNext[f->slotOf(this)])
        for (int i = 0; i < f->numBodies; i++)
            if (f->bodies[i] == other)
                return true;
    return false;
}

Particle::Particle(Solver* solver, float3 position, float mass, float radius, float friction)
    : Body(BODY_PARTICLE, solver)
{
    this->positionLin = position;
    this->mass = mass;
    this->radius = radius;
    this->friction = friction;
}

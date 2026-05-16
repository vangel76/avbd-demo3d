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

int Force::slotOf(const Body* body) const
{
    for (int i = 0; i < numBodies; i++)
        if (bodies[i] == body)
            return i;
    return -1;
}

// Threads this force into the solver list and into each connected body's force
// list (one linked-list link per body slot).
void Force::linkLists()
{
    next = solver->forces;
    solver->forces = this;
    for (int i = 0; i < numBodies; i++)
    {
        if (bodies[i])
        {
            bodyNext[i] = bodies[i]->forces;
            bodies[i]->forces = this;
        }
    }
}

Force::Force(Solver* solver, Body* bodyA, Body* bodyB)
    : solver(solver), numBodies(2)
{
    bodies[0] = bodyA;
    bodies[1] = bodyB;
    for (int i = 2; i < MAX_FORCE_BODIES; i++)
        bodies[i] = 0;
    for (int i = 0; i < MAX_FORCE_BODIES; i++)
        bodyNext[i] = 0;
    linkLists();
}

Force::Force(Solver* solver, Body* bodyA, Body* bodyB, Body* bodyC)
    : solver(solver), numBodies(3)
{
    bodies[0] = bodyA;
    bodies[1] = bodyB;
    bodies[2] = bodyC;
    bodies[3] = 0;
    for (int i = 0; i < MAX_FORCE_BODIES; i++)
        bodyNext[i] = 0;
    linkLists();
}

Force::Force(Solver* solver, Body* bodyA, Body* bodyB, Body* bodyC, Body* bodyD)
    : solver(solver), numBodies(4)
{
    bodies[0] = bodyA;
    bodies[1] = bodyB;
    bodies[2] = bodyC;
    bodies[3] = bodyD;
    for (int i = 0; i < MAX_FORCE_BODIES; i++)
        bodyNext[i] = 0;
    linkLists();
}

Force::~Force()
{
    // Remove from the solver force list
    Force** p = &solver->forces;
    while (*p != this)
        p = &(*p)->next;
    *p = next;

    // Remove from each connected body's force list
    for (int i = 0; i < numBodies; i++)
    {
        Body* b = bodies[i];
        if (!b)
            continue;
        Force** q = &b->forces;
        while (*q != this)
            q = &(*q)->bodyNext[(*q)->slotOf(b)];
        *q = bodyNext[i];
    }
}

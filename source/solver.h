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

#pragma once

// NOTE: This header is part of the headless AVBD core library. It must not pull in
// any windowing, OpenGL, or UI dependencies so it can be compiled into avbd_core and
// the avbd shared library. Rendering includes live in the demo (main.cpp) instead.

#include "maths.h"

#define PENALTY_MIN 1.0f           // Minimum penalty parameter
#define PENALTY_MAX 10000000000.0f // Maximum penalty parameter
#define COLLISION_MARGIN 0.01f     // Margin for collision detection to avoid flickering contacts
#define STICK_THRESH 0.00001f      // Position threshold for sticking contacts (ie static friction)
#define SHOW_CONTACTS true         // Whether to show contacts in the debug draw

struct Rigid;
struct Force;
struct Manifold;
struct Solver;
class ThreadPool;

// An immutable convex polyhedron used as a collision shape. Vertices are stored in
// body-local space, with the centre of mass at the origin. Face and edge connectivity
// is kept so the separating-axis test and contact clipping can run on arbitrary
// convex shapes, not just boxes. A box body has hull == nullptr (see Rigid::hull).
struct ConvexHull
{
    int numVerts;
    int numFaces;
    int numEdges;
    float3 *verts;       // [numVerts]   local-space vertices, centre of mass at origin
    float3 *faceNormals; // [numFaces]   outward unit normals
    int *faceStart;      // [numFaces+1] offsets into faceVerts
    int *faceVerts;      // [.]          per-face vertex indices, wound CCW about the normal
    int *edges;          // [numEdges*2] unique undirected edges as vertex-index pairs
    float radius;        // bounding-sphere radius about the origin
    float3 aabbHalf;     // local axis-aligned half extents about the origin

    ~ConvexHull();

    // Builds a hull from polygon faces. faceVertCounts[f] gives the vertex count of
    // face f and faceIndices concatenates the per-face vertex indices. Face normals,
    // edges, winding, and bounds are derived. Returns nullptr on degenerate input.
    static ConvexHull *create(const float3 *points, int numPoints,
                              const int *faceVertCounts, const int *faceIndices, int numFaces);

    // Builds an axis-aligned box hull of the given full-width size.
    static ConvexHull *createBox(float3 size);
};

// Integrates mass, centre of mass, and the (diagonal-approximated) inertia tensor
// of a convex polyhedron of uniform density. Used by the convex Rigid constructor.
void computeHullMassProperties(const ConvexHull *hull, float density,
                               float &massOut, float3 &comOut, float3 &momentOut);

// Holds all the state for a single rigid body that is needed by AVBD
struct Rigid
{
    Solver *solver;
    Force *forces;
    Rigid *next;
    float3 positionLin;
    quat positionAng;
    float3 initialLin;
    quat initialAng;
    float3 inertialLin;
    quat inertialAng;
    float3 velocityLin;
    float3 velocityAng;
    float3 prevVelocityLin;
    float3 size; // Full widths in each dimension (AABB extents for a hull body)
    float mass;
    float3 moment;
    float friction;
    float radius;
    ConvexHull *hull; // Convex collision shape; nullptr means an oriented box of `size`
    int index;        // Solver scratch: position in the per-step body array
    int color;        // Solver scratch: graph-colouring group (-1 for static bodies)
    bool asleep;      // Sleeping bodies are frozen and skipped by the solver until disturbed
    float sleepTimer; // Seconds spent continuously below the rest velocity threshold

    // Box body: mass properties are derived analytically from `size` and `density`.
    Rigid(Solver *solver, float3 size, float density, float friction, float3 position, float3 velocity = float3{0, 0, 0});

    // Convex-hull body: takes ownership of `hull`. Mass, centre of mass, and inertia
    // are integrated over the polyhedron; `position` is the world centre of mass.
    Rigid(Solver *solver, ConvexHull *hull, float density, float friction, float3 position, float3 velocity = float3{0, 0, 0});

    ~Rigid();

    bool constrainedTo(Rigid *other) const;
};

// Holds all user defined and derived constraint parameters, and provides a common interface for all forces.
struct Force
{
    Solver *solver;
    Rigid *bodyA;
    Rigid *bodyB;
    Force *nextA;
    Force *nextB;
    Force *next;

    Force(Solver *solver, Rigid *bodyA, Rigid *bodyB);
    virtual ~Force();

    virtual bool initialize() = 0;
    virtual void updatePrimal(Rigid *body, float alpha, float3x3 &lhsLin, float3x3 &lhsAng, float3x3 &lhsCross, float3 &rhsLin, float3 &rhsAng) = 0;
    virtual void updateDual(float alpha) = 0;
};

// Revolute joint + angle constraint between two rigid bodies, with optional fracture
struct Joint : Force
{
    float3 rA, rB;
    float3 C0Lin, C0Ang;
    float3 penaltyLin, penaltyAng;
    float3 lambdaLin, lambdaAng;
    float stiffnessLin, stiffnessAng, fracture;
    float torqueArm;
    bool broken;

    Joint(Solver *solver, Rigid *bodyA, Rigid *bodyB, float3 rA, float3 rB, float stiffnessLin = INFINITY, float stiffnessAng = 0.0f, float fracture = INFINITY);

    bool initialize() override;
    void updatePrimal(Rigid *body, float alpha, float3x3 &lhsLin, float3x3 &lhsAng, float3x3 &lhsCross, float3 &rhsLin, float3 &rhsAng) override;
    void updateDual(float alpha) override;
};

// Standard spring force
struct Spring : Force
{
    float3 rA, rB;
    float rest;
    float stiffness;

    Spring(Solver *solver, Rigid *bodyA, Rigid *bodyB, float3 rA, float3 rB, float stiffness, float rest = -1);

    bool initialize() override { return true; }
    void updatePrimal(Rigid *body, float alpha, float3x3 &lhsLin, float3x3 &lhsAng, float3x3 &lhsCross, float3 &rhsLin, float3 &rhsAng) override;
    void updateDual(float alpha) override;
};

// Force which has no physical effect, but is used to ignore collisions between two bodies
struct IgnoreCollision : Force
{
    IgnoreCollision(Solver *solver, Rigid *bodyA, Rigid *bodyB)
        : Force(solver, bodyA, bodyB) {}

    bool initialize() override { return true; }
    void updatePrimal(Rigid *body, float alpha, float3x3 &lhsLin, float3x3 &lhsAng, float3x3 &lhsCross, float3 &rhsLin, float3 &rhsAng) override {}
    void updateDual(float alpha) override {}
};

// Collision manifold between two rigid bodies, which contains up to eight frictional contact points
struct Manifold : Force
{
    // Used to track contact features between frames
    union FeaturePair
    {
        struct
        {
            char inR;
            char outR;
            char inI;
            char outI;
        };

        int key;
    };

    // Contact point information for a single contact
    struct Contact
    {
        FeaturePair feature;
        float3 rA; // contact offset in A's local space (relative to center)
        float3 rB; // contact offset in B's local space (relative to center)
        float3 C0;
        float3 penalty;
        float3 lambda;
        bool stick;
    };

    Contact contacts[8];
    float3x3 basis; // Normal in the first row (pointing from B to A), and tangents in the second and third rows
    int numContacts;
    float friction;

    Manifold(Solver *solver, Rigid *bodyA, Rigid *bodyB);

    bool initialize() override;
    void updatePrimal(Rigid *body, float alpha, float3x3 &lhsLin, float3x3 &lhsAng, float3x3 &lhsCross, float3 &rhsLin, float3 &rhsAng) override;
    void updateDual(float alpha) override;

    static int collide(Rigid *bodyA, Rigid *bodyB, Contact *contacts, float3x3 &basis);
};

// Narrow-phase collision routines. Manifold::collide dispatches to collideOBB when
// both bodies are boxes (fast tuned path) and collideConvex otherwise.
int collideOBB(Rigid *bodyA, Rigid *bodyB, Manifold::Contact *contacts, float3x3 &basis);
int collideConvex(Rigid *bodyA, Rigid *bodyB, Manifold::Contact *contacts, float3x3 &basis);

// Core solver class which holds all the rigid bodies and forces, and has logic to step the simulation forward in time
struct Solver
{
    float dt;       // Timestep
    float gravity;  // Gravity
    int iterations; // Solver iterations

    float alpha; // Stabilization parameter
    float betaLin;  // Penalty ramping parameter for linear constraints
    float betaAng;  // Penalty ramping parameter for angular constraints
    float gamma; // Warmstarting decay parameter

    // Sleeping: bodies that stay below the rest velocity thresholds for
    // sleepTime seconds are frozen and skipped by the solver until a moving
    // body disturbs them. This removes resting jitter and speeds up settled scenes.
    bool sleepEnabled;
    float sleepThresholdLin; // Linear speed below which a body counts as resting
    float sleepThresholdAng; // Angular speed below which a body counts as resting
    float sleepTime;         // Time below the thresholds before a body sleeps

    Rigid *bodies;
    Force *forces;

    int threads;      // Worker lane count for the solver (0 = hardware concurrency)
    ThreadPool *pool; // Persistent thread pool; rebuilt by setThreads

    Solver();
    ~Solver();

    Rigid *pick(float3 origin, float3 dir, float3 &local);
    void clear();
    void defaultParams();
    void step();

    // Sets the CPU thread count and rebuilds the thread pool. n = 0 selects the
    // hardware concurrency; n = 1 runs the solver single-threaded.
    void setThreads(int n);
};

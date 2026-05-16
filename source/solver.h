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
#define MAX_FORCE_BODIES 4         // Maximum number of bodies a single force element connects

struct Body;
struct Rigid;
struct Particle;
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

// Distinguishes the two degree-of-freedom kinds the solver supports.
enum BodyKind
{
    BODY_RIGID,   // 6-DOF rigid body (linear + angular)
    BODY_PARTICLE // 3-DOF point mass (cloth / soft-body vertex)
};

// Base for any simulated body. Holds the linear degrees of freedom and solver
// bookkeeping shared by 6-DOF rigid bodies and 3-DOF particles.
struct Body
{
    BodyKind kind;
    Solver *solver;
    Force *forces; // head of this body's force list (threaded via Force::bodyNext)
    Body *next;    // solver body list
    float3 positionLin;
    float3 initialLin;
    float3 inertialLin;
    float3 velocityLin;
    float3 prevVelocityLin;
    float mass;
    float radius;     // bounding / collision sphere radius
    float friction;   // Coulomb friction coefficient
    int index;        // Solver scratch: position in the per-step body array
    int color;        // Solver scratch: graph-colouring group (-1 for static bodies)
    bool asleep;      // Sleeping bodies are frozen and skipped by the solver until disturbed
    float sleepTimer; // Seconds spent continuously below the rest velocity threshold

    Body(BodyKind kind, Solver *solver);
    virtual ~Body();

    // True if this body shares any force with `other`.
    bool constrainedTo(const Body *other) const;
};

// A 6-DOF rigid body: the linear DOF of Body plus orientation and angular motion.
struct Rigid : Body
{
    quat positionAng;
    quat initialAng;
    quat inertialAng;
    float3 velocityAng;
    float3 size; // Full widths in each dimension (AABB extents for a hull body)
    float3 moment;
    ConvexHull *hull; // Convex collision shape; nullptr means an oriented box of `size`

    // Box body: mass properties are derived analytically from `size` and `density`.
    Rigid(Solver *solver, float3 size, float density, float friction, float3 position, float3 velocity = float3{0, 0, 0});

    // Convex-hull body: takes ownership of `hull`. Mass, centre of mass, and inertia
    // are integrated over the polyhedron; `position` is the world centre of mass.
    Rigid(Solver *solver, ConvexHull *hull, float density, float friction, float3 position, float3 velocity = float3{0, 0, 0});

    ~Rigid() override;
};

// A 3-DOF point mass used for cloth and soft-body vertices.
struct Particle : Body
{
    Particle(Solver *solver, float3 position, float mass, float radius, float friction = 0.5f);
};

// Holds all user defined and derived constraint parameters, and provides a common
// interface for all forces. A force connects up to MAX_FORCE_BODIES bodies.
struct Force
{
    Solver *solver;
    Body *bodies[MAX_FORCE_BODIES];   // connected bodies (first numBodies are valid)
    Force *bodyNext[MAX_FORCE_BODIES]; // per-body force-list links
    int numBodies;
    Force *next; // solver force list

    Force(Solver *solver, Body *bodyA, Body *bodyB);
    Force(Solver *solver, Body *bodyA, Body *bodyB, Body *bodyC);
    Force(Solver *solver, Body *bodyA, Body *bodyB, Body *bodyC, Body *bodyD);
    virtual ~Force();

    // Index of `body` within bodies[], or -1 if not connected.
    int slotOf(const Body *body) const;

    // Threads this force into the solver and per-body linked lists.
    void linkLists();

    virtual bool initialize() = 0;
    virtual void updatePrimal(Body *body, float alpha, float3x3 &lhsLin, float3x3 &lhsAng, float3x3 &lhsCross, float3 &rhsLin, float3 &rhsAng) = 0;
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
    void updatePrimal(Body *body, float alpha, float3x3 &lhsLin, float3x3 &lhsAng, float3x3 &lhsCross, float3 &rhsLin, float3 &rhsAng) override;
    void updateDual(float alpha) override;
};

// Spring force, modelled as an AVBD finite-stiffness force: the penalty
// stiffness ramps up to the material stiffness over iterations (Eq. 16), which
// is what lets VBD converge with high stiffness ratios (paper Sec. 3.4).
struct Spring : Force
{
    float3 rA, rB;
    float rest;
    float stiffness; // material (target) stiffness
    float penalty;   // ramped penalty stiffness used by the solver

    Spring(Solver *solver, Rigid *bodyA, Rigid *bodyB, float3 rA, float3 rB, float stiffness, float rest = -1);

    bool initialize() override;
    void updatePrimal(Body *body, float alpha, float3x3 &lhsLin, float3x3 &lhsAng, float3x3 &lhsCross, float3 &rhsLin, float3 &rhsAng) override;
    void updateDual(float alpha) override;
};

// Force which has no physical effect, but is used to ignore collisions between two bodies
struct IgnoreCollision : Force
{
    IgnoreCollision(Solver *solver, Rigid *bodyA, Rigid *bodyB)
        : Force(solver, bodyA, bodyB) {}

    bool initialize() override { return true; }
    void updatePrimal(Body *body, float alpha, float3x3 &lhsLin, float3x3 &lhsAng, float3x3 &lhsCross, float3 &rhsLin, float3 &rhsAng) override {}
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
    void updatePrimal(Body *body, float alpha, float3x3 &lhsLin, float3x3 &lhsAng, float3x3 &lhsCross, float3 &rhsLin, float3 &rhsAng) override;
    void updateDual(float alpha) override;

    static int collide(Rigid *bodyA, Rigid *bodyB, Contact *contacts, float3x3 &basis);
};

// Narrow-phase collision routines. Manifold::collide dispatches to collideOBB when
// both bodies are boxes (fast tuned path) and collideConvex otherwise.
int collideOBB(Rigid *bodyA, Rigid *bodyB, Manifold::Contact *contacts, float3x3 &basis);
int collideConvex(Rigid *bodyA, Rigid *bodyB, Manifold::Contact *contacts, float3x3 &basis);

// Triangle membrane element: a St-Venant-Kirchhoff continuum-mechanics energy
// over the 2D in-plane strain of a triangle of 3 particles. Resists stretch and
// shear automatically; bending is handled separately by BendEdge.
struct FEMTriangle : Force
{
    float2x2 dmInv;  // inverse rest shape matrix
    float2 grad[3];  // shape-function gradients (per triangle vertex)
    float restArea;
    float mu;     // Lame shear modulus
    float lambda; // Lame first parameter

    FEMTriangle(Solver *solver, Particle *p0, Particle *p1, Particle *p2, float mu, float lambda);

    bool initialize() override { return true; }
    void updatePrimal(Body *body, float alpha, float3x3 &lhsLin, float3x3 &lhsAng, float3x3 &lhsCross, float3 &rhsLin, float3 &rhsAng) override;
    void updateDual(float alpha) override {}
};

// Bending element across an interior cloth edge: a quadratic energy on the four
// vertices (the shared edge + the two opposite vertices) that is zero in the
// rest configuration and resists folding of the hinge.
struct BendEdge : Force
{
    float3 rest;     // rest value of the bending stencil (x2 + x3 - x0 - x1)
    float stiffness; // bending stiffness

    BendEdge(Solver *solver, Particle *p0, Particle *p1, Particle *p2, Particle *p3, float stiffness);

    bool initialize() override { return true; }
    void updatePrimal(Body *body, float alpha, float3x3 &lhsLin, float3x3 &lhsAng, float3x3 &lhsCross, float3 &rhsLin, float3 &rhsAng) override;
    void updateDual(float alpha) override {}
};

// Frictional contact between a particle and a rigid body. The rigid is treated
// as its oriented bounding box for the closest-point query (exact for box
// bodies, an approximation for convex hulls). Augmented-Lagrangian like Manifold.
struct ParticleContact : Force
{
    float3 rB;      // contact point on the rigid, in its local space
    float3 C0;      // constraint error at the start of the step
    float3 penalty; // penalty parameters (normal, tangent, tangent)
    float3 lambda;  // dual variables / contact force
    float3x3 basis; // row 0 = contact normal (rigid -> particle), rows 1-2 tangents
    float friction;
    bool active;

    ParticleContact(Solver *solver, Particle *particle, Rigid *rigid);

    bool initialize() override;
    void updatePrimal(Body *body, float alpha, float3x3 &lhsLin, float3x3 &lhsAng, float3x3 &lhsCross, float3 &rhsLin, float3 &rhsAng) override;
    void updateDual(float alpha) override;
};

// A triangle-mesh cloth: builds particles, FEMTriangle membrane elements, and
// BendEdge bending elements from a triangle mesh. The particles are owned by the
// solver; this struct keeps the ordered particle array for read-back.
struct Cloth
{
    Solver *solver;
    Particle **particles;
    int numParticles;

    // verts: numVerts world-space positions. triangles: 3 indices per triangle.
    Cloth(Solver *solver, const float3 *verts, int numVerts,
          const int *triangles, int numTriangles,
          float density, float thickness, float youngsModulus, float poisson,
          float bendStiffness, float particleRadius, float friction);
    ~Cloth();
};

// Core solver class which holds all the bodies and forces, and has logic to step the simulation forward in time
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

    Body *bodies;
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

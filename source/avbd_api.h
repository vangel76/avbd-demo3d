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

#ifndef AVBD_API_H
#define AVBD_API_H

/*
 * Flat C API for the AVBD physics core, exported from the `avbd` shared library.
 *
 * It is designed to be loaded from Python via ctypes (see addon/avbd_native.py),
 * so every type crossing the boundary is a plain pointer, int, or float. The API
 * is backend-agnostic: a future GPU backend can implement the same entry points.
 *
 * Vectors are passed as float[3]; quaternions as float[4] in (x, y, z, w) order.
 * All handles are owned by the solver and freed by avbd_solver_destroy / _clear.
 */

#if defined(_WIN32)
#define AVBD_API __declspec(dllexport)
#else
#define AVBD_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C"
{
#endif

    /* Opaque handles. */
    typedef struct AvbdSolver AvbdSolver;
    typedef struct AvbdBody AvbdBody;
    typedef struct AvbdForce AvbdForce;
    typedef struct AvbdCloth AvbdCloth;

    /* --- Solver lifecycle ------------------------------------------------- */

    AVBD_API AvbdSolver *avbd_solver_create(void);
    AVBD_API void avbd_solver_destroy(AvbdSolver *solver);
    AVBD_API void avbd_solver_clear(AvbdSolver *solver);
    AVBD_API void avbd_solver_step(AvbdSolver *solver);

    /* Sets the CPU worker count (0 = hardware concurrency, 1 = single-threaded). */
    AVBD_API void avbd_solver_set_threads(AvbdSolver *solver, int threads);

    /*
     * Configures body sleeping. Bodies that stay below linThreshold (linear,
     * units/s) and angThreshold (angular, rad/s) for timeToSleep seconds are
     * frozen and skipped until a moving body disturbs them. enabled = 0 disables.
     */
    AVBD_API void avbd_solver_set_sleeping(AvbdSolver *solver, int enabled,
                                           float linThreshold, float angThreshold,
                                           float timeToSleep);

    /* Solver tuning parameters (see Solver in solver.h for the meaning of each). */
    AVBD_API void avbd_solver_set_params(AvbdSolver *solver, float dt, float gravity,
                                         int iterations, float alpha, float betaLin,
                                         float betaAng, float gamma);
    AVBD_API void avbd_solver_get_params(AvbdSolver *solver, float *dt, float *gravity,
                                         int *iterations, float *alpha, float *betaLin,
                                         float *betaAng, float *gamma);

    /* --- Bodies ----------------------------------------------------------- */

    /* Adds an oriented box body. density <= 0 makes the body static/kinematic. */
    AVBD_API AvbdBody *avbd_add_box(AvbdSolver *solver, const float *size,
                                    float density, float friction,
                                    const float *position, const float *velocity);

    /*
     * Adds a convex-hull body. `verts` is numVerts * 3 floats. `faceVertCounts`
     * gives the vertex count of each of `numFaces` faces and `faceIndices`
     * concatenates the per-face vertex indices. `position` is the world centre
     * of mass. Returns NULL if the hull is degenerate.
     */
    AVBD_API AvbdBody *avbd_add_convex(AvbdSolver *solver, const float *verts, int numVerts,
                                       const int *faceVertCounts, const int *faceIndices,
                                       int numFaces, float density, float friction,
                                       const float *position, const float *velocity);

    AVBD_API void avbd_body_get_transform(AvbdBody *body, float *position, float *orientation);

    /*
     * Batched transform read: writes 7 floats per body into `out`
     * ([px,py,pz, qx,qy,qz,qw] per body). One call replaces N per-body calls,
     * which matters when streaming hundreds of bodies back to a host each frame.
     */
    AVBD_API void avbd_get_transforms(AvbdBody **bodies, int count, float *out);

    AVBD_API void avbd_body_set_transform(AvbdBody *body, const float *position, const float *orientation);
    AVBD_API void avbd_body_get_velocity(AvbdBody *body, float *linear, float *angular);
    AVBD_API void avbd_body_set_velocity(AvbdBody *body, const float *linear, const float *angular);
    AVBD_API float avbd_body_get_mass(AvbdBody *body);

    /* --- Forces / constraints -------------------------------------------- */

    /* Revolute joint + angle constraint. Pass INFINITY stiffness for a hard joint. */
    AVBD_API AvbdForce *avbd_add_joint(AvbdSolver *solver, AvbdBody *bodyA, AvbdBody *bodyB,
                                       const float *rA, const float *rB,
                                       float stiffnessLin, float stiffnessAng, float fracture);

    AVBD_API AvbdForce *avbd_add_spring(AvbdSolver *solver, AvbdBody *bodyA, AvbdBody *bodyB,
                                        const float *rA, const float *rB,
                                        float stiffness, float rest);

    /* Suppresses collision response between two bodies. */
    AVBD_API AvbdForce *avbd_add_ignore_collision(AvbdSolver *solver, AvbdBody *bodyA, AvbdBody *bodyB);

    /* Returns non-zero if a fracturing joint has broken. */
    AVBD_API int avbd_joint_is_broken(AvbdForce *force);

    /* Returns non-zero if the body is currently asleep (frozen). */
    AVBD_API int avbd_body_is_asleep(AvbdBody *body);

    /* --- Cloth (triangle FEM) -------------------------------------------- */

    /*
     * Adds a triangle-FEM cloth. `verts` is numVerts * 3 floats (world space),
     * `triangles` is numTriangles * 3 vertex indices. Material is given by
     * Young's modulus, Poisson's ratio, and a separate bending stiffness.
     * Returns a handle owning the ordered particle list for read-back.
     */
    AVBD_API AvbdCloth *avbd_add_cloth(AvbdSolver *solver, const float *verts, int numVerts,
                                       const int *triangles, int numTriangles,
                                       float density, float thickness, float youngsModulus,
                                       float poisson, float bendStiffness, float particleRadius,
                                       float friction);

    AVBD_API int avbd_cloth_vertex_count(AvbdCloth *cloth);

    /* Writes the current cloth vertex positions into `out` (numVerts * 3 floats). */
    AVBD_API void avbd_cloth_get_vertices(AvbdCloth *cloth, float *out);

    /* Pins a cloth vertex in place (makes that particle static). */
    AVBD_API void avbd_cloth_pin_vertex(AvbdCloth *cloth, int index);

    /* Frees the cloth handle. The particles themselves are freed with the solver. */
    AVBD_API void avbd_cloth_destroy(AvbdCloth *cloth);

#ifdef __cplusplus
}
#endif

#endif /* AVBD_API_H */

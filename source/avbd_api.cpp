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

// Implementation of the flat C API declared in avbd_api.h. This is the only
// translation unit of the `avbd` shared library; it links the avbd_core static
// library and re-exports the solver as C entry points for ctypes.

#include "avbd_api.h"
#include "solver.h"

namespace
{
inline Solver *S(AvbdSolver *h) { return reinterpret_cast<Solver *>(h); }
inline Rigid *B(AvbdBody *h) { return reinterpret_cast<Rigid *>(h); }
inline Force *F(AvbdForce *h) { return reinterpret_cast<Force *>(h); }
inline Cloth *CL(AvbdCloth *h) { return reinterpret_cast<Cloth *>(h); }
inline float3 V3(const float *p) { return float3{p[0], p[1], p[2]}; }
} // namespace

extern "C"
{

    AvbdSolver *avbd_solver_create(void)
    {
        return reinterpret_cast<AvbdSolver *>(new Solver());
    }

    void avbd_solver_destroy(AvbdSolver *solver)
    {
        delete S(solver);
    }

    void avbd_solver_clear(AvbdSolver *solver)
    {
        S(solver)->clear();
    }

    void avbd_solver_step(AvbdSolver *solver)
    {
        S(solver)->step();
    }

    void avbd_solver_set_threads(AvbdSolver *solver, int threads)
    {
        S(solver)->setThreads(threads);
    }

    void avbd_solver_set_sleeping(AvbdSolver *solver, int enabled,
                                  float linThreshold, float angThreshold,
                                  float timeToSleep)
    {
        Solver *s = S(solver);
        s->sleepEnabled = enabled != 0;
        s->sleepThresholdLin = linThreshold;
        s->sleepThresholdAng = angThreshold;
        s->sleepTime = timeToSleep;
    }

    void avbd_solver_set_params(AvbdSolver *solver, float dt, float gravity,
                                int iterations, float alpha, float betaLin,
                                float betaAng, float gamma)
    {
        Solver *s = S(solver);
        s->dt = dt;
        s->gravity = gravity;
        s->iterations = iterations;
        s->alpha = alpha;
        s->betaLin = betaLin;
        s->betaAng = betaAng;
        s->gamma = gamma;
    }

    void avbd_solver_get_params(AvbdSolver *solver, float *dt, float *gravity,
                                int *iterations, float *alpha, float *betaLin,
                                float *betaAng, float *gamma)
    {
        Solver *s = S(solver);
        *dt = s->dt;
        *gravity = s->gravity;
        *iterations = s->iterations;
        *alpha = s->alpha;
        *betaLin = s->betaLin;
        *betaAng = s->betaAng;
        *gamma = s->gamma;
    }

    AvbdBody *avbd_add_box(AvbdSolver *solver, const float *size,
                           float density, float friction,
                           const float *position, const float *velocity)
    {
        Rigid *body = new Rigid(S(solver), V3(size), density, friction,
                                V3(position), V3(velocity));
        return reinterpret_cast<AvbdBody *>(body);
    }

    AvbdBody *avbd_add_convex(AvbdSolver *solver, const float *verts, int numVerts,
                              const int *faceVertCounts, const int *faceIndices,
                              int numFaces, float density, float friction,
                              const float *position, const float *velocity)
    {
        ConvexHull *hull = ConvexHull::create(reinterpret_cast<const float3 *>(verts),
                                              numVerts, faceVertCounts, faceIndices, numFaces);
        if (!hull)
            return nullptr;
        Rigid *body = new Rigid(S(solver), hull, density, friction,
                                V3(position), V3(velocity));
        return reinterpret_cast<AvbdBody *>(body);
    }

    void avbd_body_get_transform(AvbdBody *body, float *position, float *orientation)
    {
        Rigid *b = B(body);
        position[0] = b->positionLin.x;
        position[1] = b->positionLin.y;
        position[2] = b->positionLin.z;
        orientation[0] = b->positionAng.x;
        orientation[1] = b->positionAng.y;
        orientation[2] = b->positionAng.z;
        orientation[3] = b->positionAng.w;
    }

    void avbd_get_transforms(AvbdBody **bodies, int count, float *out)
    {
        for (int i = 0; i < count; ++i)
        {
            Rigid *b = B(bodies[i]);
            float *o = out + i * 7;
            o[0] = b->positionLin.x;
            o[1] = b->positionLin.y;
            o[2] = b->positionLin.z;
            o[3] = b->positionAng.x;
            o[4] = b->positionAng.y;
            o[5] = b->positionAng.z;
            o[6] = b->positionAng.w;
        }
    }

    void avbd_body_set_transform(AvbdBody *body, const float *position, const float *orientation)
    {
        Rigid *b = B(body);
        b->positionLin = V3(position);
        b->positionAng = quat{orientation[0], orientation[1], orientation[2], orientation[3]};
    }

    void avbd_body_get_velocity(AvbdBody *body, float *linear, float *angular)
    {
        Rigid *b = B(body);
        linear[0] = b->velocityLin.x;
        linear[1] = b->velocityLin.y;
        linear[2] = b->velocityLin.z;
        angular[0] = b->velocityAng.x;
        angular[1] = b->velocityAng.y;
        angular[2] = b->velocityAng.z;
    }

    void avbd_body_set_velocity(AvbdBody *body, const float *linear, const float *angular)
    {
        Rigid *b = B(body);
        b->velocityLin = V3(linear);
        b->prevVelocityLin = b->velocityLin;
        b->velocityAng = V3(angular);
    }

    float avbd_body_get_mass(AvbdBody *body)
    {
        return B(body)->mass;
    }

    AvbdForce *avbd_add_joint(AvbdSolver *solver, AvbdBody *bodyA, AvbdBody *bodyB,
                              const float *rA, const float *rB,
                              float stiffnessLin, float stiffnessAng, float fracture)
    {
        Joint *joint = new Joint(S(solver), B(bodyA), B(bodyB), V3(rA), V3(rB),
                                 stiffnessLin, stiffnessAng, fracture);
        return reinterpret_cast<AvbdForce *>(static_cast<Force *>(joint));
    }

    AvbdForce *avbd_add_spring(AvbdSolver *solver, AvbdBody *bodyA, AvbdBody *bodyB,
                               const float *rA, const float *rB,
                               float stiffness, float rest)
    {
        Spring *spring = new Spring(S(solver), B(bodyA), B(bodyB), V3(rA), V3(rB),
                                    stiffness, rest);
        return reinterpret_cast<AvbdForce *>(static_cast<Force *>(spring));
    }

    AvbdForce *avbd_add_ignore_collision(AvbdSolver *solver, AvbdBody *bodyA, AvbdBody *bodyB)
    {
        IgnoreCollision *ignore = new IgnoreCollision(S(solver), B(bodyA), B(bodyB));
        return reinterpret_cast<AvbdForce *>(static_cast<Force *>(ignore));
    }

    int avbd_joint_is_broken(AvbdForce *force)
    {
        // Valid only for handles returned by avbd_add_joint.
        Joint *joint = static_cast<Joint *>(F(force));
        return joint->broken ? 1 : 0;
    }

    int avbd_body_is_asleep(AvbdBody *body)
    {
        return B(body)->asleep ? 1 : 0;
    }

    AvbdCloth *avbd_add_cloth(AvbdSolver *solver, const float *verts, int numVerts,
                              const int *triangles, int numTriangles,
                              float density, float thickness, float youngsModulus,
                              float poisson, float bendStiffness, float particleRadius,
                              float friction)
    {
        Cloth *cloth = new Cloth(S(solver), reinterpret_cast<const float3 *>(verts), numVerts,
                                 triangles, numTriangles, density, thickness, youngsModulus,
                                 poisson, bendStiffness, particleRadius, friction);
        return reinterpret_cast<AvbdCloth *>(cloth);
    }

    int avbd_cloth_vertex_count(AvbdCloth *cloth)
    {
        return CL(cloth)->numParticles;
    }

    void avbd_cloth_get_vertices(AvbdCloth *cloth, float *out)
    {
        Cloth *c = CL(cloth);
        for (int i = 0; i < c->numParticles; ++i)
        {
            out[i * 3 + 0] = c->particles[i]->positionLin.x;
            out[i * 3 + 1] = c->particles[i]->positionLin.y;
            out[i * 3 + 2] = c->particles[i]->positionLin.z;
        }
    }

    void avbd_cloth_pin_vertex(AvbdCloth *cloth, int index)
    {
        Cloth *c = CL(cloth);
        if (index >= 0 && index < c->numParticles)
            c->particles[index]->mass = 0.0f;
    }

    void avbd_cloth_destroy(AvbdCloth *cloth)
    {
        delete CL(cloth);
    }

} // extern "C"

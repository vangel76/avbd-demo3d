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

#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <map>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

#ifdef TARGET_OS_MAC
#include <OpenGL/GL.h>
#else
#include <GL/gl.h>
#endif

#include <SDL2/SDL.h>
#include <imgui.h>
#include <backends/imgui_impl_sdl2.h>
#include <backends/imgui_impl_opengl3.h>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include "maths.h"
#include "solver.h"
#include "scenes.h"

#define WinWidth 1280
#define WinHeight 720

bool Running = 1;
bool FullScreen = 0;
SDL_Window *Window;
SDL_GLContext Context;
int WindowFlags = SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE;

Solver *solver = new Solver();
Joint *drag = 0;
int currScene = 4;
float3 boxSize = {1, 1, 1};
float boxVelocity = 10.0f;
float boxFriction = 0.5f;
float boxDensity = 1.0f;
bool paused = false;
bool shootRequested = false;
Uint32 lastTapTicks = 0;
float2 lastTapPos = {0, 0};

float camDistance = 50.0f;
float camAzimuth = rad(90.0f);
float camElevation = 0.35f;
float3 camTarget = {0, 0, 5.0f};
float3 camEye = {0, 0, 0};

const float kFovY_deg = 45.0f;
const float kNear = 0.1f;
const float kFar = 2000.0f;

bool touchOnly = false;
std::map<SDL_FingerID, float2> activeFingers;
float2 prevGestureCenter;
bool hasPrevGestureCenter = false;
float dragRayDistance = 0.0f;
SDL_FingerID dragFingerId = 0;
bool touchHoldCandidate = false;
SDL_FingerID touchHoldFingerId = 0;
Uint32 touchHoldStartTicks = 0;
float2 touchHoldStartPos = {0, 0};

enum DragMode
{
    DRAG_MODE_NONE,
    DRAG_MODE_MOUSE,
    DRAG_MODE_TOUCH
};

DragMode dragMode = DRAG_MODE_NONE;

void makePlaneFromPointNormal(const float3 &p, const float3 &n, GLfloat plane[4]);
void makeShadowMatrix(GLfloat out[16], const GLfloat light[4], const GLfloat plane[4]);
void drawProjectedShadows();

bool findShadowPlane(float3 &planePoint, float3 &planeNormal)
{
    const float3 up = {0, 0, 1};
    float bestScore = 0.0f;
    bool found = false;

    for (Body *bodyIt = solver->bodies; bodyIt != 0; bodyIt = bodyIt->next)
    {
        if (bodyIt->kind != BODY_RIGID || bodyIt->mass > 0.0f)
            continue;
        Rigid *body = (Rigid *)bodyIt;

        float3 half = body->size * 0.5f;
        float3 axes[3] = {
            rotate(body->positionAng, float3{1, 0, 0}),
            rotate(body->positionAng, float3{0, 1, 0}),
            rotate(body->positionAng, float3{0, 0, 1})};

        for (int axis = 0; axis < 3; ++axis)
        {
            int i1 = (axis + 1) % 3;
            int i2 = (axis + 2) % 3;
            float area = 4.0f * half[i1] * half[i2];
            if (area <= 0.0f)
                continue;

            for (int s = 0; s < 2; ++s)
            {
                float sign = s == 0 ? -1.0f : 1.0f;
                float3 n = axes[axis] * sign;
                float upness = dot(n, up);
                if (upness <= 0.15f)
                    continue;

                float score = area * upness;
                if (!found || score > bestScore)
                {
                    found = true;
                    bestScore = score;
                    planeNormal = n;
                    planePoint = body->positionLin + n * half[axis];
                }
            }
        }
    }

    return found;
}

float3 bodyVertexWorld(const Rigid *body, const GLfloat v[3])
{
    float3 local = {v[0] * body->size.x, v[1] * body->size.y, v[2] * body->size.z};
    return transform(body->positionLin, body->positionAng, local);
}

float3 applyProjectiveMatrix(const GLfloat m[16], const float3 &p)
{
    float x = m[0] * p.x + m[4] * p.y + m[8] * p.z + m[12];
    float y = m[1] * p.x + m[5] * p.y + m[9] * p.z + m[13];
    float z = m[2] * p.x + m[6] * p.y + m[10] * p.z + m[14];
    float w = m[3] * p.x + m[7] * p.y + m[11] * p.z + m[15];
    if (fabsf(w) > 1.0e-6f)
    {
        x /= w;
        y /= w;
        z /= w;
    }
    return {x, y, z};
}

void drawBodySolid(const Rigid *body)
{
    static const GLfloat V[8][3] = {
        {-0.5f, -0.5f, -0.5f}, {+0.5f, -0.5f, -0.5f}, {+0.5f, +0.5f, -0.5f}, {-0.5f, +0.5f, -0.5f}, {-0.5f, -0.5f, +0.5f}, {+0.5f, -0.5f, +0.5f}, {+0.5f, +0.5f, +0.5f}, {-0.5f, +0.5f, +0.5f}};

    static const unsigned T[12][3] = {
        {0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6}, {1, 5, 6}, {1, 6, 2}, {4, 0, 3}, {4, 3, 7}, {3, 2, 6}, {3, 6, 7}, {4, 5, 1}, {4, 1, 0}};

    glBegin(GL_TRIANGLES);
    for (int i = 0; i < 12; ++i)
    {
        float3 a = bodyVertexWorld(body, V[T[i][0]]);
        float3 b = bodyVertexWorld(body, V[T[i][1]]);
        float3 c = bodyVertexWorld(body, V[T[i][2]]);
        glVertex3f(a.x, a.y, a.z);
        glVertex3f(b.x, b.y, b.z);
        glVertex3f(c.x, c.y, c.z);
    }
    glEnd();
}

void drawBodySolidProjected(const Rigid *body, const GLfloat shadowMat[16])
{
    static const GLfloat V[8][3] = {
        {-0.5f, -0.5f, -0.5f}, {+0.5f, -0.5f, -0.5f}, {+0.5f, +0.5f, -0.5f}, {-0.5f, +0.5f, -0.5f}, {-0.5f, -0.5f, +0.5f}, {+0.5f, -0.5f, +0.5f}, {+0.5f, +0.5f, +0.5f}, {-0.5f, +0.5f, +0.5f}};

    static const unsigned T[12][3] = {
        {0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6}, {1, 5, 6}, {1, 6, 2}, {4, 0, 3}, {4, 3, 7}, {3, 2, 6}, {3, 6, 7}, {4, 5, 1}, {4, 1, 0}};

    glBegin(GL_TRIANGLES);
    for (int i = 0; i < 12; ++i)
    {
        float3 a = applyProjectiveMatrix(shadowMat, bodyVertexWorld(body, V[T[i][0]]));
        float3 b = applyProjectiveMatrix(shadowMat, bodyVertexWorld(body, V[T[i][1]]));
        float3 c = applyProjectiveMatrix(shadowMat, bodyVertexWorld(body, V[T[i][2]]));
        glVertex3f(a.x, a.y, a.z);
        glVertex3f(b.x, b.y, b.z);
        glVertex3f(c.x, c.y, c.z);
    }
    glEnd();
}

void drawBody(const Rigid *body)
{
    static const GLfloat V[8][3] = {
        {-0.5f, -0.5f, -0.5f},
        {+0.5f, -0.5f, -0.5f},
        {+0.5f, +0.5f, -0.5f},
        {-0.5f, +0.5f, -0.5f},
        {-0.5f, -0.5f, +0.5f},
        {+0.5f, -0.5f, +0.5f},
        {+0.5f, +0.5f, +0.5f},
        {-0.5f, +0.5f, +0.5f}};

    static const unsigned T[12][3] = {
        {0, 1, 2}, {0, 2, 3}, // -Z
        {4, 6, 5},
        {4, 7, 6}, // +Z
        {1, 5, 6},
        {1, 6, 2}, // +X
        {4, 0, 3},
        {4, 3, 7}, // -X
        {3, 2, 6},
        {3, 6, 7}, // +Y
        {4, 5, 1},
        {4, 1, 0} // -Y
    };

    static const unsigned E[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};

    glColor4f(0.80f, 0.84f, 0.90f, 1.0f);

    // Push filled faces slightly behind wireframe edges to avoid z-fighting.
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0f, 1.0f);

    glBegin(GL_TRIANGLES);
    for (int i = 0; i < 12; ++i)
    {
        float3 a = bodyVertexWorld(body, V[T[i][0]]);
        float3 b = bodyVertexWorld(body, V[T[i][1]]);
        float3 c = bodyVertexWorld(body, V[T[i][2]]);
        glVertex3f(a.x, a.y, a.z);
        glVertex3f(b.x, b.y, b.z);
        glVertex3f(c.x, c.y, c.z);
    }
    glEnd();

    glDisable(GL_POLYGON_OFFSET_FILL);

    glColor4f(0.10f, 0.12f, 0.14f, 1.0f);
    glBegin(GL_LINES);
    for (int i = 0; i < 12; ++i)
    {
        float3 a = bodyVertexWorld(body, V[E[i][0]]);
        float3 b = bodyVertexWorld(body, V[E[i][1]]);
        glVertex3f(a.x, a.y, a.z);
        glVertex3f(b.x, b.y, b.z);
    }
    glEnd();
}

void drawJoint(const Joint *joint)
{
    const Rigid *bodyA = (const Rigid *)joint->bodies[0];
    const Rigid *bodyB = (const Rigid *)joint->bodies[1];
    float3 v0 = bodyA ? transform(bodyA->positionLin, bodyA->positionAng, joint->rA) : joint->rA;
    float3 v1 = transform(bodyB->positionLin, bodyB->positionAng, joint->rB);

    glColor3f(0.75f, 0.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(v0.x, v0.y, v0.z);
    glVertex3f(v1.x, v1.y, v1.z);
    glEnd();
}

void drawSpring(const Spring *spring)
{
    const Rigid *bodyA = (const Rigid *)spring->bodies[0];
    const Rigid *bodyB = (const Rigid *)spring->bodies[1];
    float3 v0 = transform(bodyA->positionLin, bodyA->positionAng, spring->rA);
    float3 v1 = transform(bodyB->positionLin, bodyB->positionAng, spring->rB);

    glColor3f(0.75f, 0.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(v0.x, v0.y, v0.z);
    glVertex3f(v1.x, v1.y, v1.z);
    glEnd();
}

void drawManifold(const Manifold *manifold)
{
    if (!SHOW_CONTACTS)
        return;

    const Rigid *bodyA = (const Rigid *)manifold->bodies[0];
    const Rigid *bodyB = (const Rigid *)manifold->bodies[1];
    glColor3f(0.75f, 0.0f, 0.0f);
    glBegin(GL_POINTS);
    for (int i = 0; i < manifold->numContacts; ++i)
    {
        float3 v0 = transform(bodyA->positionLin, bodyA->positionAng, manifold->contacts[i].rA);
        float3 v1 = transform(bodyB->positionLin, bodyB->positionAng, manifold->contacts[i].rB);
        glVertex3f(v0.x, v0.y, v0.z);
        glVertex3f(v1.x, v1.y, v1.z);
    }
    glEnd();
}

void drawSolver(const Solver *state)
{
    // Draw static receivers first so shadows depth-test against them.
    for (const Body *b = state->bodies; b != 0; b = b->next)
        if (b->kind == BODY_RIGID && b->mass <= 0.0f)
            drawBody((const Rigid *)b);

    drawProjectedShadows();

    // Draw dynamic bodies after shadows so they appear cleanly on top.
    for (const Body *b = state->bodies; b != 0; b = b->next)
        if (b->kind == BODY_RIGID && b->mass > 0.0f)
            drawBody((const Rigid *)b);

    for (const Force *force = state->forces; force != 0; force = force->next)
    {
        if (const Joint *joint = dynamic_cast<const Joint *>(force))
            drawJoint(joint);
        else if (const Spring *spring = dynamic_cast<const Spring *>(force))
            drawSpring(spring);
        else if (const Manifold *manifold = dynamic_cast<const Manifold *>(force))
            drawManifold(manifold);
    }
}

void drawProjectedShadows()
{
    GLint stencilBits = 0;
    glGetIntegerv(GL_STENCIL_BITS, &stencilBits);
    bool useStencil = stencilBits > 0;

    float3 planePoint;
    float3 planeNormal;
    if (!findShadowPlane(planePoint, planeNormal))
        return;

    GLfloat plane[4];
    makePlaneFromPointNormal(planePoint, planeNormal, plane);

    // Directional light (w=0). Normalized to keep stable projection scale.
    float3 l = normalize(float3{0.45f, 0.95f, 1.0f});
    GLfloat light[4] = {l.x, l.y, l.z, 0.0f};

    GLfloat shadowMat[16];
    makeShadowMatrix(shadowMat, light, plane);

    GLboolean lightingEnabled = glIsEnabled(GL_LIGHTING);
    GLboolean blendEnabled = glIsEnabled(GL_BLEND);
    GLboolean cullEnabled = glIsEnabled(GL_CULL_FACE);
    GLboolean polyOffsetEnabled = glIsEnabled(GL_POLYGON_OFFSET_FILL);
    GLboolean stencilEnabled = glIsEnabled(GL_STENCIL_TEST);
    GLboolean depthWriteEnabled = GL_TRUE;
    glGetBooleanv(GL_DEPTH_WRITEMASK, &depthWriteEnabled);
    GLboolean colorMask[4] = {GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE};
    glGetBooleanv(GL_COLOR_WRITEMASK, colorMask);

    auto drawProjectedCasters = [&]()
    {
        for (Body *b = solver->bodies; b != 0; b = b->next)
        {
            if (b->kind != BODY_RIGID || b->mass <= 0.0f)
                continue;
            drawBodySolidProjected((Rigid *)b, shadowMat);
        }
    };

    glDisable(GL_LIGHTING);
    glDisable(GL_CULL_FACE);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(-1.0f, -1.0f);
    glDepthMask(GL_FALSE);

    glDisable(GL_BLEND);
    if (useStencil)
    {
        glEnable(GL_STENCIL_TEST);
        glClear(GL_STENCIL_BUFFER_BIT);

        // Pass 1: mark all shadowed pixels in stencil.
        glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
        glStencilMask(0xFF);
        glStencilFunc(GL_ALWAYS, 1, 0xFF);
        glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
        drawProjectedCasters();

        // Pass 2: draw shadow once per pixel (no overlap darkening).
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        glColor3f(0.72f, 0.72f, 0.72f);
        glStencilMask(0xFF);
        glStencilFunc(GL_EQUAL, 1, 0xFF);
        glStencilOp(GL_KEEP, GL_KEEP, GL_ZERO);
        drawProjectedCasters();
    }
    else
    {
        // No stencil available: draw a flat shadow color directly.
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        glColor3f(0.72f, 0.72f, 0.72f);
        drawProjectedCasters();
    }

    glDepthMask(depthWriteEnabled);
    glColorMask(colorMask[0], colorMask[1], colorMask[2], colorMask[3]);
    if (useStencil)
        glStencilMask(0xFF);
    if (polyOffsetEnabled)
        glEnable(GL_POLYGON_OFFSET_FILL);
    else
        glDisable(GL_POLYGON_OFFSET_FILL);
    if (cullEnabled)
        glEnable(GL_CULL_FACE);
    else
        glDisable(GL_CULL_FACE);
    if (lightingEnabled)
        glEnable(GL_LIGHTING);
    else
        glDisable(GL_LIGHTING);
    if (blendEnabled)
        glEnable(GL_BLEND);
    else
        glDisable(GL_BLEND);
    if (stencilEnabled && useStencil)
        glEnable(GL_STENCIL_TEST);
    else
        glDisable(GL_STENCIL_TEST);
}

float3 orbitEye()
{
    float ce = cosf(camElevation), se = sinf(camElevation);
    float ca = cosf(camAzimuth), sa = sinf(camAzimuth);
    // Spherical with +Z up
    float3 off = {camDistance * ce * ca, camDistance * ce * sa, camDistance * se};
    return camTarget + off;
}

void shootBox()
{
    float3 forward = normalize(camTarget - camEye);
    float spawnOffset = 2.0f + 0.5f * length(boxSize);
    float3 spawnPos = camEye + forward * spawnOffset;
    float3 velocity = forward * boxVelocity;
    new Rigid(solver, boxSize, boxDensity, boxFriction, spawnPos, velocity);
}

void releaseDrag()
{
    if (drag)
    {
        delete drag;
        drag = 0;
    }

    dragMode = DRAG_MODE_NONE;
    dragFingerId = 0;
}

bool screenToWorldRay(float2 screenPos, float3 &origin, float3 &dir)
{
    int w, h;
    SDL_GetWindowSize(Window, &w, &h);
    if (w <= 0 || h <= 0)
        return false;

    float aspect = (float)w / (float)h;
    float ndcX = screenPos.x / (float)w * 2.0f - 1.0f;
    float ndcY = 1.0f - screenPos.y / (float)h * 2.0f;

    float3 forward = normalize(camTarget - camEye);
    float3 upHint = {0, 0, 1};
    float3 right = cross(forward, upHint);
    if (lengthSq(right) < 1.0e-8f)
        right = cross(forward, float3{0, 1, 0});
    right = normalize(right);
    float3 up = cross(right, forward);

    float tanHalfFovY = tanf(0.5f * rad(kFovY_deg));
    float px = ndcX * aspect * tanHalfFovY;
    float py = ndcY * tanHalfFovY;

    origin = camEye;
    dir = normalize(forward + right * px + up * py);
    return true;
}

bool beginDragAtScreen(float2 screenPos, DragMode mode, SDL_FingerID fingerId = 0)
{
    float3 rayOrigin;
    float3 rayDir;
    if (!screenToWorldRay(screenPos, rayOrigin, rayDir))
        return false;

    float3 localHit;
    Rigid *body = solver->pick(rayOrigin, rayDir, localHit);
    if (!body)
        return false;

    float3 worldHit = transform(body->positionLin, body->positionAng, localHit);
    dragRayDistance = max(dot(worldHit - rayOrigin, rayDir), 0.1f);

    const float dragStiffness = 5000.0f;
    drag = new Joint(solver, 0, body, worldHit, localHit, dragStiffness, 0.0f);
    dragMode = mode;
    dragFingerId = fingerId;
    return true;
}

void updateDragAtScreen(float2 screenPos)
{
    if (!drag)
        return;

    float3 rayOrigin;
    float3 rayDir;
    if (!screenToWorldRay(screenPos, rayOrigin, rayDir))
        return;

    drag->rA = rayOrigin + rayDir * dragRayDistance;
}

void setPerspective(float fovY_deg, float aspect, float zNear, float zFar)
{
    float top = zNear * tanf(0.5f * rad(fovY_deg));
    float right = top * aspect;
    glFrustum(-right, right, -top, top, zNear, zFar); // fixed-function perspective
}

void setLookAt(const float3 &eye, const float3 &center, const float3 &upW)
{
    float3 f = normalize(center - eye);
    float3 s = normalize(cross(f, upW)); // right
    float3 u = cross(s, f);              // corrected up

    GLfloat m[16] = {
        s.x, u.x, -f.x, 0,
        s.y, u.y, -f.y, 0,
        s.z, u.z, -f.z, 0,
        0, 0, 0, 1};
    glMultMatrixf(m);
    glTranslatef(-eye.x, -eye.y, -eye.z);
}

void makePlaneFromPointNormal(const float3 &p, const float3 &n, GLfloat plane[4])
{
    // Plane in form Ax + By + Cz + D = 0
    float3 nn = normalize(n);
    plane[0] = nn.x;
    plane[1] = nn.y;
    plane[2] = nn.z;
    plane[3] = -(nn.x * p.x + nn.y * p.y + nn.z * p.z);
}

void makeShadowMatrix(GLfloat out[16], const GLfloat light[4], const GLfloat plane[4])
{
    // M = (plane dot light) * I - light * plane^T  (column-major for OpenGL)
    const float dot = plane[0] * light[0] + plane[1] * light[1] + plane[2] * light[2] + plane[3] * light[3];
#define M(r, c) out[(c) * 4 + (r)]
    M(0, 0) = dot - light[0] * plane[0];
    M(0, 1) = -light[0] * plane[1];
    M(0, 2) = -light[0] * plane[2];
    M(0, 3) = -light[0] * plane[3];
    M(1, 0) = -light[1] * plane[0];
    M(1, 1) = dot - light[1] * plane[1];
    M(1, 2) = -light[1] * plane[2];
    M(1, 3) = -light[1] * plane[3];
    M(2, 0) = -light[2] * plane[0];
    M(2, 1) = -light[2] * plane[1];
    M(2, 2) = dot - light[2] * plane[2];
    M(2, 3) = -light[2] * plane[3];
    M(3, 0) = -light[3] * plane[0];
    M(3, 1) = -light[3] * plane[1];
    M(3, 2) = -light[3] * plane[2];
    M(3, 3) = dot - light[3] * plane[3];
#undef M
}

void ui()
{
    // Draw the ImGui UI
    ImGui::Begin("Controls");
    if (touchOnly)
    {
        ImGui::Text("Orbit Cam: Two-Finger Drag");
        ImGui::Text("Zoom Cam: Pinch");
        ImGui::Text("Shoot Box: Double Tap");
        ImGui::Text("Drag Box: Tap and Hold");
    }
    else
    {
        ImGui::Text("Orbit Cam: Right Mouse (W/A/S/D)");
        ImGui::Text("Zoom Cam: Mouse Wheel (Q/E)");
        ImGui::Text("Shoot Box: Middle Mouse (Space)");
        ImGui::Text("Drag Box: Left Mouse");
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    int scene = currScene;
    if (ImGui::BeginCombo("Scene", sceneNames[scene]))
    {
        for (int i = 0; i < sceneCount; i++)
        {
            bool selected = scene == i;
            if (ImGui::Selectable(sceneNames[i], selected) && i != currScene)
            {
                releaseDrag();
                currScene = i;
                scenes[currScene](solver);
            }
            if (selected)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    if (ImGui::Button(" Reset "))
    {
        releaseDrag();
        scenes[currScene](solver);
    }
    ImGui::SameLine();
    if (ImGui::Button("Default"))
        solver->defaultParams();

    ImGui::Checkbox("Pause", &paused);
    if (paused)
    {
        ImGui::SameLine();
        if (ImGui::Button("Step"))
            solver->step();
    }

    ImGui::Spacing();
    ImGui::SliderFloat("Box Friction", &boxFriction, 0.0f, 2.0f);
    ImGui::SliderFloat3("Box Size", &boxSize.x, 0.1f, 5.0f);
    ImGui::SliderFloat("Box Velocity", &boxVelocity, 0.0f, 20.0f);

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::SliderFloat("Gravity", &solver->gravity, -20.0f, 20.0f);
    ImGui::SliderFloat("Dt", &solver->dt, 0.001f, 0.1f);
    ImGui::SliderInt("Iterations", &solver->iterations, 1, 50);

    ImGui::End();
}

void input()
{
    auto &io = ImGui::GetIO();

    // --- Orbit controls ---
    const float orbitSpeedMouse = 0.005f;
    const float orbitSpeedKeys = rad(120.0f);
    if (ImGui::IsMouseDown(ImGuiMouseButton_Right))
    {
        camAzimuth -= io.MouseDelta.x * orbitSpeedMouse;   // right drag -> yaw
        camElevation += io.MouseDelta.y * orbitSpeedMouse; // up drag   -> pitch
        camElevation = clamp(camElevation, rad(-89.0f), rad(89.0f));
    }

    if (!io.WantCaptureKeyboard)
    {
        float orbitDelta = orbitSpeedKeys * io.DeltaTime;
        if (ImGui::IsKeyDown(ImGuiKey_A))
            camAzimuth -= orbitDelta;
        if (ImGui::IsKeyDown(ImGuiKey_D))
            camAzimuth += orbitDelta;
        if (ImGui::IsKeyDown(ImGuiKey_W))
            camElevation += orbitDelta;
        if (ImGui::IsKeyDown(ImGuiKey_S))
            camElevation -= orbitDelta;
        camElevation = clamp(camElevation, rad(-89.0f), rad(89.0f));

        if (ImGui::IsKeyDown(ImGuiKey_E))
            camDistance *= 1.025f;
        if (ImGui::IsKeyDown(ImGuiKey_Q))
            camDistance /= 1.025f;
    }

    // Mouse wheel zoom (scroll up = closer)
    if (io.MouseWheel != 0.0f)
    {
        camDistance /= powf(1.1f, io.MouseWheel);
    }
    camDistance = clamp(camDistance, 0.2f, 1000.0f);

    bool shootFromMouse = ImGui::IsMouseClicked(ImGuiMouseButton_Middle) && !io.WantCaptureMouse;
    bool shootFromKeyboard = ImGui::IsKeyPressed(ImGuiKey_Space, false) && !io.WantCaptureKeyboard;
    bool shootFromTouch = touchOnly && shootRequested;
    if (shootFromMouse || shootFromKeyboard || shootFromTouch)
        shootBox();
    shootRequested = false;

    if (!touchOnly)
    {
        bool mouseDragDown = ImGui::IsMouseDown(ImGuiMouseButton_Left) && !io.WantCaptureMouse;
        float2 mousePos = {io.MousePos.x, io.MousePos.y};
        if (mouseDragDown)
        {
            if (dragMode == DRAG_MODE_MOUSE && drag)
                updateDragAtScreen(mousePos);
            else if (!drag)
                beginDragAtScreen(mousePos, DRAG_MODE_MOUSE);
        }
        else if (dragMode == DRAG_MODE_MOUSE)
        {
            releaseDrag();
        }
    }

    if (touchOnly)
    {
        const Uint32 kHoldMs = 180;
        const float kHoldMovePx = 20.0f;

        if (activeFingers.size() == 1 && !io.WantCaptureMouse)
        {
            auto it = activeFingers.begin();
            SDL_FingerID fingerId = it->first;
            float2 pos = it->second;

            if (dragMode == DRAG_MODE_TOUCH && drag && fingerId == dragFingerId)
            {
                updateDragAtScreen(pos);
            }
            else if (!drag && touchHoldCandidate && fingerId == touchHoldFingerId)
            {
                float2 move = pos - touchHoldStartPos;
                if (lengthSq(move) > kHoldMovePx * kHoldMovePx)
                {
                    touchHoldCandidate = false;
                }
                else if (SDL_GetTicks() - touchHoldStartTicks >= kHoldMs)
                {
                    if (beginDragAtScreen(pos, DRAG_MODE_TOUCH, fingerId))
                        touchHoldCandidate = false;
                }
            }
        }
        else
        {
            touchHoldCandidate = false;
            if (dragMode == DRAG_MODE_TOUCH)
                releaseDrag();
        }
    }
}

void mainLoop()
{
    // Event loop
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
        ImGui_ImplSDL2_ProcessEvent(&event);

        if (event.type == SDL_KEYDOWN)
        {
            if (event.key.keysym.sym == SDLK_RETURN && (event.key.keysym.mod & KMOD_ALT))
            {
                FullScreen = !FullScreen;
                Uint32 fullscreenFlag = FullScreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0;
                SDL_SetWindowFullscreen(Window, fullscreenFlag);
            }

            if (event.key.keysym.sym == SDLK_ESCAPE)
            {
#ifndef __EMSCRIPTEN__
                Running = 0;
#endif
            }
        }
        else if (event.type == SDL_QUIT)
        {
#ifndef __EMSCRIPTEN__
            Running = 0;
#endif
        }
        else if (event.type == SDL_FINGERDOWN)
        {
            int w, h;
            SDL_GetWindowSize(Window, &w, &h);
            SDL_FingerID id = event.tfinger.fingerId;
            float2 pos = {event.tfinger.x * w, event.tfinger.y * h};

            if (touchOnly && activeFingers.empty())
            {
                const Uint32 kDoubleTapMs = 300;
                const float kDoubleTapDistance = 40.0f;
                Uint32 now = SDL_GetTicks();
                float2 deltaTap = pos - lastTapPos;
                bool closeInTime = lastTapTicks > 0 && (now - lastTapTicks) <= kDoubleTapMs;
                bool closeInSpace = lengthSq(deltaTap) <= kDoubleTapDistance * kDoubleTapDistance;
                if (closeInTime && closeInSpace)
                {
                    shootRequested = true;
                    lastTapTicks = 0;
                }
                else
                {
                    lastTapTicks = now;
                    lastTapPos = pos;
                }
            }

            activeFingers[id] = pos;
            if (touchOnly)
            {
                if (activeFingers.size() == 1)
                {
                    touchHoldCandidate = true;
                    touchHoldFingerId = id;
                    touchHoldStartTicks = SDL_GetTicks();
                    touchHoldStartPos = pos;
                }
                else
                {
                    touchHoldCandidate = false;
                    if (dragMode == DRAG_MODE_TOUCH)
                        releaseDrag();
                }
            }
            if (activeFingers.size() != 2)
            {
                hasPrevGestureCenter = false;
            }
        }
        else if (event.type == SDL_FINGERMOTION)
        {
            int w, h;
            SDL_GetWindowSize(Window, &w, &h);
            SDL_FingerID id = event.tfinger.fingerId;
            auto it = activeFingers.find(id);
            if (it != activeFingers.end())
                it->second = {event.tfinger.x * w, event.tfinger.y * h};
        }
        else if (event.type == SDL_FINGERUP)
        {
            if (touchOnly)
            {
                if (touchHoldCandidate && event.tfinger.fingerId == touchHoldFingerId)
                    touchHoldCandidate = false;

                if (dragMode == DRAG_MODE_TOUCH && event.tfinger.fingerId == dragFingerId)
                    releaseDrag();
            }

            activeFingers.erase(event.tfinger.fingerId);
            hasPrevGestureCenter = false;
        }
        else if (event.type == SDL_MULTIGESTURE)
        {
            if (event.mgesture.numFingers == 2)
            {
                int w, h;
                SDL_GetWindowSize(Window, &w, &h);
                float2 center = {event.mgesture.x * w, event.mgesture.y * h};

                // Two-finger drag -> orbit
                if (hasPrevGestureCenter)
                {
                    float2 d = center - prevGestureCenter;
                    camAzimuth -= d.x * 0.005f;
                    camElevation += d.y * 0.005f;
                    camElevation = clamp(camElevation, rad(-89.0f), rad(89.0f));
                }
                prevGestureCenter = center;
                hasPrevGestureCenter = true;

                // Pinch -> zoom
                float dDist = event.mgesture.dDist; // positive = fingers move apart
                if (dDist != 0.0f)
                {
                    float zoomFactor = 1.0f + dDist * 2.0f; // gentle
                    if (zoomFactor > 0.01f)
                        camDistance = clamp(camDistance / zoomFactor, 0.2f, 1000.0f);
                }
            }
        }
    }

    // Setup GL
    int w, h;
    SDL_GetWindowSize(Window, &w, &h);

    glEnable(GL_LINE_SMOOTH);
    glLineWidth(2.0f);
    glPointSize(3.0f);
    glViewport(0, 0, w, h);
    glClearColor(1, 1, 1, 1);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glShadeModel(GL_FLAT);
    glEnable(GL_NORMALIZE);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Camera matrices
    camEye = orbitEye();

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    setPerspective(kFovY_deg, (float)w / (float)h, kNear, kFar);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    setLookAt(camEye, camTarget, float3{0, 0, 1});

    // ImGUI setup
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    input();
    ui();

    // Step solver and draw it
    if (!paused)
        solver->step();
    drawSolver(solver);

    // ImGUI rendering
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    SDL_GL_SwapWindow(Window);
}

int main(int argc, char *argv[])
{
    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        printf("Failed to initialize SDL: %s\n", SDL_GetError());
        return -1;
    }

#ifdef __EMSCRIPTEN__
    touchOnly = (bool)emscripten_run_script_int("window.matchMedia('(pointer:coarse)').matches ? 1 : 0");
#endif

    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1); // Enable multisampling
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

#ifdef __EMSCRIPTEN__
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3); // OpenGL ES 3.0 (WebGL2)
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0); // No forward-compatible flag
#endif

    // Create the SDL window
    Window = SDL_CreateWindow("AVBD 3D", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WinWidth, WinHeight, WindowFlags);
    if (!Window)
    {
        printf("Failed to create window: %s\n", SDL_GetError());
        return -1;
    }

    // Create the OpenGL context
    Context = SDL_GL_CreateContext(Window);
    if (!Context)
    {
        printf("Failed to create OpenGL context: %s\n", SDL_GetError());
        SDL_DestroyWindow(Window);
        SDL_Quit();
        return -1;
    }

    SDL_GL_MakeCurrent(Window, Context);
    SDL_GL_SetSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;

// Scale UI higher for mobile devices
#ifdef __EMSCRIPTEN__
    const float uiScale = touchOnly ? 2.0f : 1.0f;
#else
    const float uiScale = 1.0f;
#endif

    ImFontConfig font_config;
    font_config.SizePixels = 13.0f * uiScale;
    io.Fonts->AddFontDefault(&font_config);

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Scale all style elements
    ImGuiStyle &style = ImGui::GetStyle();
    style.ScaleAllSizes(uiScale);

    // Setup Platform/Renderer bindings
    ImGui_ImplSDL2_InitForOpenGL(Window, Context);
#ifdef __EMSCRIPTEN__
    ImGui_ImplOpenGL3_Init("#version 300 es"); // WebAssembly
#else
    ImGui_ImplOpenGL3_Init("#version 150"); // Desktop OpenGL
#endif

    // Load scene
    scenes[currScene](solver);

#ifdef __EMSCRIPTEN__
    // Use Emscripten's main loop for the web
    emscripten_set_main_loop(mainLoop, 0, 1);
#else
    // For native builds, use a while loop
    while (Running)
    {
        mainLoop();
    }
#endif

    // Cleanup
    SDL_GL_DeleteContext(Context);
    SDL_DestroyWindow(Window);
    SDL_Quit();

    return 0;
}

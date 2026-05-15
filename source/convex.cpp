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

// Convex hull collision shape support for AVBD.
//
// A box body keeps the fast tuned OBB path (collide.cpp). When at least one body
// carries a ConvexHull, narrow phase runs here: an exact separating-axis test over
// all face normals of both hulls plus all edge-edge cross products, followed by
// reference/incident face clipping (or edge closest-points) to build the contact
// manifold. SAT is used rather than GJK/EPA because it is exact for convex
// polyhedra, robust, and matches the style of the existing box collision code.

#include "solver.h"
#include <cfloat>
#include <cmath>
#include <vector>

namespace
{
constexpr int MAX_CONTACTS = 8;
constexpr float AXIS_EPSILON = 1.0e-6f;
constexpr float PLANE_EPSILON = 1.0e-4f;
constexpr float CONTACT_MERGE_DIST_SQ = 1.0e-6f;
constexpr float PARALLEL_EPSILON = 1.0e-5f;

// Canonical second-moment (covariance) matrix of the unit tetrahedron with
// vertices (0,0,0),(1,0,0),(0,1,0),(0,0,1): integral of x*x^T over its volume.
inline float3x3 canonicalTetCovariance()
{
    return float3x3{
        2.0f / 120.0f, 1.0f / 120.0f, 1.0f / 120.0f,
        1.0f / 120.0f, 2.0f / 120.0f, 1.0f / 120.0f,
        1.0f / 120.0f, 1.0f / 120.0f, 2.0f / 120.0f};
}

inline float determinant(const float3x3 &m)
{
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
           m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
           m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

inline float3 identityRow(int i)
{
    return float3{i == 0 ? 1.0f : 0.0f, i == 1 ? 1.0f : 0.0f, i == 2 ? 1.0f : 0.0f};
}

// Clips a polygon against a half-space, keeping vertices with dot(n,p) <= offset.
int clipPolygon(const float3 *in, int inCount, float3 n, float offset, float3 *out, int maxOut)
{
    if (inCount <= 0)
        return 0;

    int outCount = 0;
    float3 a = in[inCount - 1];
    float da = dot(n, a) - offset;

    for (int i = 0; i < inCount; ++i)
    {
        float3 b = in[i];
        float db = dot(n, b) - offset;

        bool aIn = da <= PLANE_EPSILON;
        bool bIn = db <= PLANE_EPSILON;

        if (aIn != bIn)
        {
            float denom = da - db;
            float t = fabsf(denom) > AXIS_EPSILON ? clamp(da / denom, 0.0f, 1.0f) : 0.0f;
            if (outCount < maxOut)
                out[outCount++] = a + (b - a) * t;
        }
        if (bIn && outCount < maxOut)
            out[outCount++] = b;

        a = b;
        da = db;
    }
    return outCount;
}

void closestPointsOnSegments(float3 p0, float3 p1, float3 q0, float3 q1, float3 &c0, float3 &c1)
{
    float3 d1 = p1 - p0, d2 = q1 - q0, r = p0 - q0;
    float a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
    float s = 0.0f, t = 0.0f;

    if (a <= AXIS_EPSILON && e <= AXIS_EPSILON)
    {
        c0 = p0;
        c1 = q0;
        return;
    }
    if (a <= AXIS_EPSILON)
    {
        t = clamp(f / e, 0.0f, 1.0f);
    }
    else
    {
        float c = dot(d1, r);
        if (e <= AXIS_EPSILON)
        {
            s = clamp(-c / a, 0.0f, 1.0f);
        }
        else
        {
            float b = dot(d1, d2);
            float denom = a * e - b * b;
            if (fabsf(denom) > AXIS_EPSILON)
                s = clamp((b * f - c * e) / denom, 0.0f, 1.0f);
            t = (b * s + f) / e;
            if (t < 0.0f)
            {
                t = 0.0f;
                s = clamp(-c / a, 0.0f, 1.0f);
            }
            else if (t > 1.0f)
            {
                t = 1.0f;
                s = clamp((b - c) / a, 0.0f, 1.0f);
            }
        }
    }
    c0 = p0 + d1 * s;
    c1 = q0 + d2 * t;
}

// World-space view of a hull for one collision query.
struct HullWorld
{
    const ConvexHull *hull;
    std::vector<float3> verts;   // world-space vertices
    std::vector<float3> normals; // world-space face normals
    float3 center;               // world centre of mass

    void build(const Rigid *body, const ConvexHull *h)
    {
        hull = h;
        center = body->positionLin;
        verts.resize(h->numVerts);
        for (int i = 0; i < h->numVerts; ++i)
            verts[i] = transform(body->positionLin, body->positionAng, h->verts[i]);
        normals.resize(h->numFaces);
        for (int i = 0; i < h->numFaces; ++i)
            normals[i] = rotate(body->positionAng, h->faceNormals[i]);
    }

    float supportMax(float3 n) const
    {
        float m = -FLT_MAX;
        for (const float3 &v : verts)
            m = max(m, dot(n, v));
        return m;
    }
    float supportMin(float3 n) const
    {
        float m = FLT_MAX;
        for (const float3 &v : verts)
        {
            float d = dot(n, v);
            if (d < m)
                m = d;
        }
        return m;
    }
};

// Largest separation of `other` from a face plane of `ref`. Positive => disjoint.
float queryFaceSeparation(const HullWorld &ref, const HullWorld &other, int &bestFace)
{
    float best = -FLT_MAX;
    bestFace = 0;
    for (int f = 0; f < ref.hull->numFaces; ++f)
    {
        float3 n = ref.normals[f];
        float planeOffset = dot(n, ref.verts[ref.hull->faceVerts[ref.hull->faceStart[f]]]);
        float sep = other.supportMin(n) - planeOffset;
        if (sep > best)
        {
            best = sep;
            bestFace = f;
        }
    }
    return best;
}

// Largest edge-edge separation. `axisOut` points from A to B.
float queryEdgeSeparation(const HullWorld &a, const HullWorld &b, int &edgeA, int &edgeB, float3 &axisOut)
{
    float best = -FLT_MAX;
    edgeA = edgeB = 0;
    axisOut = float3{0, 0, 1};
    float3 dirBA = b.center - a.center;

    for (int i = 0; i < a.hull->numEdges; ++i)
    {
        float3 a0 = a.verts[a.hull->edges[i * 2 + 0]];
        float3 a1 = a.verts[a.hull->edges[i * 2 + 1]];
        float3 dA = a1 - a0;
        for (int j = 0; j < b.hull->numEdges; ++j)
        {
            float3 b0 = b.verts[b.hull->edges[j * 2 + 0]];
            float3 b1 = b.verts[b.hull->edges[j * 2 + 1]];
            float3 dB = b1 - b0;

            float3 axis = cross(dA, dB);
            float len = length(axis);
            if (len < PARALLEL_EPSILON)
                continue;
            axis = axis / len;
            if (dot(axis, dirBA) < 0.0f)
                axis = -axis;

            float sep = b.supportMin(axis) - a.supportMax(axis);
            if (sep > best)
            {
                best = sep;
                edgeA = i;
                edgeB = j;
                axisOut = axis;
            }
        }
    }
    return best;
}

bool addContact(Rigid *bodyA, Rigid *bodyB, Manifold::Contact *contacts, int &count,
                float3 *midpoints, float3 xA, float3 xB, int featureKey)
{
    if (count >= MAX_CONTACTS)
        return false;
    float3 mid = (xA + xB) * 0.5f;
    for (int i = 0; i < count; ++i)
        if (lengthSq(mid - midpoints[i]) < CONTACT_MERGE_DIST_SQ)
            return false;

    Manifold::Contact &c = contacts[count];
    c.feature.key = featureKey;
    c.rA = rotate(conjugate(bodyA->positionAng), xA - bodyA->positionLin);
    c.rB = rotate(conjugate(bodyB->positionAng), xB - bodyB->positionLin);
    midpoints[count] = mid;
    ++count;
    return true;
}

// Builds a contact manifold for a face reference axis by clipping the incident
// face of the other hull against the reference face side planes.
int buildFaceManifold(Rigid *bodyA, Rigid *bodyB, const HullWorld &a, const HullWorld &b,
                      bool referenceIsA, int referenceFace, Manifold::Contact *contacts)
{
    const HullWorld &ref = referenceIsA ? a : b;
    const HullWorld &inc = referenceIsA ? b : a;
    float3 refNormal = ref.normals[referenceFace];

    // Reference face polygon (world space).
    const ConvexHull *rh = ref.hull;
    int rStart = rh->faceStart[referenceFace];
    int rCount = rh->faceStart[referenceFace + 1] - rStart;
    std::vector<float3> refPoly(rCount);
    for (int i = 0; i < rCount; ++i)
        refPoly[i] = ref.verts[rh->faceVerts[rStart + i]];

    // Incident face: the face of the other hull most anti-parallel to refNormal.
    int incidentFace = 0;
    float bestDot = FLT_MAX;
    for (int f = 0; f < inc.hull->numFaces; ++f)
    {
        float d = dot(inc.normals[f], refNormal);
        if (d < bestDot)
        {
            bestDot = d;
            incidentFace = f;
        }
    }

    const ConvexHull *ih = inc.hull;
    int iStart = ih->faceStart[incidentFace];
    int iCount = ih->faceStart[incidentFace + 1] - iStart;

    std::vector<float3> bufA(64), bufB(64);
    int count = iCount;
    for (int i = 0; i < iCount; ++i)
        bufA[i] = inc.verts[ih->faceVerts[iStart + i]];

    // Clip against each side plane of the reference face.
    float3 *src = bufA.data();
    float3 *dst = bufB.data();
    for (int i = 0; i < rCount && count > 0; ++i)
    {
        float3 e = refPoly[(i + 1) % rCount] - refPoly[i];
        float3 sideN = cross(e, refNormal); // outward for CCW winding about refNormal
        float lenSq = lengthSq(sideN);
        if (lenSq < AXIS_EPSILON)
            continue;
        sideN = sideN / sqrtf(lenSq);
        float offset = dot(sideN, refPoly[i]);
        count = clipPolygon(src, count, sideN, offset, dst, 64);
        float3 *tmp = src;
        src = dst;
        dst = tmp;
    }

    int contactCount = 0;
    float3 midpoints[MAX_CONTACTS];
    float refPlaneOffset = dot(refNormal, refPoly[0]);
    int featurePrefix = ((referenceIsA ? 1 : 2) << 24) | ((referenceFace & 0xFF) << 16) | ((incidentFace & 0xFF) << 8);

    for (int i = 0; i < count; ++i)
    {
        float3 pInc = src[i];
        float dist = dot(pInc, refNormal) - refPlaneOffset;
        if (dist > PLANE_EPSILON)
            continue;
        float3 pRef = pInc - refNormal * dist;
        float3 xA = referenceIsA ? pRef : pInc;
        float3 xB = referenceIsA ? pInc : pRef;
        addContact(bodyA, bodyB, contacts, contactCount, midpoints, xA, xB, featurePrefix | (i & 0xFF));
    }

    // Fallback: use deepest support points if clipping produced nothing.
    if (contactCount == 0)
    {
        float3 nAB = referenceIsA ? refNormal : -refNormal;
        float3 xA = a.verts[0];
        float bestA = -FLT_MAX;
        for (const float3 &v : a.verts)
            if (dot(nAB, v) > bestA)
            {
                bestA = dot(nAB, v);
                xA = v;
            }
        float3 xB = b.verts[0];
        float bestB = -FLT_MAX;
        for (const float3 &v : b.verts)
            if (dot(-nAB, v) > bestB)
            {
                bestB = dot(-nAB, v);
                xB = v;
            }
        addContact(bodyA, bodyB, contacts, contactCount, midpoints, xA, xB, featurePrefix);
    }
    return contactCount;
}

int buildEdgeManifold(Rigid *bodyA, Rigid *bodyB, const HullWorld &a, const HullWorld &b,
                      int edgeA, int edgeB, Manifold::Contact *contacts)
{
    float3 a0 = a.verts[a.hull->edges[edgeA * 2 + 0]];
    float3 a1 = a.verts[a.hull->edges[edgeA * 2 + 1]];
    float3 b0 = b.verts[b.hull->edges[edgeB * 2 + 0]];
    float3 b1 = b.verts[b.hull->edges[edgeB * 2 + 1]];

    float3 xA, xB;
    closestPointsOnSegments(a0, a1, b0, b1, xA, xB);

    int contactCount = 0;
    float3 midpoints[MAX_CONTACTS];
    int featureKey = (3 << 24) | ((edgeA & 0xFFF) << 12) | (edgeB & 0xFFF);
    addContact(bodyA, bodyB, contacts, contactCount, midpoints, xA, xB, featureKey);
    return contactCount;
}

} // namespace

// ---------------------------------------------------------------------------
// ConvexHull construction
// ---------------------------------------------------------------------------

ConvexHull::~ConvexHull()
{
    delete[] verts;
    delete[] faceNormals;
    delete[] faceStart;
    delete[] faceVerts;
    delete[] edges;
}

ConvexHull *ConvexHull::create(const float3 *points, int numPoints,
                               const int *faceVertCounts, const int *faceIndices, int numFaces)
{
    if (numPoints < 4 || numFaces < 4)
        return nullptr;

    int totalIndices = 0;
    for (int f = 0; f < numFaces; ++f)
    {
        if (faceVertCounts[f] < 3)
            return nullptr;
        totalIndices += faceVertCounts[f];
    }

    ConvexHull *h = new ConvexHull();
    h->numVerts = numPoints;
    h->numFaces = numFaces;
    h->verts = new float3[numPoints];
    h->faceNormals = new float3[numFaces];
    h->faceStart = new int[numFaces + 1];
    h->faceVerts = new int[totalIndices];

    for (int i = 0; i < numPoints; ++i)
        h->verts[i] = points[i];

    // Hull centroid, used to orient face normals/winding outward.
    float3 centroid{0, 0, 0};
    for (int i = 0; i < numPoints; ++i)
        centroid += h->verts[i];
    centroid = centroid / (float)numPoints;

    int cursor = 0;
    for (int f = 0; f < numFaces; ++f)
    {
        h->faceStart[f] = cursor;
        int vc = faceVertCounts[f];
        for (int i = 0; i < vc; ++i)
            h->faceVerts[cursor + i] = faceIndices[cursor + i];

        // Face normal from the first non-degenerate vertex triple.
        float3 v0 = h->verts[h->faceVerts[cursor]];
        float3 n{0, 0, 0};
        for (int i = 1; i + 1 < vc; ++i)
        {
            float3 e1 = h->verts[h->faceVerts[cursor + i]] - v0;
            float3 e2 = h->verts[h->faceVerts[cursor + i + 1]] - v0;
            n = cross(e1, e2);
            if (lengthSq(n) > AXIS_EPSILON)
                break;
        }
        if (lengthSq(n) < AXIS_EPSILON)
            n = float3{0, 0, 1};
        n = normalize(n);

        // Orient outward; reverse winding to keep CCW about the outward normal.
        if (dot(n, v0 - centroid) < 0.0f)
        {
            n = -n;
            for (int i = 0; i < vc / 2; ++i)
            {
                int tmp = h->faceVerts[cursor + i];
                h->faceVerts[cursor + i] = h->faceVerts[cursor + vc - 1 - i];
                h->faceVerts[cursor + vc - 1 - i] = tmp;
            }
        }
        h->faceNormals[f] = n;
        cursor += vc;
    }
    h->faceStart[numFaces] = cursor;

    // Derive unique undirected edges from the face loops.
    std::vector<int> edgeList;
    for (int f = 0; f < numFaces; ++f)
    {
        int s = h->faceStart[f];
        int vc = h->faceStart[f + 1] - s;
        for (int i = 0; i < vc; ++i)
        {
            int va = h->faceVerts[s + i];
            int vb = h->faceVerts[s + (i + 1) % vc];
            int lo = min(va, vb), hi = max(va, vb);
            bool found = false;
            for (size_t e = 0; e < edgeList.size(); e += 2)
                if (edgeList[e] == lo && edgeList[e + 1] == hi)
                {
                    found = true;
                    break;
                }
            if (!found)
            {
                edgeList.push_back(lo);
                edgeList.push_back(hi);
            }
        }
    }
    h->numEdges = (int)edgeList.size() / 2;
    h->edges = new int[edgeList.size()];
    for (size_t i = 0; i < edgeList.size(); ++i)
        h->edges[i] = edgeList[i];

    // Bounds about the origin.
    h->radius = 0.0f;
    float3 lo{FLT_MAX, FLT_MAX, FLT_MAX}, hi{-FLT_MAX, -FLT_MAX, -FLT_MAX};
    for (int i = 0; i < numPoints; ++i)
    {
        h->radius = max(h->radius, length(h->verts[i]));
        lo = float3{min(lo.x, h->verts[i].x), min(lo.y, h->verts[i].y), min(lo.z, h->verts[i].z)};
        hi.x = max(hi.x, h->verts[i].x);
        hi.y = max(hi.y, h->verts[i].y);
        hi.z = max(hi.z, h->verts[i].z);
    }
    h->aabbHalf = float3{max(fabsf(lo.x), fabsf(hi.x)), max(fabsf(lo.y), fabsf(hi.y)), max(fabsf(lo.z), fabsf(hi.z))};
    return h;
}

ConvexHull *ConvexHull::createBox(float3 size)
{
    float3 h = size * 0.5f;
    float3 pts[8] = {
        {-h.x, -h.y, -h.z}, {h.x, -h.y, -h.z}, {h.x, h.y, -h.z}, {-h.x, h.y, -h.z},
        {-h.x, -h.y, h.z}, {h.x, -h.y, h.z}, {h.x, h.y, h.z}, {-h.x, h.y, h.z}};
    // Six quad faces, each wound CCW about its outward normal.
    int faceCounts[6] = {4, 4, 4, 4, 4, 4};
    int faceIdx[24] = {
        0, 3, 2, 1, // -Z
        4, 5, 6, 7, // +Z
        0, 1, 5, 4, // -Y
        2, 3, 7, 6, // +Y
        0, 4, 7, 3, // -X
        1, 2, 6, 5  // +X
    };
    return create(pts, 8, faceCounts, faceIdx, 6);
}

// ---------------------------------------------------------------------------
// Convex polyhedron mass properties (tetrahedron decomposition about the origin)
// ---------------------------------------------------------------------------

void computeHullMassProperties(const ConvexHull *h, float density,
                               float &massOut, float3 &comOut, float3 &momentOut)
{
    float3x3 covariance{0, 0, 0, 0, 0, 0, 0, 0, 0};
    float3 comIntegral{0, 0, 0};
    float volume = 0.0f;
    float3x3 C0 = canonicalTetCovariance();

    for (int f = 0; f < h->numFaces; ++f)
    {
        int s = h->faceStart[f];
        int vc = h->faceStart[f + 1] - s;
        float3 p0 = h->verts[h->faceVerts[s]];
        // Fan-triangulate the face; each triangle plus the origin forms a tet.
        for (int i = 1; i + 1 < vc; ++i)
        {
            float3 p1 = h->verts[h->faceVerts[s + i]];
            float3 p2 = h->verts[h->faceVerts[s + i + 1]];

            // A has the tet edge vectors as columns (one vertex at the origin).
            float3x3 A{
                p0.x, p1.x, p2.x,
                p0.y, p1.y, p2.y,
                p0.z, p1.z, p2.z};
            float det = determinant(A);

            volume += det / 6.0f;
            comIntegral += (p0 + p1 + p2) * (det / 24.0f);
            covariance += (A * C0 * transpose(A)) * det;
        }
    }

    if (volume < 0.0f)
    {
        volume = -volume;
        comIntegral = -comIntegral;
        covariance = -covariance;
    }
    if (volume < 1.0e-9f)
    {
        massOut = 0.0f;
        comOut = float3{0, 0, 0};
        momentOut = float3{0, 0, 0};
        return;
    }

    massOut = density * volume;
    comOut = comIntegral / volume;

    // Inertia about the origin: trace(C)*I - C.
    float traceC = covariance[0][0] + covariance[1][1] + covariance[2][2];
    float3x3 inertiaO = diagonal(traceC, traceC, traceC) - covariance;

    // Shift to the centre of mass (parallel axis theorem) and scale by density.
    float comDot = dot(comOut, comOut);
    float3x3 shift = diagonal(comDot, comDot, comDot) - outer(comOut, comOut);
    float3x3 inertiaCom = (inertiaO - shift * volume) * density;

    momentOut = float3{inertiaCom[0][0], inertiaCom[1][1], inertiaCom[2][2]};
}

// ---------------------------------------------------------------------------
// Convex narrow phase
// ---------------------------------------------------------------------------

int collideConvex(Rigid *bodyA, Rigid *bodyB, Manifold::Contact *contacts, float3x3 &basisOut)
{
    // Box bodies have no stored hull; build a temporary one for the convex path.
    ConvexHull *tmpA = bodyA->hull ? nullptr : ConvexHull::createBox(bodyA->size);
    ConvexHull *tmpB = bodyB->hull ? nullptr : ConvexHull::createBox(bodyB->size);
    const ConvexHull *hullA = bodyA->hull ? bodyA->hull : tmpA;
    const ConvexHull *hullB = bodyB->hull ? bodyB->hull : tmpB;

    int result = 0;
    if (hullA && hullB)
    {
        HullWorld a, b;
        a.build(bodyA, hullA);
        b.build(bodyB, hullB);

        int faceA = 0, faceB = 0, edgeA = 0, edgeB = 0;
        float sepFaceA = queryFaceSeparation(a, b, faceA);
        float sepFaceB = sepFaceA > 0.0f ? 1.0f : queryFaceSeparation(b, a, faceB);
        float3 edgeAxis{0, 0, 1};
        float sepEdge = (sepFaceA > 0.0f || sepFaceB > 0.0f)
                            ? 1.0f
                            : queryEdgeSeparation(a, b, edgeA, edgeB, edgeAxis);

        if (sepFaceA <= 0.0f && sepFaceB <= 0.0f && sepEdge <= 0.0f)
        {
            // Prefer a face axis unless an edge axis is clearly deeper (less overlap).
            const float faceBias = 0.005f;
            float bestFaceSep = max(sepFaceA, sepFaceB);

            if (sepEdge > bestFaceSep + faceBias)
            {
                basisOut = orthonormal(-edgeAxis);
                result = buildEdgeManifold(bodyA, bodyB, a, b, edgeA, edgeB, contacts);
            }
            else if (sepFaceA >= sepFaceB)
            {
                basisOut = orthonormal(-a.normals[faceA]);
                result = buildFaceManifold(bodyA, bodyB, a, b, true, faceA, contacts);
            }
            else
            {
                basisOut = orthonormal(b.normals[faceB]);
                result = buildFaceManifold(bodyA, bodyB, a, b, false, faceB, contacts);
            }
        }
    }

    delete tmpA;
    delete tmpB;
    return result;
}

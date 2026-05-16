# Copyright (c) 2026 Chris Giles
#
# Permission to use, copy, modify, distribute and sell this software
# and its documentation for any purpose is hereby granted without fee,
# provided that the above copyright notice appear in all copies.
# Chris Giles makes no representations about the suitability
# of this software for any purpose.
# It is provided "as is" without express or implied warranty.

"""Bridge between the Blender scene and the native AVBD solver.

This module gathers AVBD-enabled objects, builds a native solver, and maps
results back onto Blender object transforms. It isolates all coordinate-frame
bookkeeping (object origin vs. centre of mass, baked scale) in one place.
"""

import bmesh
import numpy as np
from mathutils import Matrix, Quaternion, Vector

_native = None


def get_native():
    """Imports the ctypes binding lazily, surfacing a clear error if missing."""
    global _native
    if _native is None:
        try:
            from . import avbd_native
        except OSError as exc:
            raise RuntimeError(
                "AVBD native library failed to load. Build the 'avbd' shared "
                "library and place it under the add-on's lib/ folder. " + str(exc))
        _native = avbd_native
    return _native


# --------------------------------------------------------------------------
# Geometry helpers
# --------------------------------------------------------------------------


def _scaled_local_verts(obj, depsgraph):
    """Returns the evaluated mesh vertices in the object's scaled local frame."""
    eval_obj = obj.evaluated_get(depsgraph)
    mesh = eval_obj.to_mesh()
    scale = obj.matrix_world.decompose()[2]
    verts = [(v.co.x * scale.x, v.co.y * scale.y, v.co.z * scale.z)
             for v in mesh.vertices]
    eval_obj.to_mesh_clear()
    return verts


def _convex_hull(obj, depsgraph):
    """Builds the convex hull of an object, returned as (verts, faces)."""
    verts_in = _scaled_local_verts(obj, depsgraph)
    bm = bmesh.new()
    for co in verts_in:
        bm.verts.new(co)
    bm.verts.ensure_lookup_table()
    result = bmesh.ops.convex_hull(bm, input=bm.verts, use_existing_faces=False)
    # Discard points and geometry that did not end up on the hull. The interior
    # and unused lists can overlap, so de-duplicate before deleting.
    discard = list(dict.fromkeys(result.get("geom_interior", [])
                                 + result.get("geom_unused", [])))
    if discard:
        bmesh.ops.delete(bm, geom=discard, context='VERTS')
    bm.verts.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    for i, v in enumerate(bm.verts):
        v.index = i
    verts = [(v.co.x, v.co.y, v.co.z) for v in bm.verts]
    faces = [[v.index for v in f.verts] for f in bm.faces]
    bm.free()
    return verts, faces


def _hull_com(verts, faces):
    """Centre of mass of a convex polyhedron (tetrahedron decomposition)."""
    volume = 0.0
    cx = cy = cz = 0.0
    for face in faces:
        p0 = verts[face[0]]
        for i in range(1, len(face) - 1):
            p1 = verts[face[i]]
            p2 = verts[face[i + 1]]
            det = (p0[0] * (p1[1] * p2[2] - p1[2] * p2[1])
                   - p0[1] * (p1[0] * p2[2] - p1[2] * p2[0])
                   + p0[2] * (p1[0] * p2[1] - p1[1] * p2[0]))
            volume += det / 6.0
            cx += det * (p0[0] + p1[0] + p2[0])
            cy += det * (p0[1] + p1[1] + p2[1])
            cz += det * (p0[2] + p1[2] + p2[2])
    if abs(volume) < 1.0e-12:
        return Vector((0.0, 0.0, 0.0))
    f = 1.0 / (24.0 * volume)
    return Vector((cx * f, cy * f, cz * f))


def _box_extents(obj):
    """Oriented box size and local centre from the object's scaled bound box."""
    scale = obj.matrix_world.decompose()[2]
    corners = [(c[0] * scale.x, c[1] * scale.y, c[2] * scale.z)
               for c in obj.bound_box]
    lo = Vector((min(c[i] for c in corners) for i in range(3)))
    hi = Vector((max(c[i] for c in corners) for i in range(3)))
    return hi - lo, (hi + lo) * 0.5


# --------------------------------------------------------------------------
# Solver construction / read-back
# --------------------------------------------------------------------------


class BodyRecord:
    """Tracks a baked body and the data needed to map it back to its object."""

    __slots__ = ("body", "local_com")

    def __init__(self, body, local_com):
        self.body = body
        self.local_com = local_com  # COM offset in the object's scaled local frame


class ClothRecord:
    """Tracks a baked cloth and how to map its vertices back to the mesh."""

    __slots__ = ("cloth", "world_to_local")

    def __init__(self, cloth, world_to_local):
        self.cloth = cloth
        self.world_to_local = world_to_local  # matrix_world inverse at build time


def frame_timestep(scene):
    """Seconds of simulation time per Blender frame.

    Synced to the scene frame rate by default so the bake plays at the correct
    speed; otherwise the manually set timestep is used.
    """
    settings = scene.avbd
    if settings.timestep_auto:
        fps = scene.render.fps
        return scene.render.fps_base / fps if fps else 1.0 / 60.0
    return settings.timestep


def iter_bodies(scene):
    """Yields scene objects that are AVBD rigid bodies (active or passive)."""
    for obj in scene.objects:
        if (obj.type == 'MESH' and obj.avbd_body.enabled
                and obj.avbd_body.body_type in ('ACTIVE', 'PASSIVE')):
            yield obj


def iter_cloths(scene):
    """Yields scene objects that are AVBD cloth."""
    for obj in scene.objects:
        if (obj.type == 'MESH' and obj.avbd_body.enabled
                and obj.avbd_body.body_type == 'CLOTH'):
            yield obj


def iter_constraints(scene):
    """Yields scene objects that define an AVBD constraint."""
    for obj in scene.objects:
        if obj.avbd_constraint.is_constraint:
            yield obj


def build_solver(context):
    """Builds a native solver from the current scene.

    Returns (solver, records) where records maps object name -> BodyRecord.
    """
    avbd = get_native()
    scene = context.scene
    settings = scene.avbd
    depsgraph = context.evaluated_depsgraph_get()

    solver = avbd.Solver()
    solver.set_threads(settings.threads)
    # Real substepping: the solver step is the frame time divided by the substep
    # count, so each frame advances `timestep` seconds in finer increments. This
    # is what limits tunnelling of fast-moving bodies.
    substeps = max(1, settings.substeps)
    solver.set_params(
        dt=frame_timestep(scene) / substeps, gravity=settings.gravity,
        iterations=settings.iterations, alpha=settings.alpha,
        beta_lin=settings.beta_lin, beta_ang=settings.beta_ang,
        gamma=settings.gamma)
    solver.set_sleeping(
        enabled=settings.enable_sleeping,
        lin_threshold=settings.sleep_threshold,
        ang_threshold=settings.sleep_threshold)

    records = {}
    for obj in iter_bodies(scene):
        bs = obj.avbd_body
        density = bs.density if bs.body_type == 'ACTIVE' else 0.0
        loc, rot, _scale = obj.matrix_world.decompose()

        if bs.collision_shape == 'BOX':
            size, com = _box_extents(obj)
            pos = loc + rot @ com
            body = solver.add_box((size.x, size.y, size.z), density,
                                  bs.friction, (pos.x, pos.y, pos.z))
        else:
            verts, faces = _convex_hull(obj, depsgraph)
            com = _hull_com(verts, faces)
            recentered = [(v[0] - com.x, v[1] - com.y, v[2] - com.z) for v in verts]
            pos = loc + rot @ com
            body = solver.add_convex(recentered, faces, density, bs.friction,
                                     (pos.x, pos.y, pos.z))

        # The native body starts axis-aligned; apply the object's orientation.
        body.transform = ((pos.x, pos.y, pos.z), (rot.w, rot.x, rot.y, rot.z))
        records[obj.name] = BodyRecord(body, com)

    for obj in iter_cloths(scene):
        cloth_record = _build_cloth(solver, obj)
        if cloth_record is not None:
            records[obj.name] = cloth_record

    _build_constraints(solver, scene, records)
    return solver, records


def _build_cloth(solver, obj):
    """Builds a triangle-FEM cloth from a Blender mesh object."""
    bs = obj.avbd_body
    mesh = obj.data
    mw = obj.matrix_world

    verts = [tuple(mw @ v.co) for v in mesh.vertices]
    mesh.calc_loop_triangles()
    tris = [tuple(lt.vertices) for lt in mesh.loop_triangles]
    if not verts or not tris:
        return None

    cloth = solver.add_cloth(
        verts, tris, density=bs.density, thickness=bs.cloth_thickness,
        youngs_modulus=bs.cloth_youngs, poisson=bs.cloth_poisson,
        bend_stiffness=bs.cloth_bend, particle_radius=bs.cloth_thickness * 0.5,
        friction=bs.friction)

    # Pin the vertices belonging to the chosen vertex group.
    group = obj.vertex_groups.get(bs.cloth_pin_group) if bs.cloth_pin_group else None
    if group is not None:
        gi = group.index
        for v in mesh.vertices:
            for g in v.groups:
                if g.group == gi and g.weight > 0.0:
                    cloth.pin(v.index)
                    break

    return ClothRecord(cloth, mw.inverted())


def _build_constraints(solver, scene, records):
    avbd = get_native()
    for obj in iter_constraints(scene):
        cs = obj.avbd_constraint
        if cs.object_a is None or cs.object_b is None:
            continue
        rec_a = records.get(cs.object_a.name)
        rec_b = records.get(cs.object_b.name)
        if not isinstance(rec_a, BodyRecord) or not isinstance(rec_b, BodyRecord):
            continue  # constraints are only supported between rigid bodies

        # Anchor: the constraint object's world position, expressed in each
        # body's local frame.
        world_point = obj.matrix_world.translation
        pos_a, quat_a = rec_a.body.transform
        pos_b, quat_b = rec_b.body.transform
        anchor_a = Quaternion(quat_a).inverted() @ (world_point - Vector(pos_a))
        anchor_b = Quaternion(quat_b).inverted() @ (world_point - Vector(pos_b))

        if cs.constraint_type == 'SPRING':
            solver.add_spring(rec_a.body, rec_b.body,
                              tuple(anchor_a), tuple(anchor_b),
                              cs.spring_stiffness, cs.spring_rest)
        else:
            stiff_lin = avbd.INFINITY if cs.stiffness_lin < 0 else cs.stiffness_lin
            fracture = avbd.INFINITY if cs.fracture < 0 else cs.fracture
            solver.add_joint(rec_a.body, rec_b.body,
                             tuple(anchor_a), tuple(anchor_b),
                             stiff_lin, cs.stiffness_ang, fracture)


def _apply_transform(obj, record, com_world, quat_wxyz, apply=True):
    """Computes one body's object pose. Writes it onto the object when `apply`
    is set (live preview); a bake skips the write and only keyframes the result.
    Returns (location, rotation)."""
    rotation = Quaternion(quat_wxyz)
    # Object origin = centre of mass minus the (rotated) local COM offset.
    location = Vector(com_world) - (rotation @ record.local_com)
    if apply:
        if obj.rotation_mode != 'QUATERNION':
            obj.rotation_mode = 'QUATERNION'
        # One matrix assignment instead of separate location + rotation writes.
        obj.matrix_basis = Matrix.LocRotScale(location, rotation, obj.scale)
    return location, rotation


def read_body(obj, record):
    """Writes a single native body's transform back onto its Blender object."""
    com_world, quat_wxyz = record.body.transform
    _apply_transform(obj, record, com_world, quat_wxyz)


def read_active_bodies(scene, records, apply=True):
    """Reads every active body's transform back in one batched native call.

    Avoids the per-body ctypes overhead that dominates when streaming hundreds
    of bodies back to Blender each frame. Returns a list of
    (object, location, rotation). With apply=False the object transforms are
    not written (a bake only needs the values to keyframe them).
    """
    items = []
    bodies = []
    for name, record in records.items():
        obj = scene.objects.get(name)
        if obj is None or obj.avbd_body.body_type != 'ACTIVE':
            continue
        items.append((obj, record))
        bodies.append(record.body)
    if not bodies:
        return []

    transforms = get_native().read_transforms(bodies)
    applied = []
    for (obj, record), (com_world, quat_wxyz) in zip(items, transforms):
        location, rotation = _apply_transform(obj, record, com_world, quat_wxyz, apply)
        applied.append((obj, location, rotation))
    return applied


# World->local 4x4 applied to an (N, 3) world-space vertex array, vectorised.
def _to_local(world, world_to_local):
    m = np.array(world_to_local, dtype=np.float32)  # 4x4 matrix rows
    return world @ m[:3, :3].T + m[:3, 3]


def read_cloths(scene, records):
    """Writes every cloth's simulated vertices back onto its Blender mesh."""
    for name, record in records.items():
        if not isinstance(record, ClothRecord):
            continue
        obj = scene.objects.get(name)
        if obj is None:
            continue
        mesh = obj.data
        cloth = record.cloth
        n = cloth.vertex_count
        # Fast path: pull the native buffer, transform every vertex with numpy,
        # and write the mesh in one bulk foreach_set.
        if n == len(mesh.vertices):
            world = np.ctypeslib.as_array(cloth.vertices_buffer()).reshape(n, 3)
            local = _to_local(world, record.world_to_local)
            mesh.vertices.foreach_set("co", local.reshape(-1))
        else:
            w2l = record.world_to_local
            positions = cloth.vertices()
            count = min(len(positions), len(mesh.vertices))
            for i in range(count):
                mesh.vertices[i].co = w2l @ Vector(positions[i])
        mesh.update()


def cloth_local_positions(scene, records):
    """Returns {object_name: flat float32 local-space vertex array} for cloths.

    The flat layout lets the bake cache be applied with a single foreach_set.
    """
    result = {}
    for name, record in records.items():
        if not isinstance(record, ClothRecord):
            continue
        cloth = record.cloth
        n = cloth.vertex_count
        world = np.ctypeslib.as_array(cloth.vertices_buffer()).reshape(n, 3)
        # _to_local returns a fresh array; reshape keeps it alive (buffer reused).
        result[name] = _to_local(world, record.world_to_local).reshape(-1)
    return result

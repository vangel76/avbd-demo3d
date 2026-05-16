# Copyright (c) 2026 Chris Giles
#
# Permission to use, copy, modify, distribute and sell this software
# and its documentation for any purpose is hereby granted without fee,
# provided that the above copyright notice appear in all copies.
# Chris Giles makes no representations about the suitability
# of this software for any purpose.
# It is provided "as is" without express or implied warranty.

"""Blender operators for the AVBD physics add-on: bake, clear, live preview."""

import time

import bmesh
import bpy
import numpy as np
from bpy.app.handlers import persistent
from bpy.props import IntProperty, StringProperty

from . import simulation

_BAKED_PATHS = ("location", "rotation_quaternion")


def _find_baked_fcurve(action, data_path, index):
    """Locates a baked F-curve, handling both legacy and layered (4.4+) actions."""
    layers = getattr(action, "layers", None)
    if layers:
        for layer in layers:
            for strip in layer.strips:
                for bag in getattr(strip, "channelbags", []):
                    fc = bag.fcurves.find(data_path, index=index)
                    if fc:
                        return fc
        return None
    legacy = getattr(action, "fcurves", None)
    return legacy.find(data_path, index=index) if legacy is not None else None


def _bulk_keyframe(obj, frames, locs, rots):
    """Writes all collected pose samples as keyframes in one bulk pass per
    F-curve (keyframe_points.add + foreach_set). Far faster than a per-frame
    keyframe_insert when baking thousands of bodies.

    frames: list of int. locs: list of (x, y, z). rots: list of (w, x, y, z).
    """
    n = len(frames)
    if n == 0:
        return

    if obj.animation_data is None:
        obj.animation_data_create()
    if obj.animation_data.action is None:
        obj.animation_data.action = bpy.data.actions.new(obj.name + "_AVBD")

    # rotation_quaternion keyframes only drive the object in quaternion mode.
    if obj.rotation_mode != 'QUATERNION':
        obj.rotation_mode = 'QUATERNION'

    # Clear any earlier bake, then create the F-curves through the official
    # keyframe_insert path (one key each) so this stays correct across legacy
    # and layered actions; the remaining keys are bulk-filled below.
    _remove_baked_keyframes(obj)
    obj.keyframe_insert(data_path="location", frame=frames[0])
    obj.keyframe_insert(data_path="rotation_quaternion", frame=frames[0])

    action = obj.animation_data.action
    fr = np.asarray(frames, dtype=np.float32)
    loc = np.asarray(locs, dtype=np.float32)  # (n, 3)
    rot = np.asarray(rots, dtype=np.float32)  # (n, 4)
    channels = (
        ("location", 0, loc[:, 0]),
        ("location", 1, loc[:, 1]),
        ("location", 2, loc[:, 2]),
        ("rotation_quaternion", 0, rot[:, 0]),
        ("rotation_quaternion", 1, rot[:, 1]),
        ("rotation_quaternion", 2, rot[:, 2]),
        ("rotation_quaternion", 3, rot[:, 3]),
    )
    for data_path, index, values in channels:
        fc = _find_baked_fcurve(action, data_path, index)
        if fc is None:
            continue
        existing = len(fc.keyframe_points)
        if existing < n:
            fc.keyframe_points.add(n - existing)
        co = np.empty(2 * n, dtype=np.float32)
        co[0::2] = fr        # keyframe x = frame
        co[1::2] = values    # keyframe y = channel value
        fc.keyframe_points.foreach_set("co", co)
        fc.update()


def _remove_baked_keyframes(obj):
    """Removes baked location/rotation F-curves, handling Blender's layered
    actions (4.4+). The compatibility `action.fcurves` accessor is read-only
    there, so removal must go through the channelbags."""
    anim = obj.animation_data
    if not anim or not anim.action:
        return 0
    action = anim.action
    removed = 0

    layers = getattr(action, "layers", None)
    if layers:
        for layer in layers:
            for strip in layer.strips:
                for bag in getattr(strip, "channelbags", []):
                    for fc in list(bag.fcurves):
                        if fc.data_path in _BAKED_PATHS:
                            bag.fcurves.remove(fc)
                            removed += 1
    else:
        # Legacy (pre-4.4) actions expose F-curves directly; on 4.4+/5.x the
        # compatibility accessor is gone, so guard it.
        legacy = getattr(action, "fcurves", None)
        if legacy is not None:
            for fc in list(legacy):
                if fc.data_path in _BAKED_PATHS:
                    legacy.remove(fc)
                    removed += 1
    return removed


# Session cloth cache: object name -> {frame: [local-space vertex tuples]}.
# Applied by a frame-change handler. Cloth deformation cannot be keyframed per
# vertex practically, so it is cached here. The cache is not saved with the
# .blend file; re-bake after reloading.
_cloth_cache = {}


@persistent
def apply_cloth_cache(scene, depsgraph=None):
    """frame_change_post handler: applies cached cloth vertices for the frame.

    Marked persistent so it survives .blend loads."""
    for name, frames in _cloth_cache.items():
        verts = frames.get(scene.frame_current)
        if verts is None:
            continue
        obj = scene.objects.get(name)
        if obj is None:
            continue
        mesh = obj.data
        # The cache stores a flat float array; apply it in one bulk write.
        if len(verts) == len(mesh.vertices) * 3:
            mesh.vertices.foreach_set("co", verts)
        else:
            count = min(len(verts) // 3, len(mesh.vertices))
            for i in range(count):
                mesh.vertices[i].co = verts[i * 3:i * 3 + 3]
        mesh.update()


class AVBD_OT_bake(bpy.types.Operator):
    """Run the AVBD simulation and bake the result to keyframes"""

    bl_idname = "avbd.bake"
    bl_label = "Bake AVBD Simulation"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        scene = context.scene
        settings = scene.avbd

        # Debug timing: total + per-section, printed to the console after the bake.
        t_total = time.perf_counter()
        t_build = time.perf_counter()
        try:
            solver, records = simulation.build_solver(context)
        except RuntimeError as exc:
            self.report({'ERROR'}, str(exc))
            return {'CANCELLED'}
        t_build = time.perf_counter() - t_build
        t_solve = t_read = t_cloth = 0.0

        if not records:
            solver.destroy()
            self.report({'WARNING'}, "No AVBD rigid bodies found in the scene")
            return {'CANCELLED'}

        f0 = settings.frame_start
        f1 = max(settings.frame_end, f0)
        window = context.window_manager
        window.progress_begin(f0, f1)

        def cache_cloth(frame):
            for name, verts in simulation.cloth_local_positions(scene, records).items():
                _cloth_cache.setdefault(name, {})[frame] = verts

        # Fresh cloth cache for this bake.
        _cloth_cache.clear()

        # Per active object, accumulate (frame, location, quaternion) samples.
        # The keyframes are written in one bulk pass after the simulation rather
        # than an O(frames x bodies) keyframe_insert every frame.
        tracks = {}

        # apply=False: a bake only needs the transform values to keyframe them,
        # so the per-frame object writes (and the depsgraph churn they cause)
        # are skipped -- the objects are driven by the keyframes afterwards.
        def record_pose(frame):
            for obj, loc, rot in simulation.read_active_bodies(scene, records, apply=False):
                track = tracks.get(obj)
                if track is None:
                    track = tracks[obj] = ([], [], [])
                track[0].append(frame)
                track[1].append((loc.x, loc.y, loc.z))
                track[2].append((rot.w, rot.x, rot.y, rot.z))

        # Capture the initial pose of every active body and cloth at f0.
        ts = time.perf_counter()
        record_pose(f0)
        t_read += time.perf_counter() - ts
        ts = time.perf_counter()
        cache_cloth(f0)
        t_cloth += time.perf_counter() - ts

        for frame in range(f0 + 1, f1 + 1):
            ts = time.perf_counter()
            for _ in range(settings.substeps):
                solver.step()
            t_solve += time.perf_counter() - ts

            ts = time.perf_counter()
            record_pose(frame)
            t_read += time.perf_counter() - ts

            ts = time.perf_counter()
            cache_cloth(frame)
            t_cloth += time.perf_counter() - ts

            window.progress_update(frame)

        # Write every collected track as keyframes in one bulk pass per object.
        t_key = time.perf_counter()
        for obj, (frames, locs, rots) in tracks.items():
            _bulk_keyframe(obj, frames, locs, rots)
        t_key = time.perf_counter() - t_key

        window.progress_end()
        solver.destroy()
        scene.frame_set(f0)

        # Debug timing report to the console.
        total = time.perf_counter() - t_total
        nframes = f1 - f0 + 1
        other = max(0.0, total - t_build - t_solve - t_read - t_cloth - t_key)
        print("[AVBD] bake: {} bodies, {} frames -> {:.2f} s total "
              "({:.1f} ms/frame)".format(len(records), nframes, total,
                                         1000.0 * total / nframes))
        for label, secs in (("build solver", t_build), ("solver step", t_solve),
                            ("pose readback", t_read), ("cloth cache", t_cloth),
                            ("keyframing", t_key), ("other", other)):
            print("[AVBD]   {:<14} {:8.2f} s  {:5.1f}%".format(
                label, secs, 100.0 * secs / total if total else 0.0))

        self.report({'INFO'}, "Baked {} bodies, {} frames in {:.1f} s".format(
            len(records), nframes, total))
        return {'FINISHED'}


class AVBD_OT_clear_bake(bpy.types.Operator):
    """Remove baked AVBD keyframes from all rigid bodies"""

    bl_idname = "avbd.clear_bake"
    bl_label = "Clear AVBD Bake"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        cleared = 0
        for obj in simulation.iter_bodies(context.scene):
            if _remove_baked_keyframes(obj):
                cleared += 1
        _cloth_cache.clear()
        self.report({'INFO'}, "Cleared baked keyframes from {} bodies".format(cleared))
        return {'FINISHED'}


class AVBD_OT_live_preview(bpy.types.Operator):
    """Step the AVBD solver live in the viewport (press ESC to stop)"""

    bl_idname = "avbd.live_preview"
    bl_label = "AVBD Live Preview"

    _timer = None
    _solver = None
    _records = None
    _saved = None
    _saved_cloths = None
    _ticks = 0
    _solve_ms = 0.0
    _wall_start = 0.0

    def modal(self, context, event):
        window = context.window_manager
        if not window.avbd_live_running or event.type == 'ESC':
            self.cancel(context)
            return {'CANCELLED'}

        if event.type == 'TIMER':
            settings = context.scene.avbd
            t0 = time.perf_counter()
            for _ in range(settings.substeps):
                self._solver.step()
            simulation.read_active_bodies(context.scene, self._records)
            simulation.read_cloths(context.scene, self._records)
            self._solve_ms += (time.perf_counter() - t0) * 1000.0
            self._ticks += 1
            for area in context.screen.areas:
                if area.type == 'VIEW_3D':
                    area.tag_redraw()
            return {'RUNNING_MODAL'}

        # Pass every other event through so the user keeps full viewport
        # control (orbit, pan, zoom, selection) while the preview runs.
        return {'PASS_THROUGH'}

    def execute(self, context):
        try:
            self._solver, self._records = simulation.build_solver(context)
        except RuntimeError as exc:
            self.report({'ERROR'}, str(exc))
            return {'CANCELLED'}

        if not self._records:
            self._solver.destroy()
            self.report({'WARNING'}, "No AVBD rigid bodies found in the scene")
            return {'CANCELLED'}

        # Save transforms so they can be restored when the preview ends. The
        # basis matrix is stored because read_body switches objects to
        # quaternion mode, and a later rotation_mode change would otherwise
        # convert (and discard) the rotation.
        self._saved = {}
        self._saved_cloths = {}
        for name in self._records:
            obj = context.scene.objects.get(name)
            if obj is None:
                continue
            if obj.avbd_body.body_type == 'CLOTH':
                self._saved_cloths[name] = [v.co.copy() for v in obj.data.vertices]
            else:
                self._saved[name] = (obj.rotation_mode, obj.matrix_basis.copy())

        window = context.window_manager
        window.avbd_live_running = True
        self._ticks = 0
        self._solve_ms = 0.0
        self._wall_start = time.perf_counter()
        self._timer = window.event_timer_add(
            simulation.frame_timestep(context.scene), window=context.window)
        window.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def cancel(self, context):
        window = context.window_manager
        if self._timer is not None:
            window.event_timer_remove(self._timer)
            self._timer = None

        # Report measured performance for the preview session.
        if self._ticks > 0:
            elapsed = time.perf_counter() - self._wall_start
            fps = self._ticks / elapsed if elapsed > 0.0 else 0.0
            wall_ms = 1000.0 * elapsed / self._ticks
            solve_ms = self._solve_ms / self._ticks
            bodies = len(self._records) if self._records else 0
            msg = ("AVBD preview: {} bodies | {} frames | {:.1f} fps | "
                   "{:.1f} ms/frame wall | {:.2f} ms/frame solve+readback"
                   ).format(bodies, self._ticks, fps, wall_ms, solve_ms)
            print(msg)
            self.report({'INFO'}, msg)
        if self._solver is not None:
            self._solver.destroy()
            self._solver = None
        # Restore the pre-preview transforms. The rotation mode is set first so
        # that assigning the basis matrix stores the rotation back into the
        # original mode's fields (euler/quaternion/axis-angle).
        if self._saved:
            for name, (mode, matrix) in self._saved.items():
                obj = context.scene.objects.get(name)
                if obj is None:
                    continue
                obj.rotation_mode = mode
                obj.matrix_basis = matrix
        # Restore the pre-preview cloth meshes.
        if self._saved_cloths:
            for name, coords in self._saved_cloths.items():
                obj = context.scene.objects.get(name)
                if obj is None:
                    continue
                mesh = obj.data
                for i in range(min(len(coords), len(mesh.vertices))):
                    mesh.vertices[i].co = coords[i]
                mesh.update()
        self._records = None
        self._saved_cloths = None
        window.avbd_live_running = False


class AVBD_OT_stop_live_preview(bpy.types.Operator):
    """Stop the running AVBD live preview"""

    bl_idname = "avbd.stop_live_preview"
    bl_label = "Stop AVBD Live Preview"

    def execute(self, context):
        context.window_manager.avbd_live_running = False
        return {'FINISHED'}


class AVBD_OT_add_constraint(bpy.types.Operator):
    """Create an AVBD constraint linking the two selected rigid bodies"""

    bl_idname = "avbd.add_constraint"
    bl_label = "Add AVBD Constraint"
    bl_options = {'REGISTER', 'UNDO'}

    constraint_type: StringProperty(default='JOINT')

    def execute(self, context):
        selected = [o for o in context.selected_objects if o.type == 'MESH']
        if len(selected) < 2:
            self.report({'ERROR'}, "Select exactly two mesh objects to link")
            return {'CANCELLED'}

        body_a, body_b = selected[0], selected[1]
        midpoint = (body_a.matrix_world.translation
                    + body_b.matrix_world.translation) * 0.5

        empty = bpy.data.objects.new(
            "AVBD_" + self.constraint_type.capitalize(), None)
        empty.empty_display_type = 'SPHERE'
        empty.empty_display_size = 0.25
        empty.location = midpoint
        context.collection.objects.link(empty)

        cs = empty.avbd_constraint
        cs.is_constraint = True
        cs.constraint_type = self.constraint_type
        cs.object_a = body_a
        cs.object_b = body_b

        for obj in context.selected_objects:
            obj.select_set(False)
        empty.select_set(True)
        context.view_layer.objects.active = empty
        self.report({'INFO'}, "Created {} constraint".format(self.constraint_type))
        return {'FINISHED'}


def _unit_cube_mesh():
    """Returns a shared 1x1x1 cube mesh, creating it once."""
    mesh = bpy.data.meshes.get("AVBD_TestCube")
    if mesh is None:
        mesh = bpy.data.meshes.new("AVBD_TestCube")
        bm = bmesh.new()
        bmesh.ops.create_cube(bm, size=1.0)
        bm.to_mesh(mesh)
        bm.free()
    return mesh


class AVBD_OT_make_test_scene(bpy.types.Operator):
    """Build a staggered brick wall on solid ground for quick testing"""

    bl_idname = "avbd.make_test_scene"
    bl_label = "Create Brick Wall Test Scene"
    bl_options = {'REGISTER', 'UNDO'}

    width: IntProperty(
        name="Bricks Wide", default=24, min=2, max=80,
        description="Number of bricks per row")
    height: IntProperty(
        name="Rows High", default=12, min=1, max=40,
        description="Number of brick rows")

    def execute(self, context):
        # Brick and ground dimensions (a unit cube is scaled to each).
        bx, by, bz = 1.0, 2.0, 0.5
        cube = _unit_cube_mesh()

        collection = bpy.data.collections.new("AVBD Test Scene")
        context.scene.collection.children.link(collection)

        def add(name, scale, location, body_type):
            obj = bpy.data.objects.new(name, cube)
            obj.scale = scale
            obj.location = location
            collection.objects.link(obj)
            b = obj.avbd_body
            b.enabled = True
            b.body_type = body_type
            b.collision_shape = 'BOX'
            b.density = 1.0
            b.friction = 0.6
            return obj

        # Solid passive ground; its top surface sits at z = 0.25.
        add("AVBD_Ground", (self.width + 6.0, 10.0, 0.5), (0, 0, 0), 'PASSIVE')

        # Staggered (running-bond) wall of active bricks.
        count = 0
        for row in range(self.height):
            staggered = row & 1
            offset = bx * 0.5 if staggered else 0.0
            bricks = self.width - 1 if staggered else self.width
            z = 0.25 + bz * 0.5 + row * bz
            for col in range(bricks):
                x = (col - self.width / 2.0) * bx + offset
                add("AVBD_Brick", (bx, by, bz), (x, 0.0, z), 'ACTIVE')
                count += 1

        settings = context.scene.avbd
        settings.frame_start = 1
        settings.frame_end = 150

        self.report({'INFO'}, "Created brick wall: {} bricks".format(count))
        return {'FINISHED'}


class AVBD_OT_make_cloth_scene(bpy.types.Operator):
    """Build a cloth sheet draping over a box for quick cloth testing"""

    bl_idname = "avbd.make_cloth_scene"
    bl_label = "Create Cloth Drape Test Scene"
    bl_options = {'REGISTER', 'UNDO'}

    resolution: IntProperty(
        name="Cloth Resolution", default=20, min=4, max=100,
        description="Grid subdivisions of the cloth sheet")

    def execute(self, context):
        cube = _unit_cube_mesh()
        collection = bpy.data.collections.new("AVBD Cloth Scene")
        context.scene.collection.children.link(collection)

        def add_box(name, scale, location, body_type):
            obj = bpy.data.objects.new(name, cube)
            obj.scale = scale
            obj.location = location
            collection.objects.link(obj)
            b = obj.avbd_body
            b.enabled = True
            b.body_type = body_type
            b.collision_shape = 'BOX'
            b.friction = 0.5
            return obj

        # Solid passive ground (top surface at z = 0.25) and a box obstacle.
        add_box("AVBD_Ground", (8.0, 8.0, 0.5), (0, 0, 0), 'PASSIVE')
        add_box("AVBD_Obstacle", (1.5, 1.5, 1.5), (0, 0, 1.0), 'PASSIVE')

        # Cloth sheet hovering above, which falls and drapes over the obstacle.
        bpy.ops.mesh.primitive_grid_add(
            x_subdivisions=self.resolution, y_subdivisions=self.resolution,
            size=4.0, location=(0, 0, 3.0))
        cloth = context.active_object
        for c in list(cloth.users_collection):
            c.objects.unlink(cloth)
        collection.objects.link(cloth)
        cloth.name = "AVBD_Cloth"

        b = cloth.avbd_body
        b.enabled = True
        b.body_type = 'CLOTH'
        b.density = 1.0
        b.friction = 0.5
        b.cloth_youngs = 2000.0
        b.cloth_poisson = 0.3
        b.cloth_bend = 0.3
        b.cloth_thickness = 0.02

        settings = context.scene.avbd
        settings.frame_start = 1
        settings.frame_end = 150

        self.report({'INFO'}, "Created cloth drape scene")
        return {'FINISHED'}


_CLASSES = (
    AVBD_OT_bake,
    AVBD_OT_clear_bake,
    AVBD_OT_live_preview,
    AVBD_OT_stop_live_preview,
    AVBD_OT_add_constraint,
    AVBD_OT_make_test_scene,
    AVBD_OT_make_cloth_scene,
)


def register():
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)

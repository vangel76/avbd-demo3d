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
from bpy.app.handlers import persistent
from bpy.props import IntProperty, StringProperty

from . import simulation

_BAKED_PATHS = ("location", "rotation_quaternion")


def _keyframe(obj, frame):
    for path in _BAKED_PATHS:
        obj.keyframe_insert(data_path=path, frame=frame)


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
        # Legacy (pre-4.4) actions
        for fc in list(action.fcurves):
            if fc.data_path in _BAKED_PATHS:
                action.fcurves.remove(fc)
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
        count = min(len(verts), len(mesh.vertices))
        for i in range(count):
            mesh.vertices[i].co = verts[i]
        mesh.update()


class AVBD_OT_bake(bpy.types.Operator):
    """Run the AVBD simulation and bake the result to keyframes"""

    bl_idname = "avbd.bake"
    bl_label = "Bake AVBD Simulation"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        scene = context.scene
        settings = scene.avbd

        try:
            solver, records = simulation.build_solver(context)
        except RuntimeError as exc:
            self.report({'ERROR'}, str(exc))
            return {'CANCELLED'}

        if not records:
            solver.destroy()
            self.report({'WARNING'}, "No AVBD rigid bodies found in the scene")
            return {'CANCELLED'}

        f0 = settings.frame_start
        f1 = max(settings.frame_end, f0)
        window = context.window_manager
        window.progress_begin(f0, f1)

        def keyframe_active(frame):
            for name in records:
                obj = scene.objects.get(name)
                if obj is not None and obj.avbd_body.body_type == 'ACTIVE':
                    _keyframe(obj, frame)

        def cache_cloth(frame):
            for name, verts in simulation.cloth_local_positions(scene, records).items():
                _cloth_cache.setdefault(name, {})[frame] = verts

        # Fresh cloth cache for this bake.
        _cloth_cache.clear()

        # Capture the initial pose of every active body and cloth at f0.
        simulation.read_active_bodies(scene, records)
        keyframe_active(f0)
        cache_cloth(f0)

        for frame in range(f0 + 1, f1 + 1):
            for _ in range(settings.substeps):
                solver.step()
            simulation.read_active_bodies(scene, records)
            keyframe_active(frame)
            cache_cloth(frame)
            window.progress_update(frame)

            # Show baking progress in the viewport every 10 frames. redraw_timer
            # forces a repaint mid-operator; it has no window in background mode.
            if (frame - f0) % 10 == 0:
                scene.frame_set(frame)
                try:
                    bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
                except RuntimeError:
                    pass

        window.progress_end()
        solver.destroy()
        scene.frame_set(f0)
        self.report({'INFO'}, "Baked {} bodies over frames {}-{}".format(
            len(records), f0, f1))
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

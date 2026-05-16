# Copyright (c) 2026 Chris Giles
#
# Permission to use, copy, modify, distribute and sell this software
# and its documentation for any purpose is hereby granted without fee,
# provided that the above copyright notice appear in all copies.
# Chris Giles makes no representations about the suitability
# of this software for any purpose.
# It is provided "as is" without express or implied warranty.

"""Blender UI panels for the AVBD physics add-on."""

import bpy


class AVBD_PT_scene(bpy.types.Panel):
    bl_label = "AVBD Physics"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "scene"

    def draw(self, context):
        layout = self.layout
        settings = context.scene.avbd

        col = layout.column(align=True)
        col.prop(settings, "gravity")
        col.prop(settings, "timestep_auto")
        row = col.row()
        row.enabled = not settings.timestep_auto
        row.prop(settings, "timestep")
        col.prop(settings, "iterations")
        col.prop(settings, "substeps")
        col.prop(settings, "threads")
        col.prop(settings, "enable_sleeping")
        sub = col.row()
        sub.enabled = settings.enable_sleeping
        sub.prop(settings, "sleep_threshold")

        box = layout.box()
        box.label(text="Advanced (AVBD tuning)")
        adv = box.column(align=True)
        adv.prop(settings, "alpha")
        adv.prop(settings, "beta_lin")
        adv.prop(settings, "beta_ang")
        adv.prop(settings, "gamma")

        frames = layout.row(align=True)
        frames.prop(settings, "frame_start")
        frames.prop(settings, "frame_end")

        row = layout.row(align=True)
        row.scale_y = 1.4
        row.operator("avbd.bake", icon='PHYSICS')
        layout.operator("avbd.clear_bake", icon='TRASH')


class AVBD_PT_body(bpy.types.Panel):
    bl_label = "AVBD Rigid Body"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "physics"

    @classmethod
    def poll(cls, context):
        return context.object is not None and context.object.type == 'MESH'

    def draw(self, context):
        layout = self.layout
        body = context.object.avbd_body

        layout.prop(body, "enabled")
        col = layout.column()
        col.enabled = body.enabled
        col.prop(body, "body_type")

        if body.body_type == 'CLOTH':
            col.prop(body, "density")
            col.prop(body, "friction")
            col.separator()
            col.prop(body, "cloth_youngs")
            col.prop(body, "cloth_poisson")
            col.prop(body, "cloth_bend")
            col.prop(body, "cloth_thickness")
            col.prop_search(body, "cloth_pin_group",
                            context.object, "vertex_groups")
        else:
            col.prop(body, "collision_shape")
            if body.body_type == 'ACTIVE':
                col.prop(body, "density")
            col.prop(body, "friction")


class AVBD_PT_constraint(bpy.types.Panel):
    bl_label = "AVBD Constraint"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "physics"

    @classmethod
    def poll(cls, context):
        return context.object is not None

    def draw(self, context):
        layout = self.layout
        cs = context.object.avbd_constraint

        layout.prop(cs, "is_constraint")
        col = layout.column()
        col.enabled = cs.is_constraint
        col.prop(cs, "constraint_type")
        col.prop(cs, "object_a")
        col.prop(cs, "object_b")

        if cs.constraint_type == 'JOINT':
            col.prop(cs, "stiffness_lin")
            col.prop(cs, "stiffness_ang")
            col.prop(cs, "fracture")
        else:
            col.prop(cs, "spring_stiffness")
            col.prop(cs, "spring_rest")


class AVBD_PT_viewport(bpy.types.Panel):
    bl_label = "AVBD Physics"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "AVBD"

    def draw(self, context):
        layout = self.layout
        running = context.window_manager.avbd_live_running

        row = layout.row()
        row.scale_y = 1.4
        if running:
            row.operator("avbd.stop_live_preview", icon='PAUSE')
        else:
            row.operator("avbd.live_preview", text="Live Preview", icon='PLAY')

        layout.separator()
        layout.operator("avbd.bake", icon='PHYSICS')
        layout.operator("avbd.clear_bake", icon='TRASH')

        layout.separator()
        layout.operator("avbd.make_test_scene", icon='MOD_BUILD')
        layout.operator("avbd.make_cloth_scene", icon='MOD_CLOTH')

        layout.separator()
        col = layout.column(align=True)
        col.label(text="Constraints:")
        op = col.operator("avbd.add_constraint", text="Add Joint", icon='CONSTRAINT')
        op.constraint_type = 'JOINT'
        op = col.operator("avbd.add_constraint", text="Add Spring", icon='FORCE_HARMONIC')
        op.constraint_type = 'SPRING'


_CLASSES = (
    AVBD_PT_scene,
    AVBD_PT_body,
    AVBD_PT_constraint,
    AVBD_PT_viewport,
)


def register():
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)

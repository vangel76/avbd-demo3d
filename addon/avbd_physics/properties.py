# Copyright (c) 2026 Chris Giles
#
# Permission to use, copy, modify, distribute and sell this software
# and its documentation for any purpose is hereby granted without fee,
# provided that the above copyright notice appear in all copies.
# Chris Giles makes no representations about the suitability
# of this software for any purpose.
# It is provided "as is" without express or implied warranty.

"""Blender property groups for the AVBD physics add-on."""

import bpy
from bpy.props import (
    BoolProperty,
    EnumProperty,
    FloatProperty,
    IntProperty,
    PointerProperty,
    StringProperty,
)


class AvbdSceneSettings(bpy.types.PropertyGroup):
    """Solver settings, stored per scene."""

    gravity: FloatProperty(
        name="Gravity", default=-9.81,
        description="Gravitational acceleration along world Z")
    timestep_auto: BoolProperty(
        name="Sync Timestep to Scene FPS", default=True,
        description="Use the scene frame rate as the timestep so the baked "
                    "simulation plays back at the correct speed")
    timestep: FloatProperty(
        name="Timestep", default=1.0 / 60.0, min=1.0e-4, max=1.0,
        description="Manual simulation step size in seconds (when not synced "
                    "to the scene FPS)")
    iterations: IntProperty(
        name="Iterations", default=10, min=1, max=200,
        description="AVBD solver iterations per step")
    alpha: FloatProperty(
        name="Stabilization", default=0.9, min=0.0, max=1.0,
        description="AVBD alpha. Lower values push penetrating objects apart "
                    "faster (less sinking); higher values are smoother but slower")
    beta_lin: FloatProperty(
        name="Penalty (Linear)", default=100000.0, min=0.0,
        description="Penalty ramping for linear constraints (AVBD beta). "
                    "Raise it if stacked objects sink into each other")
    beta_ang: FloatProperty(
        name="Penalty (Angular)", default=100.0, min=0.0,
        description="Penalty ramping for angular constraints (AVBD beta)")
    gamma: FloatProperty(
        name="Warmstart Decay", default=0.999, min=0.0, max=1.0,
        description="Penalty/dual decay between steps (AVBD gamma)")
    threads: IntProperty(
        name="Threads", default=0, min=0, max=256,
        description="CPU worker threads (0 = match the hardware)")
    enable_sleeping: BoolProperty(
        name="Sleeping", default=True,
        description="Freeze resting bodies: removes jitter on settled objects "
                    "and speeds up scenes once they come to rest")
    sleep_threshold: FloatProperty(
        name="Sleep Threshold", default=0.05, min=0.0, soft_max=1.0,
        description="Velocity below which a body is treated as at rest")
    substeps: IntProperty(
        name="Substeps", default=2, min=1, max=32,
        description="Solver steps per frame, each at timestep/substeps. "
                    "Raise it to stop fast-moving objects tunnelling")
    frame_start: IntProperty(name="Start Frame", default=1, min=0)
    frame_end: IntProperty(name="End Frame", default=120, min=1)


class AvbdBodySettings(bpy.types.PropertyGroup):
    """Per-object rigid body settings."""

    enabled: BoolProperty(
        name="AVBD Rigid Body", default=False,
        description="Include this object in the AVBD simulation")
    body_type: EnumProperty(
        name="Type",
        items=[
            ('ACTIVE', "Active", "Dynamic rigid body driven by the simulation"),
            ('PASSIVE', "Passive", "Static collider that does not move"),
            ('CLOTH', "Cloth", "Triangle-FEM cloth simulated from this mesh"),
        ],
        default='ACTIVE')
    collision_shape: EnumProperty(
        name="Shape",
        items=[
            ('CONVEX_HULL', "Convex Hull", "Convex hull of the object's mesh"),
            ('BOX', "Box", "Oriented bounding box of the object"),
        ],
        default='CONVEX_HULL')
    density: FloatProperty(
        name="Density", default=1.0, min=0.0,
        description="Mass per unit volume (relative units)")
    friction: FloatProperty(
        name="Friction", default=0.5, min=0.0,
        description="Coulomb friction coefficient")

    # --- Cloth (triangle FEM) settings ---
    cloth_youngs: FloatProperty(
        name="Stiffness", default=1000.0, min=0.0,
        description="Young's modulus of the cloth membrane")
    cloth_poisson: FloatProperty(
        name="Poisson Ratio", default=0.3, min=0.0, max=0.49,
        description="Cloth Poisson ratio (lateral contraction under stretch)")
    cloth_bend: FloatProperty(
        name="Bend Stiffness", default=0.5, min=0.0,
        description="Resistance to folding")
    cloth_thickness: FloatProperty(
        name="Thickness", default=0.01, min=1.0e-4,
        description="Cloth thickness; also sets the collision radius")
    cloth_pin_group: StringProperty(
        name="Pinned Group",
        description="Vertex group whose vertices are pinned in place")


class AvbdConstraintSettings(bpy.types.PropertyGroup):
    """Per-object constraint settings (lives on an Empty linking two bodies)."""

    is_constraint: BoolProperty(name="AVBD Constraint", default=False)
    constraint_type: EnumProperty(
        name="Type",
        items=[
            ('JOINT', "Joint", "Revolute joint / attachment between two bodies"),
            ('SPRING', "Spring", "Distance spring between two bodies"),
        ],
        default='JOINT')
    object_a: PointerProperty(name="Body A", type=bpy.types.Object)
    object_b: PointerProperty(name="Body B", type=bpy.types.Object)
    stiffness_lin: FloatProperty(
        name="Linear Stiffness", default=-1.0,
        description="Joint linear stiffness; -1 means a hard (infinite) constraint")
    stiffness_ang: FloatProperty(
        name="Angular Stiffness", default=0.0, min=0.0,
        description="Joint angular stiffness (0 = free rotation)")
    fracture: FloatProperty(
        name="Fracture Force", default=-1.0,
        description="Force at which the joint breaks; -1 means unbreakable")
    spring_stiffness: FloatProperty(
        name="Spring Stiffness", default=100.0, min=0.0)
    spring_rest: FloatProperty(
        name="Rest Length", default=-1.0,
        description="Spring rest length; -1 means the current distance")


_CLASSES = (AvbdSceneSettings, AvbdBodySettings, AvbdConstraintSettings)


def register():
    for cls in _CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.avbd = PointerProperty(type=AvbdSceneSettings)
    bpy.types.Object.avbd_body = PointerProperty(type=AvbdBodySettings)
    bpy.types.Object.avbd_constraint = PointerProperty(type=AvbdConstraintSettings)


def unregister():
    del bpy.types.Object.avbd_constraint
    del bpy.types.Object.avbd_body
    del bpy.types.Scene.avbd
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)

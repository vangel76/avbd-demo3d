"""Headless Blender test for the AVBD Physics add-on.

Run with:
    blender --background --python tests/test_blender.py

It registers the add-on, builds a small scene, bakes the simulation, and checks
that the dynamic bodies fell and settled onto the passive ground.
"""

import os
import sys

import bpy

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, "addon"))

_failures = 0


def check(name, value, expected, tol):
    global _failures
    ok = abs(value - expected) <= tol
    print("[{}] {:<34} value={:.4f} expected={:.4f} tol={:.3f}".format(
        "PASS" if ok else "FAIL", name, value, expected, tol))
    if not ok:
        _failures += 1


def make_body(obj, body_type, shape, density=1.0, friction=0.5):
    b = obj.avbd_body
    b.enabled = True
    b.body_type = body_type
    b.collision_shape = shape
    b.density = density
    b.friction = friction


def main():
    import avbd_physics
    avbd_physics.register()

    # Fresh empty scene.
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene

    # Ground: a flat passive box.
    bpy.ops.mesh.primitive_cube_add(size=2.0, location=(0, 0, 0))
    ground = bpy.context.active_object
    ground.scale = (10.0, 10.0, 0.25)  # 20 x 20 x 0.5, top surface at z = 0.25
    make_body(ground, 'PASSIVE', 'BOX')

    # Two active boxes that should fall and stack on the ground.
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=(0, 0, 3.0))
    lower = bpy.context.active_object
    make_body(lower, 'ACTIVE', 'BOX')

    bpy.ops.mesh.primitive_cube_add(size=1.0, location=(0.1, 0, 5.0))
    upper = bpy.context.active_object
    make_body(upper, 'ACTIVE', 'BOX')

    # An active convex-hull body (Suzanne) dropped onto the ground.
    bpy.ops.mesh.primitive_monkey_add(size=1.0, location=(4.0, 0, 4.0))
    monkey = bpy.context.active_object
    make_body(monkey, 'ACTIVE', 'CONVEX_HULL')

    settings = scene.avbd
    settings.frame_start = 1
    settings.frame_end = 150
    settings.threads = 4

    result = bpy.ops.avbd.bake()
    print("bake result:", result)

    scene.frame_set(settings.frame_end)
    bpy.context.view_layer.update()

    # Boxes settle on the ground: ground top 0.25 + half box 0.5 = 0.75.
    check("lower box settled Z", lower.location.z, 0.75, 0.2)
    check("upper box settled Z", upper.location.z, 1.75, 0.35)
    # Suzanne fell from z=4 onto the ground.
    check("monkey fell below start", 1.0 if monkey.location.z < 3.0 else 0.0, 1.0, 0.5)
    check("monkey above ground", 1.0 if monkey.location.z > 0.0 else 0.0, 1.0, 0.5)

    # The bake must have written keyframes.
    keyed = lower.animation_data is not None and lower.animation_data.action is not None
    check("keyframes written", 1.0 if keyed else 0.0, 1.0, 0.5)

    test_brick_wall_scene()
    test_cloth()
    test_cloth_scene()

    print()
    if _failures:
        print("BLENDER TESTS FAILED ({} failures)".format(_failures))
        sys.exit(1)
    print("ALL BLENDER TESTS PASSED")


def test_cloth():
    """A cloth grid pinned at one edge is baked; the free edge must droop."""
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene

    # A subdivided grid plane lying flat at z = 3.
    bpy.ops.mesh.primitive_grid_add(x_subdivisions=8, y_subdivisions=8,
                                    size=2.0, location=(0, 0, 3))
    cloth = bpy.context.active_object
    b = cloth.avbd_body
    b.enabled = True
    b.body_type = 'CLOTH'
    b.cloth_youngs = 2000.0
    b.cloth_bend = 0.2

    # Pin the vertices along the +Y edge.
    group = cloth.vertex_groups.new(name="Pinned")
    pinned = [v.index for v in cloth.data.vertices if v.co.y > 0.99]
    group.add(pinned, 1.0, 'REPLACE')
    b.cloth_pin_group = "Pinned"

    start_min_z = min(v.co.z for v in cloth.data.vertices)

    scene.avbd.frame_start = 1
    scene.avbd.frame_end = 60
    result = bpy.ops.avbd.bake()
    print("cloth bake result:", result)

    scene.frame_set(60)
    bpy.context.view_layer.update()
    # World-space Z of the lowest cloth vertex must have dropped (it drooped).
    mw = cloth.matrix_world
    min_z = min((mw @ v.co).z for v in cloth.data.vertices)
    check("cloth free edge drooped", 1.0 if min_z < 2.7 else 0.0, 1.0, 0.5)
    check("cloth did not explode", 1.0 if min_z > -5.0 else 0.0, 1.0, 0.5)


def test_cloth_scene():
    """Exercises the one-click cloth-drape scene operator and bakes it."""
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene

    bpy.ops.avbd.make_cloth_scene(resolution=16)
    cloth = scene.objects.get("AVBD_Cloth")
    check("cloth scene created", 1.0 if cloth is not None else 0.0, 1.0, 0.5)
    if cloth is None:
        return

    start_min_z = min((cloth.matrix_world @ v.co).z for v in cloth.data.vertices)
    scene.avbd.frame_end = 80
    bpy.ops.avbd.bake()

    scene.frame_set(80)
    bpy.context.view_layer.update()
    end_min_z = min((cloth.matrix_world @ v.co).z for v in cloth.data.vertices)
    # The sheet starts at z=3 and should fall and drape over the obstacle.
    check("cloth scene draped", 1.0 if end_min_z < start_min_z - 0.5 else 0.0, 1.0, 0.5)


def test_brick_wall_scene():
    """Exercises the one-click test-scene operator and bakes it."""
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene

    bpy.ops.avbd.make_test_scene(width=16, height=8)
    bricks = [o for o in scene.objects
              if o.avbd_body.enabled and o.avbd_body.body_type == 'ACTIVE']
    check("test scene brick count", float(len(bricks)), 120.0, 5.0)

    scene.avbd.frame_end = 60
    scene.avbd.threads = 4
    result = bpy.ops.avbd.bake()
    print("brick wall bake result:", result)

    scene.frame_set(60)
    bpy.context.view_layer.update()
    # The wall should still be standing: the highest brick stays well above ground.
    top_z = max(o.location.z for o in bricks)
    check("brick wall top still up", 1.0 if top_z > 2.0 else 0.0, 1.0, 0.5)


if __name__ == "__main__":
    main()

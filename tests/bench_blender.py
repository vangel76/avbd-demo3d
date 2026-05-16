"""Blender-side A/B benchmark for the AVBD Physics add-on.

Run with:
    blender --background --factory-startup --python tests/bench_blender.py

Measures the three Blender-side optimisations against the code paths they
replaced, all in one process (no git checkout needed):
  * cloth read-back  : numpy + foreach_set   vs  per-vertex Python loop
  * transform apply  : one matrix_basis write vs  rotation + location writes
  * bake keyframing  : bulk foreach_set       vs  per-frame keyframe_insert
Also reports the absolute wall time of a full avbd.bake on a heavy scene.
"""

import os
import sys
import time

import bpy
from mathutils import Matrix, Quaternion, Vector

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, "addon"))


def _bench(label, fn, reps):
    # Best-of-3 to suppress scheduler noise; each timing runs `reps` iterations.
    best = min(_time(fn, reps) for _ in range(3))
    print("    {:<26} {:8.3f} ms / call  ({} reps)".format(label, best / reps, reps))
    return best / reps


def _time(fn, reps):
    t0 = time.perf_counter()
    for _ in range(reps):
        fn()
    return (time.perf_counter() - t0) * 1000.0


def bench_full_bake():
    import avbd_physics
    from avbd_physics import simulation

    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    bpy.ops.avbd.make_test_scene(width=26, height=16)
    bodies = [o for o in scene.objects
              if o.avbd_body.enabled and o.avbd_body.body_type == 'ACTIVE']
    scene.avbd.frame_start = 1
    scene.avbd.frame_end = 120
    scene.avbd.threads = 0

    t0 = time.perf_counter()
    bpy.ops.avbd.bake()
    ms = (time.perf_counter() - t0) * 1000.0
    print("\n  Full avbd.bake : {} bodies x 120 frames -> {:.0f} ms ({:.2f} ms/frame)"
          .format(len(bodies), ms, ms / 120.0))


def bench_apply():
    """A/B the per-body transform write: matrix_basis vs rotation+location."""
    from avbd_physics import simulation

    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.avbd.make_test_scene(width=26, height=16)
    solver, records = simulation.build_solver(bpy.context)
    for _ in range(20):
        solver.step()

    items = []
    for name, record in records.items():
        obj = bpy.context.scene.objects.get(name)
        if obj is None or obj.avbd_body.body_type != 'ACTIVE':
            continue
        com, quat = record.body.transform
        items.append((obj, record, com, quat))

    def new_apply():
        for obj, record, com, quat in items:
            rotation = Quaternion(quat)
            if obj.rotation_mode != 'QUATERNION':
                obj.rotation_mode = 'QUATERNION'
            location = Vector(com) - (rotation @ record.local_com)
            obj.matrix_basis = Matrix.LocRotScale(location, rotation, obj.scale)

    def old_apply():
        for obj, record, com, quat in items:
            rotation = Quaternion(quat)
            if obj.rotation_mode != 'QUATERNION':
                obj.rotation_mode = 'QUATERNION'
            obj.rotation_quaternion = rotation
            obj.location = Vector(com) - (rotation @ record.local_com)

    print("\n  Transform apply ({} bodies):".format(len(items)))
    old = _bench("old (rotation+location)", old_apply, 200)
    new = _bench("new (matrix_basis)", new_apply, 200)
    print("    -> {:.2f}x".format(old / new if new else 0.0))
    solver.destroy()


def bench_cloth_readback():
    """A/B cloth read-back: numpy + foreach_set vs per-vertex Python loop."""
    from avbd_physics import simulation

    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.avbd.make_cloth_scene(resolution=48)
    solver, records = simulation.build_solver(bpy.context)
    for _ in range(20):
        solver.step()

    cloth_items = []
    for name, record in records.items():
        if not isinstance(record, simulation.ClothRecord):
            continue
        obj = bpy.context.scene.objects.get(name)
        if obj is not None:
            cloth_items.append((obj, record))
    verts = sum(r.cloth.vertex_count for _, r in cloth_items)

    def new_read():
        simulation.read_cloths(bpy.context.scene, records)

    def old_read():
        for obj, record in cloth_items:
            positions = record.cloth.vertices()
            mesh = obj.data
            w2l = record.world_to_local
            count = min(len(positions), len(mesh.vertices))
            for i in range(count):
                mesh.vertices[i].co = w2l @ Vector(positions[i])
            mesh.update()

    print("\n  Cloth read-back ({} particles):".format(verts))
    old = _bench("old (per-vertex loop)", old_read, 60)
    new = _bench("new (numpy foreach_set)", new_read, 60)
    print("    -> {:.2f}x".format(old / new if new else 0.0))
    solver.destroy()


def bench_keyframing():
    """A/B writing a bake's keyframes: bulk foreach_set vs keyframe_insert."""
    import avbd_physics
    from avbd_physics import simulation, operators

    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.avbd.make_test_scene(width=26, height=16)
    solver, records = simulation.build_solver(bpy.context)

    # Simulate 120 frames, collecting a pose track per active body.
    tracks = {}
    for frame in range(1, 121):
        solver.step()
        for obj, loc, rot in simulation.read_active_bodies(bpy.context.scene, records):
            t = tracks.setdefault(obj, ([], [], []))
            t[0].append(frame)
            t[1].append((loc.x, loc.y, loc.z))
            t[2].append((rot.w, rot.x, rot.y, rot.z))
    solver.destroy()

    n_obj = len(tracks)
    n_frames = len(next(iter(tracks.values()))[0]) if tracks else 0

    def clear():
        for obj in tracks:
            operators._remove_baked_keyframes(obj)

    def new_keyframe():
        for obj, (frames, locs, rots) in tracks.items():
            operators._bulk_keyframe(obj, frames, locs, rots)

    def old_keyframe():
        for obj, (frames, locs, rots) in tracks.items():
            for i, frame in enumerate(frames):
                obj.location = locs[i]
                obj.rotation_quaternion = rots[i]
                obj.keyframe_insert(data_path="location", frame=frame)
                obj.keyframe_insert(data_path="rotation_quaternion", frame=frame)

    print("\n  Bake keyframing ({} bodies x {} frames):".format(n_obj, n_frames))
    # One rep each: keyframe writes are stateful; clear between.
    clear()
    old_ms = _time(old_keyframe, 1)
    clear()
    new_ms = _time(new_keyframe, 1)
    print("    {:<26} {:8.1f} ms total".format("old (keyframe_insert)", old_ms))
    print("    {:<26} {:8.1f} ms total".format("new (bulk foreach_set)", new_ms))
    print("    -> {:.2f}x".format(old_ms / new_ms if new_ms else 0.0))


def main():
    import avbd_physics
    avbd_physics.register()

    print("AVBD add-on Blender-side benchmark")
    bench_full_bake()
    bench_apply()
    bench_cloth_readback()
    bench_keyframing()
    print("\ndone")


if __name__ == "__main__":
    main()

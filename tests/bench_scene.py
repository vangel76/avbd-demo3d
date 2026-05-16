"""Benchmark the AVBD add-on on a user-supplied .blend scene.

Run with:
    blender --background --factory-startup <scene.blend> --python tests/bench_scene.py

The scene must already have its AVBD bodies configured (avbd_body.enabled etc).
Reports: scene inventory, full avbd.bake wall time, pure solver time, the
Blender-side overhead (bake minus solver), and an A/B of the read-back /
keyframing paths against the code they replaced.
"""

import os
import sys
import time

import bpy
from mathutils import Matrix, Quaternion, Vector

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, "addon"))


def _time(fn, reps):
    t0 = time.perf_counter()
    for _ in range(reps):
        fn()
    return (time.perf_counter() - t0) * 1000.0


def main():
    import avbd_physics
    from avbd_physics import simulation, operators
    avbd_physics.register()

    scene = bpy.context.scene
    settings = scene.avbd
    f0 = settings.frame_start
    f1 = max(settings.frame_end, f0)
    # Optional: "blender ... --python bench_scene.py -- <frame_count>" overrides
    # the timed frame count.
    if "--" in sys.argv:
        extra = sys.argv[sys.argv.index("--") + 1:]
        if extra and extra[0].isdigit():
            f1 = f0 + int(extra[0]) - 1
            settings.frame_end = f1
    nframes = f1 - f0 + 1
    substeps = settings.substeps

    active, passive, cloth = [], [], []
    for obj in scene.objects:
        if obj.type != 'MESH' or not obj.avbd_body.enabled:
            continue
        bt = obj.avbd_body.body_type
        if bt == 'ACTIVE':
            active.append(obj)
        elif bt == 'PASSIVE':
            passive.append(obj)
        elif bt == 'CLOTH':
            cloth.append(obj)
    cloth_verts = sum(len(o.data.vertices) for o in cloth)

    print("\n" + "=" * 66)
    print("AVBD scene benchmark: {}".format(bpy.data.filepath or "<unsaved>"))
    print("  active={}  passive={}  cloth={} ({} verts)".format(
        len(active), len(passive), len(cloth), cloth_verts))
    print("  frames {}-{} ({} frames), substeps={}, threads={}".format(
        f0, f1, nframes, substeps, settings.threads))
    if not (active or cloth):
        print("  no AVBD bodies found -- nothing to benchmark")
        return

    # --- Full bake (pristine scene) ------------------------------------------
    t0 = time.perf_counter()
    result = bpy.ops.avbd.bake()
    bake_ms = (time.perf_counter() - t0) * 1000.0
    print("\n  Full avbd.bake : {} -> {:.0f} ms total, {:.2f} ms/frame".format(
        result, bake_ms, bake_ms / nframes))

    # --- Pure solver time + collect pose tracks ------------------------------
    scene.frame_set(f0)
    bpy.context.view_layer.update()
    solver, records = simulation.build_solver(bpy.context)

    solve_ms = 0.0
    tracks = {}
    for frame in range(f0, f1 + 1):
        t0 = time.perf_counter()
        for _ in range(substeps):
            solver.step()
        solve_ms += (time.perf_counter() - t0) * 1000.0
        for obj, loc, rot in simulation.read_active_bodies(scene, records):
            t = tracks.setdefault(obj, ([], [], []))
            t[0].append(frame)
            t[1].append((loc.x, loc.y, loc.z))
            t[2].append((rot.w, rot.x, rot.y, rot.z))

    print("\n  Pure solver    : {:.0f} ms ({:.2f} ms/frame, {} steps/frame)".format(
        solve_ms, solve_ms / nframes, substeps))
    overhead = bake_ms - solve_ms
    print("  Blender side   : {:.0f} ms ({:.2f} ms/frame) -- read-back + "
          "keyframing + depsgraph".format(overhead, overhead / nframes))

    # --- A/B: bake keyframing ------------------------------------------------
    if tracks:
        n_obj = len(tracks)
        n_kf = len(next(iter(tracks.values()))[0])

        def clear():
            for obj in tracks:
                operators._remove_baked_keyframes(obj)

        def new_kf():
            for obj, (fr, locs, rots) in tracks.items():
                operators._bulk_keyframe(obj, fr, locs, rots)

        def old_kf():
            for obj, (fr, locs, rots) in tracks.items():
                for i, frame in enumerate(fr):
                    obj.location = locs[i]
                    obj.rotation_quaternion = rots[i]
                    obj.keyframe_insert(data_path="location", frame=frame)
                    obj.keyframe_insert(data_path="rotation_quaternion", frame=frame)

        clear()
        old_ms = _time(old_kf, 1)
        clear()
        new_ms = _time(new_kf, 1)
        print("\n  Keyframing ({} bodies x {} frames):".format(n_obj, n_kf))
        print("    old (keyframe_insert)  {:8.1f} ms".format(old_ms))
        print("    new (bulk foreach_set) {:8.1f} ms   -> {:.2f}x".format(
            new_ms, old_ms / new_ms if new_ms else 0.0))

    # --- A/B: cloth read-back ------------------------------------------------
    cloth_recs = [(scene.objects.get(n), r) for n, r in records.items()
                  if isinstance(r, simulation.ClothRecord)]
    cloth_recs = [(o, r) for o, r in cloth_recs if o is not None]
    if cloth_recs:
        def new_read():
            simulation.read_cloths(scene, records)

        def old_read():
            for obj, record in cloth_recs:
                positions = record.cloth.vertices()
                mesh = obj.data
                w2l = record.world_to_local
                count = min(len(positions), len(mesh.vertices))
                for i in range(count):
                    mesh.vertices[i].co = w2l @ Vector(positions[i])
                mesh.update()

        reps = 30
        old_ms = min(_time(old_read, reps) for _ in range(3)) / reps
        new_ms = min(_time(new_read, reps) for _ in range(3)) / reps
        verts = sum(r.cloth.vertex_count for _, r in cloth_recs)
        print("\n  Cloth read-back ({} particles):".format(verts))
        print("    old (per-vertex loop)   {:7.3f} ms/call".format(old_ms))
        print("    new (numpy foreach_set) {:7.3f} ms/call -> {:.2f}x".format(
            new_ms, old_ms / new_ms if new_ms else 0.0))

    solver.destroy()
    print("\ndone")


if __name__ == "__main__":
    main()

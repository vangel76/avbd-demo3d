"""Print an AVBD body inventory for a .blend scene (no bake).

Run: blender --background --factory-startup <scene.blend> --python tests/inspect_scene.py
"""

import os
import sys
from collections import Counter

import bpy

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, "addon"))


def main():
    import avbd_physics
    avbd_physics.register()

    scene = bpy.context.scene
    objs = [o for o in scene.objects if o.type == 'MESH' and o.avbd_body.enabled]

    print("\n== AVBD scene inventory ==")
    print("  total enabled bodies:", len(objs))
    shapes = Counter((o.avbd_body.body_type, o.avbd_body.collision_shape) for o in objs)
    for (bt, shape), n in sorted(shapes.items()):
        print("   {:>8} / {:<14} : {}".format(bt, shape, n))

    for bt in ('ACTIVE', 'PASSIVE'):
        verts = [len(o.data.vertices) for o in objs if o.avbd_body.body_type == bt]
        if verts:
            print("  {} mesh verts: min={} max={} mean={}".format(
                bt, min(verts), max(verts), sum(verts) // len(verts)))
        polys = [len(o.data.polygons) for o in objs if o.avbd_body.body_type == bt]
        if polys:
            print("  {} mesh faces: min={} max={} mean={}".format(
                bt, min(polys), max(polys), sum(polys) // len(polys)))

    print("  avbd settings: substeps={} iterations(default 10) threads={}".format(
        getattr(scene.avbd, "substeps", "?"), getattr(scene.avbd, "threads", "?")))


if __name__ == "__main__":
    main()

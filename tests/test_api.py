"""Smoke test for the AVBD ctypes binding (addon/avbd_physics/avbd_native.py).

Run after building the `avbd` shared library:
    python tests/test_api.py
"""

import math
import os
import sys

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, "addon", "avbd_physics"))

import avbd_native as avbd  # noqa: E402

_failures = 0


def check(name, value, expected, tol):
    global _failures
    ok = abs(value - expected) <= tol
    print("[{}] {:<36} value={:.4f} expected={:.4f} tol={:.3f}".format(
        "PASS" if ok else "FAIL", name, value, expected, tol))
    if not ok:
        _failures += 1


def test_box_stack():
    with avbd.Solver() as solver:
        solver.set_params()
        solver.add_box((100, 100, 1), 0.0, 0.5, (0, 0, 0))   # ground
        boxes = [solver.add_box((1, 1, 1), 1.0, 0.5, (0, 0, 1.0 + i * 1.5))
                 for i in range(5)]
        for _ in range(300):
            solver.step()
        bottom = boxes[0].transform[0][2]
        top = boxes[-1].transform[0][2]
        check("box stack bottom Z", bottom, 1.0, 0.15)
        check("box stack top Z", top, 5.0, 0.4)


def test_convex_body():
    # A unit cube described as a convex hull (8 verts, 6 quad faces).
    h = 0.5
    verts = [(-h, -h, -h), (h, -h, -h), (h, h, -h), (-h, h, -h),
             (-h, -h, h), (h, -h, h), (h, h, h), (-h, h, h)]
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
             (2, 3, 7, 6), (0, 4, 7, 3), (1, 2, 6, 5)]
    with avbd.Solver() as solver:
        solver.set_params()
        solver.add_box((100, 100, 1), 0.0, 0.5, (0, 0, 0))
        cube = solver.add_convex(verts, faces, 1.0, 0.5, (0, 0, 4))
        check("convex body mass", cube.mass, 1.0, 1e-3)
        for _ in range(240):
            solver.step()
        check("convex body rests on ground", cube.transform[0][2], 1.0, 0.12)


def test_joint_pendulum():
    with avbd.Solver() as solver:
        solver.set_params()
        anchor = solver.add_box((1, 1, 1), 0.0, 0.5, (0, 0, 10))
        link = solver.add_box((2, 1, 1), 1.0, 0.5, (1.5, 0, 10))
        # Joint connects the two at world point (0.5, 0, 10); the link centre of
        # mass sits 1.0 from that point (the anchor_b offset length).
        solver.add_joint(anchor, link, (0.5, 0, 0), (-1.0, 0, 0))
        for _ in range(600):
            solver.step()
        # At rest the link hangs straight down: COM directly below the joint point.
        pos = link.transform[0]
        dist = math.sqrt((pos[0] - 0.5) ** 2 + pos[1] ** 2 + (pos[2] - 10.0) ** 2)
        check("pendulum link hangs at joint radius", dist, 1.0, 0.15)
        check("pendulum link swung downward", pos[2], 9.0, 0.4)


def test_threads():
    with avbd.Solver() as solver:
        solver.set_threads(4)
        solver.set_params()
        solver.add_box((100, 100, 1), 0.0, 0.5, (0, 0, 0))
        box = solver.add_box((1, 1, 1), 1.0, 0.5, (0, 0, 6))
        for _ in range(240):
            solver.step()
        check("multi-threaded box rests", box.transform[0][2], 1.0, 0.12)


if __name__ == "__main__":
    test_box_stack()
    test_convex_body()
    test_joint_pendulum()
    test_threads()
    print()
    if _failures:
        print("TESTS FAILED ({} failure{})".format(_failures, "" if _failures == 1 else "s"))
        sys.exit(1)
    print("ALL TESTS PASSED")

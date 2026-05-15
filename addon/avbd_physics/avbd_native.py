# Copyright (c) 2026 Chris Giles
#
# Permission to use, copy, modify, distribute and sell this software
# and its documentation for any purpose is hereby granted without fee,
# provided that the above copyright notice appear in all copies.
# Chris Giles makes no representations about the suitability
# of this software for any purpose.
# It is provided "as is" without express or implied warranty.

"""ctypes binding for the AVBD physics core (`avbd` shared library).

This module loads the native ``avbd`` library and exposes a small Pythonic
wrapper around its flat C API. It has no Blender dependency, so it can also be
exercised from a plain Python interpreter for testing.

Quaternions are exchanged with callers in Blender's ``(w, x, y, z)`` order; the
engine internally uses ``(x, y, z, w)`` and this module converts transparently.
"""

import ctypes
import os
import sys

# --------------------------------------------------------------------------
# Library loading
# --------------------------------------------------------------------------


def _platform_dir():
    """Returns the lib/<platform> subdirectory name for the current OS."""
    if sys.platform.startswith("win"):
        return "windows-x64"
    if sys.platform == "darwin":
        return "macos"
    return "linux-x64"


def _library_filename():
    if sys.platform.startswith("win"):
        return "avbd.dll"
    if sys.platform == "darwin":
        return "libavbd.dylib"
    return "libavbd.so"


def _candidate_paths():
    """Yields candidate absolute paths to the native library, best first."""
    override = os.environ.get("AVBD_LIBRARY")
    if override:
        yield override

    here = os.path.dirname(os.path.abspath(__file__))
    name = _library_filename()
    # Packaged location inside the add-on.
    yield os.path.join(here, "lib", _platform_dir(), name)
    # Developer build trees (running straight from the repo).
    repo = os.path.dirname(os.path.dirname(here))
    for build in ("build/Release", "build", "build-web"):
        yield os.path.join(repo, build, name)
    # Fall back to the system loader search path.
    yield name


def _load_library():
    last_error = None
    for path in _candidate_paths():
        try:
            return ctypes.CDLL(path)
        except OSError as exc:  # not found / wrong arch
            last_error = exc
    raise OSError(
        "Could not load the AVBD native library ({}). Set the AVBD_LIBRARY "
        "environment variable to its full path. Last error: {}".format(
            _library_filename(), last_error
        )
    )


_lib = _load_library()

# --------------------------------------------------------------------------
# C API signatures
# --------------------------------------------------------------------------

_void_p = ctypes.c_void_p
_f = ctypes.c_float
_i = ctypes.c_int
_f3 = _f * 3
_f4 = _f * 4


def _sig(name, restype, argtypes):
    fn = getattr(_lib, name)
    fn.restype = restype
    fn.argtypes = argtypes
    return fn


_solver_create = _sig("avbd_solver_create", _void_p, [])
_solver_destroy = _sig("avbd_solver_destroy", None, [_void_p])
_solver_clear = _sig("avbd_solver_clear", None, [_void_p])
_solver_step = _sig("avbd_solver_step", None, [_void_p])
_solver_set_threads = _sig("avbd_solver_set_threads", None, [_void_p, _i])
# Optional (newer libraries): graceful fallback if an older avbd.dll is loaded.
try:
    _solver_set_sleeping = _sig(
        "avbd_solver_set_sleeping", None, [_void_p, _i, _f, _f, _f])
except AttributeError:
    _solver_set_sleeping = None
try:
    _body_is_asleep = _sig("avbd_body_is_asleep", _i, [_void_p])
except AttributeError:
    _body_is_asleep = None
_solver_set_params = _sig(
    "avbd_solver_set_params", None, [_void_p, _f, _f, _i, _f, _f, _f, _f]
)
_solver_get_params = _sig(
    "avbd_solver_get_params",
    None,
    [_void_p, ctypes.POINTER(_f), ctypes.POINTER(_f), ctypes.POINTER(_i),
     ctypes.POINTER(_f), ctypes.POINTER(_f), ctypes.POINTER(_f), ctypes.POINTER(_f)],
)
_add_box = _sig(
    "avbd_add_box", _void_p,
    [_void_p, ctypes.POINTER(_f), _f, _f, ctypes.POINTER(_f), ctypes.POINTER(_f)],
)
_add_convex = _sig(
    "avbd_add_convex", _void_p,
    [_void_p, ctypes.POINTER(_f), _i, ctypes.POINTER(_i), ctypes.POINTER(_i), _i,
     _f, _f, ctypes.POINTER(_f), ctypes.POINTER(_f)],
)
_body_get_transform = _sig(
    "avbd_body_get_transform", None, [_void_p, ctypes.POINTER(_f), ctypes.POINTER(_f)]
)
# Optional: present only in newer builds of the library. If an older avbd.dll
# is loaded (eg. still mapped in the host process from a previous session),
# read_transforms transparently falls back to per-body reads.
try:
    _get_transforms = _sig(
        "avbd_get_transforms", None, [ctypes.POINTER(_void_p), _i, ctypes.POINTER(_f)]
    )
except AttributeError:
    _get_transforms = None
_body_set_transform = _sig(
    "avbd_body_set_transform", None, [_void_p, ctypes.POINTER(_f), ctypes.POINTER(_f)]
)
_body_get_velocity = _sig(
    "avbd_body_get_velocity", None, [_void_p, ctypes.POINTER(_f), ctypes.POINTER(_f)]
)
_body_set_velocity = _sig(
    "avbd_body_set_velocity", None, [_void_p, ctypes.POINTER(_f), ctypes.POINTER(_f)]
)
_body_get_mass = _sig("avbd_body_get_mass", _f, [_void_p])
_add_joint = _sig(
    "avbd_add_joint", _void_p,
    [_void_p, _void_p, _void_p, ctypes.POINTER(_f), ctypes.POINTER(_f), _f, _f, _f],
)
_add_spring = _sig(
    "avbd_add_spring", _void_p,
    [_void_p, _void_p, _void_p, ctypes.POINTER(_f), ctypes.POINTER(_f), _f, _f],
)
_add_ignore = _sig("avbd_add_ignore_collision", _void_p, [_void_p, _void_p, _void_p])
_joint_is_broken = _sig("avbd_joint_is_broken", _i, [_void_p])

INFINITY = float("inf")

# --------------------------------------------------------------------------
# Pythonic wrappers
# --------------------------------------------------------------------------


class Body:
    """A rigid body handle. Created via Solver.add_box / add_convex."""

    __slots__ = ("_handle",)

    def __init__(self, handle):
        self._handle = handle

    @property
    def mass(self):
        return _body_get_mass(self._handle)

    @property
    def transform(self):
        """Returns (position, orientation) with orientation in (w, x, y, z)."""
        pos = _f3()
        quat = _f4()
        _body_get_transform(self._handle, pos, quat)
        # engine (x, y, z, w) -> Blender (w, x, y, z)
        return (pos[0], pos[1], pos[2]), (quat[3], quat[0], quat[1], quat[2])

    @transform.setter
    def transform(self, value):
        position, orientation = value
        pos = _f3(*position)
        # Blender (w, x, y, z) -> engine (x, y, z, w)
        quat = _f4(orientation[1], orientation[2], orientation[3], orientation[0])
        _body_set_transform(self._handle, pos, quat)

    @property
    def asleep(self):
        """True if the body is currently frozen by the sleeping system."""
        return bool(_body_is_asleep(self._handle)) if _body_is_asleep else False

    @property
    def velocity(self):
        """Returns (linear, angular) velocity tuples."""
        lin = _f3()
        ang = _f3()
        _body_get_velocity(self._handle, lin, ang)
        return (lin[0], lin[1], lin[2]), (ang[0], ang[1], ang[2])

    @velocity.setter
    def velocity(self, value):
        linear, angular = value
        _body_set_velocity(self._handle, _f3(*linear), _f3(*angular))


def read_transforms(bodies):
    """Batched transform read for many bodies in a single native call.

    bodies: a sequence of Body. Returns a list of (position, orientation) with
    position as (x, y, z) and orientation as (w, x, y, z).
    """
    n = len(bodies)
    if n == 0:
        return []

    # Fallback for older libraries without the batched entry point.
    if _get_transforms is None:
        return [b.transform for b in bodies]

    handles = (_void_p * n)(*[b._handle for b in bodies])
    out = (_f * (n * 7))()
    _get_transforms(handles, n, out)
    result = []
    for i in range(n):
        o = i * 7
        # engine stores (x, y, z, w); expose (w, x, y, z)
        result.append(((out[o], out[o + 1], out[o + 2]),
                       (out[o + 6], out[o + 3], out[o + 4], out[o + 5])))
    return result


class Force:
    """A constraint handle (joint, spring, or ignore-collision)."""

    __slots__ = ("_handle",)

    def __init__(self, handle):
        self._handle = handle

    @property
    def broken(self):
        """True if this is a fracturing joint that has broken. Joints only."""
        return bool(_joint_is_broken(self._handle))


class Solver:
    """An AVBD physics world."""

    def __init__(self, threads=0):
        self._handle = _solver_create()
        if not self._handle:
            raise RuntimeError("avbd_solver_create failed")
        if threads:
            self.set_threads(threads)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.destroy()

    def destroy(self):
        if self._handle:
            _solver_destroy(self._handle)
            self._handle = None

    def clear(self):
        _solver_clear(self._handle)

    def step(self):
        _solver_step(self._handle)

    def set_threads(self, n):
        _solver_set_threads(self._handle, int(n))

    def set_sleeping(self, enabled=True, lin_threshold=0.05,
                     ang_threshold=0.05, time_to_sleep=0.5):
        """Configures body sleeping. No-op on libraries that lack it."""
        if _solver_set_sleeping is not None:
            _solver_set_sleeping(self._handle, 1 if enabled else 0,
                                 lin_threshold, ang_threshold, time_to_sleep)

    def set_params(self, dt=1.0 / 60.0, gravity=-10.0, iterations=10,
                   alpha=0.99, beta_lin=10000.0, beta_ang=100.0, gamma=0.999):
        _solver_set_params(self._handle, dt, gravity, int(iterations),
                           alpha, beta_lin, beta_ang, gamma)

    def get_params(self):
        dt, gravity, alpha = _f(), _f(), _f()
        beta_lin, beta_ang, gamma = _f(), _f(), _f()
        iterations = _i()
        _solver_get_params(self._handle, dt, gravity, iterations,
                           alpha, beta_lin, beta_ang, gamma)
        return dict(dt=dt.value, gravity=gravity.value, iterations=iterations.value,
                    alpha=alpha.value, beta_lin=beta_lin.value,
                    beta_ang=beta_ang.value, gamma=gamma.value)

    def add_box(self, size, density, friction, position, velocity=(0, 0, 0)):
        handle = _add_box(self._handle, _f3(*size), density, friction,
                          _f3(*position), _f3(*velocity))
        return Body(handle)

    def add_convex(self, verts, faces, density, friction, position, velocity=(0, 0, 0)):
        """Adds a convex-hull body.

        verts: sequence of (x, y, z) vertices.
        faces: sequence of vertex-index sequences (one per polygon face).
        """
        flat_verts = []
        for v in verts:
            flat_verts.extend(v)
        face_counts = []
        face_indices = []
        for face in faces:
            face_counts.append(len(face))
            face_indices.extend(face)

        verts_arr = (_f * len(flat_verts))(*flat_verts)
        counts_arr = (_i * len(face_counts))(*face_counts)
        indices_arr = (_i * len(face_indices))(*face_indices)
        handle = _add_convex(self._handle, verts_arr, len(verts),
                             counts_arr, indices_arr, len(faces),
                             density, friction, _f3(*position), _f3(*velocity))
        if not handle:
            raise ValueError("avbd_add_convex failed (degenerate hull)")
        return Body(handle)

    def add_joint(self, body_a, body_b, anchor_a, anchor_b,
                  stiffness_lin=INFINITY, stiffness_ang=0.0, fracture=INFINITY):
        handle = _add_joint(self._handle, body_a._handle, body_b._handle,
                            _f3(*anchor_a), _f3(*anchor_b),
                            stiffness_lin, stiffness_ang, fracture)
        return Force(handle)

    def add_spring(self, body_a, body_b, anchor_a, anchor_b, stiffness, rest=-1.0):
        handle = _add_spring(self._handle, body_a._handle, body_b._handle,
                             _f3(*anchor_a), _f3(*anchor_b), stiffness, rest)
        return Force(handle)

    def add_ignore_collision(self, body_a, body_b):
        handle = _add_ignore(self._handle, body_a._handle, body_b._handle)
        return Force(handle)

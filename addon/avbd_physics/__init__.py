# Copyright (c) 2026 Chris Giles
#
# Permission to use, copy, modify, distribute and sell this software
# and its documentation for any purpose is hereby granted without fee,
# provided that the above copyright notice appear in all copies.
# Chris Giles makes no representations about the suitability
# of this software for any purpose.
# It is provided "as is" without express or implied warranty.

"""AVBD Physics - Augmented Vertex Block Descent rigid body simulation for Blender.

The heavy lifting runs in the native `avbd` shared library (see avbd_native.py);
this package gathers the scene, drives the solver, and bakes results back onto
Blender objects.
"""

bl_info = {
    "name": "AVBD Physics",
    "author": "Chris Giles; AVBD by Giles, Diaz, Yuksel (SIGGRAPH 2025)",
    "version": (0, 2, 0),
    "blender": (3, 6, 0),
    "location": "Properties > Scene & Physics, View3D > Sidebar > AVBD",
    "description": "Augmented Vertex Block Descent rigid body physics (bake & live preview)",
    "category": "Physics",
}

import bpy
from bpy.props import BoolProperty

from . import operators, properties, ui


def register():
    properties.register()
    operators.register()
    ui.register()
    # Shared flag so the UI and the live-preview modal operator can coordinate.
    bpy.types.WindowManager.avbd_live_running = BoolProperty(default=False)


def unregister():
    del bpy.types.WindowManager.avbd_live_running
    ui.unregister()
    operators.unregister()
    properties.unregister()


if __name__ == "__main__":
    register()

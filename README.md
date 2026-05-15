# avbd-demo3d

A 3D implementation of Augmented Vertex Block Descent (AVBD), plus a Blender
physics add-on built on top of it.

For details on the technique (including a pre-built web demo) see the project
page: https://graphics.cs.utah.edu/research/projects/avbd/

This repository is not intended to be a super optimized implementation, but an
easy to understand demonstration of how to implement the technique.

## What's in here

| Component | Description |
|-----------|-------------|
| `avbd_core` | Headless C++ physics core: AVBD solver, convex-hull collision, CPU multi-threading. No windowing/OpenGL dependency. |
| `avbd` | Shared library exposing a flat C API ([source/avbd_api.h](source/avbd_api.h)) for use from other languages. |
| `avbd_demo3d` | The original interactive SDL/OpenGL/ImGui demo (visual test bed). |
| `avbd_test` | Headless C++ smoke tests for the core. |
| `addon/avbd_physics` | Blender add-on driving Blender objects with AVBD. |

## Building

Make sure you have CMake (3.21+) and a C++17 compiler installed.

Checkout the code and submodules using:

```
git clone --recurse-submodules https://github.com/savant117/avbd-demo3d
```

The demo target additionally needs the SDL/ImGui submodules; the `avbd_core`,
`avbd`, and `avbd_test` targets do not.

### Native

```
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

Targets: `avbd_demo3d` (launch `Release/avbd_demo3d`), `avbd` (the shared
library), `avbd_core`, and `avbd_test` (run it to check the core).

The MSVC build statically links the C++ runtime so `avbd.dll` is self-contained
when loaded into a host process such as Blender.

### Web

Install [emscripten](https://emscripten.org/docs/getting_started/downloads.html)
and ninja (`winget install Ninja-build.Ninja` on Windows), then:

```
mkdir build-web
cd build-web
emcmake cmake ..
ninja
```

Open `avbd_demo3d.html` in your browser.

## Blender add-on

The add-on (`addon/avbd_physics`) drives Blender mesh objects with the AVBD
solver, supporting convex-hull and box colliders, joints, and springs.

### Setup

1. Build the `avbd` shared library (see above).
2. Copy the built library into the add-on, matching your platform:
   - Windows: `addon/avbd_physics/lib/windows-x64/avbd.dll`
   - Linux: `addon/avbd_physics/lib/linux-x64/libavbd.so`
   - macOS: `addon/avbd_physics/lib/macos/libavbd.dylib`
3. Package the add-on: `python tools/package_addon.py` writes
   `dist/avbd_physics-<version>.zip`.
4. In Blender (4.2+), install the zip via *Edit > Preferences > Add-ons*.

### Usage

- **Test scene**: *View3D > Sidebar > AVBD > Create Brick Wall Test Scene*
  builds a staggered brick wall (~a few hundred bodies) on solid ground for
  quick experimentation.
- **Bodies**: select a mesh, open *Properties > Physics > AVBD Rigid Body*,
  enable it, and choose Active/Passive, a collision shape, density, and friction.
- **Constraints**: select two bodies, then use *View3D > Sidebar > AVBD > Add
  Joint / Add Spring*. The created Empty marks the anchor point.
- **Solver settings**: *Properties > Scene > AVBD Physics* (gravity, timestep,
  iterations, threads, frame range).
- **Bake**: writes the simulation to keyframes over the frame range.
- **Live Preview**: steps the solver interactively in the viewport (ESC stops).

## Tests

- `build/Release/avbd_test` — C++ core tests (convex collision, threading).
- `python tests/test_api.py` — exercises the C API via the ctypes binding.
- `blender --background --python tests/test_blender.py` — end-to-end add-on test.

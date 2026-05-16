# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A CPU implementation of Augmented Vertex Block Descent (AVBD, Giles/Diaz/Yuksel
SIGGRAPH 2025) — a physics solver — plus a Blender physics add-on built on it.
The two PDFs in the repo root are the source papers. The solver supports rigid
bodies (box + convex hull), joints, springs, and triangle-FEM cloth.

## Build & test

```
cmake -S . -B build
cmake --build build --config Release
```

CMake targets:
- `avbd_core` — headless static library (the solver). No SDL/ImGui needed.
- `avbd` — shared library (`avbd.dll`) exposing the flat C API; loaded by Python.
- `avbd_test` — C++ smoke tests. Run: `./build/Release/avbd_test.exe`.
- `avbd_bench` — solver profiling benchmark: per-phase timing + thread scaling
  on rigid-box / convex-hull / cloth scenes, plus a memory-locality probe. Run:
  `./build/Release/avbd_bench.exe [wall|hull|cloth|locality|all]`.
- `avbd_demo3d` — interactive SDL/OpenGL demo. Built only when the SDL/ImGui
  submodules are present (`git submodule update --init --recursive`).

Tests (there is no per-test runner — tests are functions called from a `main`):
- C++: build & run `avbd_test`; tests are functions in `tests/test_convex.cpp`.
- Python C API: `python tests/test_api.py` (needs `avbd.dll` built).
- Blender end-to-end: `blender --background --factory-startup --python tests/test_blender.py`.
- Blender-side benchmark (A/B of the add-on read-back / keyframing paths):
  `blender --background --factory-startup --python tests/bench_blender.py`.
- Benchmark / inspect a real .blend: `tests/bench_scene.py` (bake timing +
  read-back A/B; pass `-- <frames>` to override the frame count) and
  `tests/inspect_scene.py` (AVBD body inventory). Run with
  `blender --background --factory-startup <scene.blend> --python <script>`.

The MSVC build statically links the C++ runtime (`/MT`) so `avbd.dll` is
self-contained inside Blender. **Blender keeps a loaded DLL mapped** — after
rebuilding `avbd.dll` you must fully restart Blender to pick up changes.

Running headless Blender many times in quick succession can leave numpy DLLs
locked under `…/Blender/5.x/extensions/.local`, which intermittently aborts a
test mid-run; a clean re-run passes. This is an environment artefact, not a bug.

## Add-on workflow

After a C++ change that the add-on must see:
1. Rebuild `avbd`, copy `build/Release/avbd.dll` → `addon/avbd_physics/lib/windows-x64/avbd.dll`.
2. `python tools/package_addon.py` → `dist/avbd_physics-<version>.zip`.
3. Reinstall the zip in Blender (Preferences > Add-ons > Install from Disk) and restart.

Linux/macOS native libraries must be built on those platforms and dropped into
`addon/avbd_physics/lib/linux-x64/` and `.../macos/`.

## Performance

The solver and add-on are perf-tuned. Before changing a hot path, profile with
the benchmarks above; keep optimisations behaviour-preserving (`avbd_test`
checks thread determinism — the colored Gauss-Seidel result must not depend on
thread count). See `CHANGELOG.md` for the optimisation history.

- `ThreadPool` ([parallel.cpp](source/parallel.cpp)) is a spin-hybrid pool:
  workers spin on an atomic then fall back to a CV sleep, so the ~100
  `parallelFor` dispatches per step stay cheap and an idle pool burns no CPU.
- `Solver::step()` runs force `initialize()` (collision narrow-phase) in
  parallel; only the linked-list surgery for inactive forces is serial.
- `Solver::profile` / `profileEnabled` accumulate per-phase step timings (used
  by `avbd_bench`).
- Broadphase uses a flat open-addressed grid hash, not `std::unordered_map`.
- `collideConvex` uses inline stack buffers (`kHullInlineCap`); it runs in
  parallel across every contact, so it must not heap-allocate per call.
- `collideOBB` reads each rigid body's world-space axes from a per-step cache
  (`Rigid::worldAxis`, filled in `Solver::step`) instead of recomputing them
  per contact pair.
- Add-on read-back is bulk: cloth via numpy + `foreach_set`, bakes via bulk
  F-curve `keyframe_points.foreach_set` (never `keyframe_insert` per frame).
- The per-body contact-force traversal (the primal hot path) is latency-bound
  on the contact data, not memory-layout-bound: a contiguous `Manifold` pool
  was profiled (`avbd_bench locality`) and gave 1.02x — not worth pursuing.

## Architecture

Layers, innermost first:

**`avbd_core`** (`source/*.cpp`, central header `source/solver.h`) — the headless
solver. `solver.h` must never pull in windowing/OpenGL/UI headers.

- `Body` is the base degree-of-freedom holder (linear position/velocity, mass,
  sleep state). `Rigid : Body` adds orientation/angular DOF + a collision shape
  (`size` box, or owned `ConvexHull`). `Particle : Body` is a 3-DOF point mass.
- `Force` is an energy element connecting up to `MAX_FORCE_BODIES` bodies; each
  body threads its own force list. Subclasses: `Joint`, `Spring`,
  `IgnoreCollision`, `Manifold` (rigid-rigid contact), `FEMTriangle` + `BendEdge`
  (cloth), `ParticleContact` (particle-rigid contact). A force implements
  `initialize` / `updatePrimal` / `updateDual`.
- `Solver::step()` ([solver.cpp](source/solver.cpp)): spatial-hash broadphase →
  create contact forces → initialize forces → wake sleeping bodies → warmstart →
  greedy graph-colour the dynamic bodies → for each iteration { primal update per
  colour in parallel, then parallel dual update } → BDF1 velocity → sleep pass.
  This is colored Gauss-Seidel; `ThreadPool` ([parallel.h](source/parallel.h))
  runs each colour in parallel and the result is deterministic regardless of
  thread count.
- AVBD specifics: forces use an augmented-Lagrangian penalty + dual variable;
  the primal step solves a per-body Newton system (6×6 LDLᵀ for rigids, 3×3 for
  particles); penalty stiffness ramps over iterations (paper Eq. 16); `alpha`
  controls error-correction speed; bodies sleep when at rest.

**`avbd` C API** (`source/avbd_api.{h,cpp}`) — `extern "C"` wrapper, the only
translation unit of `avbd.dll`. Opaque handles; batched transform readback.

**Blender add-on** (`addon/avbd_physics/`):
- `avbd_native.py` — ctypes binding; loads the platform library, Pythonic
  `Solver` / `Body` / `Cloth`. Quaternions cross the boundary as `(w,x,y,z)`;
  the engine uses `(x,y,z,w)`.
- `simulation.py` — bridges the Blender scene and the solver. Owns all
  coordinate-frame bookkeeping: AVBD tracks a body's centre of mass, so this
  maps object-origin ↔ COM, bakes object scale into geometry, and builds convex
  hulls / cloth meshes.
- `operators.py` (bake, live preview, test-scene builders), `properties.py`,
  `ui.py`.
- Rigid bodies bake to `location`/`rotation_quaternion` keyframes. Cloth cannot
  keyframe per vertex, so a bake fills a session vertex cache applied by a
  `@persistent` `frame_change_post` handler (`apply_cloth_cache`) — not saved
  with the .blend.
- Bake writes keyframes in bulk (`_bulk_keyframe`: create F-curves via one
  `keyframe_insert`, then `keyframe_points.foreach_set`). Blender 5.x removed
  the legacy `Action.fcurves` accessor — F-curve lookup/removal must walk
  `action.layers[].strips[].channelbags[]` (`_find_baked_fcurve`,
  `_remove_baked_keyframes`); guard the legacy path with `getattr`.
- The bake collects poses and keyframes them once at the end; it does not write
  object transforms per frame (`read_active_bodies(apply=False)`) and does not
  repaint the viewport mid-bake. It prints a per-section timing breakdown to the
  console (System Console on Windows) on completion.

**`avbd_demo3d`** (`source/main.cpp`, `scenes.h`) — SDL/OpenGL/ImGui demo and
visual test bed; iterates `Solver::bodies` as `Body*`.

## Conventions

- Keep `avbd_core` free of GL/windowing dependencies.
- Existing rigid force subclasses read `bodies[0]`/`bodies[1]` cast to `Rigid*`
  at the top of each method — follow that pattern when editing them.
- The add-on uses numpy (`simulation.py`, `operators.py`) for bulk mesh and
  keyframe I/O. numpy ships with Blender, so it is not declared as a wheel.
- Add-on version: bump the patch number each delivered iteration, the minor
  number for a large feature. Keep `__init__.py` `bl_info["version"]` and
  `blender_manifest.toml` `version` in sync; `package_addon.py` reads the former.
  Record every version bump in `CHANGELOG.md`.

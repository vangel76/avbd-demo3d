# Changelog

Notable changes to the AVBD project (the `avbd_core` solver, the `avbd.dll`
C API, and the Blender add-on). Format based on
[Keep a Changelog](https://keepachangelog.com/). The add-on version
(`__init__.py` `bl_info` / `blender_manifest.toml`) is the reference version;
the solver and C API ship inside it.

## [0.3.5] — 2026-05-17

### Added
- The bake prints a timing breakdown to the console on completion: body count,
  frame count, total time and ms/frame, plus a per-section split (build solver /
  solver step / pose readback / cloth cache / keyframing / other).

## [0.3.4] — 2026-05-17

### Changed
- `collideOBB` reads each rigid body's world-space box axes from a per-step
  cache (`Rigid::worldAxis`, filled once per `Solver::step`) instead of
  recomputing them for every contact pair. Bit-identical result; a small win on
  contact-dense scenes.

### Added
- `avbd_bench locality` — a memory-locality probe for the per-body force
  traversal.
- `tests/bench_scene.py` / `tests/inspect_scene.py` — benchmark and inventory a
  user-supplied .blend scene; `bench_scene.py` takes an optional frame count
  (`-- <n>`).

### Notes
- A contiguous memory pool for `Manifold` objects was profiled (`avbd_bench
  locality`) and rejected: the per-body force traversal is latency-bound on the
  ~11 MB of contact data, not layout-bound — contiguous vs. scattered measured
  1.02x. The solver is at its practical limit for dense rigid scenes.

## [0.3.3] — 2026-05-16

Blender-side (add-on) performance pass. No solver / `avbd.dll` change.

### Changed
- Cloth read-back (`read_cloths`): pulls the native vertex buffer and applies
  the world→local transform with numpy, then writes the mesh in one
  `foreach_set` — replaces the per-vertex Python loop. ~13x faster (measured on
  a 2400-particle cloth).
- Bake keyframing: poses are collected during the simulation and written in one
  bulk pass per F-curve (`keyframe_points.add` + `foreach_set`) instead of a
  per-frame `keyframe_insert`. ~16x faster (measured, 408 bodies × 120 frames).
- Transform write-back uses a single `matrix_basis` assignment instead of
  separate `rotation_quaternion` + `location` writes (~1.9x faster).
- The cloth bake cache stores flat float arrays; `apply_cloth_cache` applies
  them with `foreach_set`.
- The bake no longer repaints the viewport every 10 frames and no longer writes
  object transforms per frame — it collects poses and keyframes them at the end
  (`read_active_bodies(..., apply=False)`). Removes the mid-bake redraws and the
  per-frame depsgraph churn; the progress bar still updates.

### Fixed
- Blender 5.x removed the legacy `Action.fcurves` accessor. F-curve lookup and
  removal now walk the layered-action channelbags, with the legacy path
  `getattr`-guarded — bakes crashed on Blender 5.x without this.

### Added
- `tests/bench_blender.py` — A/B benchmark of the add-on read-back and
  keyframing paths.

## [0.3.2] — 2026-05-16

Solver (`avbd_core` / `avbd.dll`) performance pass. `Solver::step()` measured
~30–56% faster depending on scene; thread scaling across 16 lanes improved from
~2.3–2.9x to ~3.3–4.5x.

### Changed
- Force `initialize()` (the collision narrow-phase recompute) now runs in
  parallel across all forces; only the inactive-force linked-list surgery stays
  serial. This was the single largest serial cost in a step.
- `ThreadPool` reworked into a spin-hybrid pool: workers spin on an atomic
  generation counter and fall back to a timed condition-variable sleep only
  when idle. Removes the per-`parallelFor` kernel transitions (~100 dispatches
  per step) without burning CPU on an idle pool.
- `collideConvex` uses inline stack buffers (`HullWorld` vertices/normals, clip
  buffers) instead of per-call `std::vector` heap allocations, so parallel
  collision no longer contends on the allocator lock.
- Broadphase spatial hash is a flat open-addressed grid built by counting sort,
  replacing the node-based `std::unordered_map`.
- `Manifold` primal/dual: diagonal penalty matrices replaced with direct
  component-wise operations (bit-identical result).

### Fixed
- Broadphase `constrainedTo` was queried on the *large* body's force list — a
  ground plane accumulates thousands of contacts — for every candidate, making
  the pass O(n²) on cloth / particle scenes. It now walks the shorter list.

### Added
- `avbd_bench` benchmark target + `tests/benchmark.cpp`: rigid-box, convex-hull,
  and cloth scenes with a per-phase breakdown and thread-scaling sweep.
- `SolverProfile` / `Solver::profileEnabled` — per-phase `step()` timing.

## [0.3.1] — 2026-05-16

### Added
- Triangle-FEM cloth (`source/cloth.cpp`): `Particle` point masses,
  `FEMTriangle` membrane elements, `BendEdge` bending.
- Particle–rigid contact (`source/pcontact.cpp`).
- Cloth bodies in the add-on (vertex-cache bake) and C API.
- `CLAUDE.md` project guide.

## [0.2.0] — 2026-05-15

### Added
- Blender add-on (`addon/avbd_physics/`): ctypes binding, scene bridge,
  operators (bake / live preview / one-click test scenes), properties, UI.
- Flat C API (`source/avbd_api.{h,cpp}`) compiled to `avbd.dll`.
- Convex-hull collision (`source/convex.cpp`): SAT + face clipping narrow-phase.
- Multi-threaded solver (`source/parallel.{h,cpp}`, colored Gauss-Seidel).
- AVBD SIGGRAPH 2025 source papers; cross-platform build scripts.

## Initial release

CPU reference implementation of Augmented Vertex Block Descent (Giles, Diaz,
Yuksel — SIGGRAPH 2025): a rigid-body solver with joints and springs, plus the
interactive SDL/OpenGL demo (`avbd_demo3d`).

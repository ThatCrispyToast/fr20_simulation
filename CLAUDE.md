# CLAUDE.md

Guidance for working in this repo. See `README.md` for the user-facing overview.

## What this is

A single-file feasibility study: can an overhead-mounted Fairino **FR20** arm with a
**flat vacuum gripper** reach into a pallet bin to pick parts, and where should it be
mounted? It sweeps base poses (XYZ + yaw), tests a grid of pick targets in the bin via
seeded inverse kinematics + collision checks, and reports the top-N mount poses with
coverage heatmaps, a data bundle, and a rendered animation.

Everything lives in **`src/bin_reach.py`** (~990 lines). There is no package structure
and no test suite — verification is done by running the script (see below).

## Layout

- `src/bin_reach.py` — the entire program. All tunables are in the `CONFIG` block near
  the top; edit there, not in the functions.
- `resources/fairino20_v6.urdf` + `resources/fairino_description/` — robot model and
  meshes (extracted from `fairino20_v6_description.zip`). The URDF's `package://` paths
  resolve relative to `resources/` via `p.setAdditionalSearchPath` in `connect()`.
- `resources/bin_info.md` — the pallet dimensions the bin geometry is derived from.
- `out/run_<timestamp>/` — per-run outputs (git-ignored). Never overwritten.

## Run / verify

```bash
uv sync                              # install deps (pybullet, numpy, matplotlib, pillow)
uv run python src/bin_reach.py       # full run; writes out/run_<timestamp>/
```

Syntax check without running:
```bash
uv run python -c "import ast; ast.parse(open('src/bin_reach.py').read())"
```

Quick functional check (override CONFIG for a tiny, fast sweep — import as a module so
nothing in the file needs editing):
```bash
uv run python - <<'PY'
import numpy as np, src.bin_reach as m
m.BASE_X_RANGE = np.linspace(-0.3,0.3,3); m.BASE_Y_RANGE = np.linspace(-0.3,0.3,3)
m.BASE_Z_RANGE = np.array([1.35,1.55]); m.BASE_YAW_RANGE = np.deg2rad([0.0,45.0])
m.N_X=m.N_Y=4; m.N_Z=3; m.N_IK_SEEDS=3; m.N_WORKERS=4; m.SHOW_SIM=False
print([round(b["cov"],3) for b in m.run()])
PY
```
Module-level CONFIG names are read at call time, so overriding `m.<NAME>` before
`run()` takes effect (this is how all verification is done).

## Architecture / data flow

`run()` orchestrates everything:

1. `_pose_list()` builds every base pose in nested (yaw, z, y, x) order; `pose_idx` is
   the list position, so aggregation order — and the result — is independent of when
   parallel workers finish.
2. Sweep, one of three paths:
   - **parallel** (`N_WORKERS > 1`, default): `mp.Pool` with `_worker_init` →
     `_eval_pose`. Each worker builds its own headless world.
   - **serial** (`N_WORKERS <= 1`): same `_eval_pose` loop in-process.
   - **GUI** (`SHOW_SIM`): `_sweep_gui`, single-process, drives the live PyBullet GUI.
3. `_aggregate()` folds per-pose masks into the coverage grid, per-target frequency,
   and the ranked `bests` list (top `N_BEST`).
4. Plots (`plot_*`), then `dump_best_data()` (JSON + NPZ) and `render_animation()`
   (GIF). `run()` returns the `bests` array.

The reachability core:
- **`solve_pick()` is the single source of truth.** It returns the actual joint config
  (or `None`) after all gates: suction-face position, tool tilt, joint limits, and
  collisions (bin, self, gripper). Coverage, the JSON per-pick data, and the animation
  all call `solve_pick`, so they can never disagree.
- `reachable()` is just `solve_pick(...) is not None`.

## Non-obvious invariants — read before editing

- **IK targets the URDF link frame**, i.e. `getLinkState(...)[4]` (worldLinkFramePosition),
  *not* the COM (`[0]`). FK validation uses `[4]` to match. Don't switch indices.
- **`REACH_MAX` must stay a safe upper bound on the true reach.** Measured max
  `||flange − base||` is **2.073 m**; `REACH_MAX = 2.15`. The prune skips IK for targets
  beyond it. If you change the robot, gripper stand-off, or EE link, **re-measure** or it
  will silently prune reachable targets (a correctness bug).
- **The gripper is a Schmalz FQE/FXCB 400×280 plate** modeled as a massless **box**
  (`make_gripper`, `GEOM_BOX`) teleported onto the flange (`place_gripper`) for the
  wall-clearance check. The tool is **assumed exactly parallel to the ground**, so the
  plate is placed at the commanded *level* orientation (perfectly horizontal, clocked by
  its yaw) — not the solver's ≤`ORI_TOL` tilt residual. The
  TCP is the foam face, a `GRIPPER_STANDOFF` below the flange: IK is commanded to
  `target + [0,0,STANDOFF]` and the face position is what's validated. Footprint
  `GRIPPER_LENGTH × GRIPPER_WIDTH` is from the datasheet; **`GRIPPER_STANDOFF` is an
  estimate** (the real flange→foam height is only in the vendor STEP/2D file). Swapping
  tools is the CONFIG dims + `TOOL_YAW_DEG` only — no mesh/URDF needed.
- **A rectangular footprint makes clocking matter.** `down_orientations()` samples the
  tool yaw over `TOOL_YAW_DEG` (0°, 90°) so a near-wall pick can fit with the short side
  facing the wall; a target is reachable if it fits at any clocking. Each clocking is a
  separate IK attempt, so adding clockings multiplies sweep cost.
- **Strict tool-down by design:** `TILT_CONE_DEG = 0` (flat vacuum face ∥ ground),
  `ORI_TOL_DEG` is solver-residual slack. Don't raise the cone unless the tool can pick
  on a slope.
- **Self-collision is enabled at load** (`URDF_USE_SELF_COLLISION`); `_penetrates(rid, rid,
  exclude_adjacent=True)` ignores neighbouring links (they touch by design).
- **Parallel safety:** the parent holds **no** PyBullet connection during the `Pool`
  sweep — each worker connects its own `DIRECT` client in `_worker_init`. Forking after
  connecting is unsafe. After the sweep the parent builds one world for the dump +
  animation, guarded by `if _W is None`. Keep it that way.
- **Determinism:** `ik_seeds` uses a fixed `np.random.default_rng(0)`. Coverage is
  identical for any `N_WORKERS`, and the animation GIF is byte-reproducible. Preserve
  this when changing the sweep.
- **Coverage is a lower bound** (numerical IK has occasional false negatives) and
  **goal-pose only** (no arm-trajectory planning — only the gripper's vertical descent is
  implied). State these honestly; don't claim exact pickability.

## Performance

Full default sweep (5324 poses × 144 targets): ~**16–19 min** on 12 cores (~3 h
single-process) with the two `TOOL_YAW_DEG` clockings — ~2.1 s/pose. Scales with core
count and with the clocking count (each clocking is a separate IK attempt; one clocking
was ~0.96 s/pose ≈ 7–9 min). The dump + animation add ~15–30 s; disable via
`SAVE_ANIMATION` / lower `N_BEST` if needed.

## Conventions

- New tunables go in the `CONFIG` block with a comment explaining units and intent.
- Match the existing dense-comment style (the *why*, not the *what*).
- Don't add a real gripper mesh/URDF edit for the tool — the box envelope is
  intentional and conservative; just update the `GRIPPER_*` dims / `TOOL_YAW_DEG`.

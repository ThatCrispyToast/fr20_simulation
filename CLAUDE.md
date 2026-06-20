# CLAUDE.md

Guidance for working in this repo. See `README.md` for the user-facing overview.

## What this is

A single-file feasibility study: can an overhead-mounted Fairino **FR20** arm with a
**flat vacuum gripper** reach into a pallet bin to pick **packets**, and where should
it be mounted? It sweeps base poses (XYZ + yaw), fills the bin with a grid of packets
(rectangular boxes, default 9.5×13×1.25 in), tests each via seeded inverse kinematics +
collision checks + a foam-contact test, and reports the top-N mount poses with coverage
heatmaps, a data bundle, and a rendered animation.

Everything lives in **`src/bin_reach.py`** (~1000 lines). There is no package structure
and no test suite — verification is done by running the script (see below).

## Layout

- `src/bin_reach.py` — the entire program. All tunables are in the `CONFIG` block near
  the top; edit there, not in the functions.
- `src/serve_viz.py` + `viz/` — the optional browser viewer (see "3D viewer" below). It
  only **consumes** `best_versions.json`; it does not run the study.
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
m.N_X=m.N_Y=4; m.N_Z=3; m.N_IK_SEEDS=3; m.N_WORKERS=4
m.SHOW_SIM=False; m.SHOW_BEST_AFTER=False   # keep it non-interactive
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
   and a fully `ranked` pose list. `run()` slices `ranked[:N_BEST]` for the top list
   and `_select_diverse(ranked)` for the "best of different sections" — high-coverage
   poses greedily spread `DIVERSE_MIN_DIST` apart in the normalized (x,y,z,yaw) genome
   (quality-diversity; the top-N usually cluster, the diverse set doesn't).
4. Plots (`plot_*`), then `dump_best_data()` (JSON `top_bests`+`diverse_bests`, NPZ) and
   `render_animation()` (GIF). `run()` returns the top `bests` array. A headless run can
   still open a GUI of the best base afterward via `SHOW_BEST_AFTER` (rebuilds the world
   with `build_world(force_gui=True)` after swapping out the DIRECT one).

The reachability core:
- **Targets are PACKETS, not points.** Each grid target is the *center* of an
  axis-aligned packet box (`PACKET_L/W/H`, default 9.5×13×1.25 in) whose **top face**
  is at the target z (the suction contact plane). The old abstract point target is
  gone; `target_m` is now a packet center.
- **The XY grid insets by the packet half-extent** (not `MARGIN`) so whole packets stay
  inside the bin — the outermost packets sit **flush** against the inner walls (edge on
  the wall), as in a packed bin. `MARGIN` now only insets z (packet-top depth). Using
  the old point `MARGIN` for XY made edge packets overhang the walls (a modeling bug):
  with point targets a center 6 cm from the wall was fine, but a packet's half-extent
  is 12–17 cm, so its box poked through. Keep the XY inset = packet half-extent.
- **`solve_pick()` is the single source of truth for a *centered placement*.** It
  returns the actual joint config (or `None`) after all gates: suction-face center on
  the packet center, tool tilt, joint limits, and collisions (bin, self, gripper), plus
  `ori_idx` = which clocking won. `reachable()` is just `solve_pick(...) is not None`.
  (The packet itself is **not** a collision body — the foam is meant to land on it.)
- **Pickable ≠ centered placement.** A packet is *pickable* (the reported metric, still
  stored in the `mask`/`covered` variables) if some collision-free placement — not only
  the one centered on it — covers at least `PACKET_CONTACT_FRAC` of the **packet's top**
  with the **tool bottom** (foam face). `_eval_pose` builds the per-clocking *center* masks
  from `solve_pick`, then `_covered_mask` dilates them by the **contact offsets** from
  `_cover_offsets` → `_pickup_offsets` (per clocking, within each depth layer). The
  offsets are the (di,dj) grid steps where a packet that far from a centered tool still
  has ≥ PACKET_CONTACT_FRAC of its top under the foam (overlap ÷ **packet** area),
  computed by exact convex-polygon intersection
  (`_rect_poly` / `_convex_intersect_area`) — this **replaced** the old point-under-
  footprint `_footprint_offsets`. It returns `(pickable_mask, center_mask)`:
  **coverage/plots/`target_freq` use the pickable mask**; the **animation and JSON joint
  configs use `center_mask`** (the real, achievable placements). Each `ranked`/best dict
  carries both `mask` (pickable) and `center_mask`. Dilation keeps the early-out in
  `solve_pick` (no extra IK) and is conservative when a center is reachable at multiple
  clockings (credits only the clocking that won). If no offset can meet the contact
  fraction (e.g. a tiny packet), `_pickup_offsets` is empty and the packet is unpickable.

## Non-obvious invariants — read before editing

- **IK targets the URDF link frame**, i.e. `getLinkState(...)[4]` (worldLinkFramePosition),
  *not* the COM (`[0]`). FK validation uses `[4]` to match. Don't switch indices.
- **`REACH_MAX` must stay a safe upper bound on the true reach.** Measured max
  `||flange − base||` is **2.073 m**; `REACH_MAX = 2.15`. The prune skips IK for targets
  beyond it. If you change the robot, gripper stand-off, or EE link, **re-measure** or it
  will silently prune reachable targets (a correctness bug).
- **The gripper is a Schmalz FQE/FXCB 400×280 plate** modeled as a massless **compound
  body** (`make_gripper`, `createCollisionShapeArray`): a thin `GRIPPER_POST_RADIUS`
  cylinder over the top half of the stand-off and the full 400×280 box over the bottom
  half (down to the foam face). The body origin is the flange and the two shapes carry
  their own down-axis offsets, so `place_gripper` just sets the body to the flange pose.
  Splitting is a fidelity/visual win — with vertical walls and a vertical tool the
  constant XY footprint means it does **not** change coverage vs a solid box. The tool
  is **assumed exactly parallel to the ground**, so the
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
- **`USE_GRIPPER = False` toggles the whole tool off** (bare flange): stand-off → 0, no
  gripper body (`gripper_id is None`), the footprint collision is skipped, and clockings
  collapse to one. It's threaded via a local `standoff = GRIPPER_STANDOFF if USE_GRIPPER
  else 0.0` in `solve_pick`/`pose_at` plus `is not None` guards — keep both in sync if you
  touch the stand-off or gripper-collision logic.
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

## 3D viewer (`viz/` + `src/serve_viz.py`)

A standalone browser viewer for a finished run — independent of the study, reads only
`best_versions.json`. `src/serve_viz.py` finds the newest `out/run_*` (or takes a run
arg), serves the **repo root** over HTTP (so the page can `fetch` the JSON *and* the
URDF/meshes — `file://` can't), and opens `viz/index.html?run=<rel-json>`.

- `viz/index.html` — layout, control panel, and the importmap pinning three.js +
  `urdf-loader` to a CDN (so the first load needs internet; run data/meshes are local).
- `viz/app.js` — the scene. **Kept in the sim's native Z-up frame** (no axis
  conversion) so poses match exactly: base orientation is `Rz(yaw)*Rx(pi)` (the same
  hang-down flip as `base_orientation()`), and the gripper plate reuses that quaternion
  clocked by `tool_yaw_deg`. The arm is the real FR20 via `urdf-loader` (joints `j1..j6`,
  flange = `wrist3_link`); playback drives `joints_rad` from the JSON `picks`, and the
  gripper follows the FK flange. Packets are drawn as boxes coloured by `is_placement`
  / `pickable` (the viewer reads `CFG.packet` dims; falls back to `covered` for old JSON).
- Faithful plate clocking needs the `tool_yaw_deg` field that `_pick_records` now writes
  per placement; the viewer falls back to 0 for older JSON without it.
- The viewer is **not** part of `run()` and writes nothing — keep the study self-contained.

## Performance

Full default sweep (5324 poses × 600 targets): ~**65–80 min** on 12 cores (~13 h
single-process) with the two `TOOL_YAW_DEG` clockings — ~8.8 s/pose. Scales linearly
with the target count (`N_X·N_Y·N_Z`, raised from 144 to 600), the clocking count (each
is a separate IK attempt; `USE_GRIPPER = False` collapses to one), and `N_IK_SEEDS`. The
dump re-solves every target for each of the `N_BEST` + `N_DIVERSE` poses, and the
animation renders one frame per reachable pick — together ~15–40 s; trim via
`SAVE_ANIMATION` / `N_BEST` / `N_DIVERSE` if needed.

## Conventions

- New tunables go in the `CONFIG` block with a comment explaining units and intent.
- Match the existing dense-comment style (the *why*, not the *what*).
- Don't add a real gripper mesh/URDF edit for the tool — the post+plate envelope is
  intentional and conservative; just update the `GRIPPER_*` dims / `TOOL_YAW_DEG`.

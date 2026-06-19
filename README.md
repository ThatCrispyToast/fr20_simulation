# FR20 bin-reachability feasibility study

Sweeps overhead mount positions for a Fairino **FR20** arm hanging over a pallet
bin and reports how many **packets** it can pick with a **flat vacuum gripper**
(suction face parallel to the ground, picking straight down). Headless (PyBullet
`DIRECT`), parallelized across CPU cores; writes five diagnostic diagrams and prints
a terminal progress bar.

The bin is filled with a 3D grid of **packets** — real rectangular boxes (default
**9.5 × 13 × 1.25 in**, sizes are parameters) resting with their top face at each
sampled depth. A **placement** is valid when some IK solution centers the foam face
on a packet with the tool pointing straight down, in joint limits, and collision-free
— checked against the bin walls, the arm against **itself**, and the **gripper body**
against the walls (the 400×280 plate is clocked to whichever rotation fits best). A
packet is then counted as **pickable** if some valid placement — not only the one
centered on it — covers at least `PACKET_CONTACT_FRAC` (default **60%**) of the
**packet's top** with the **tool bottom** (the foam face), i.e. enough of the packet
sits under the suction face to lift it. So the area gripper picks a packet near a wall
by sitting the plate inward, as long as enough of the packet stays under the foam.
Pickability is the reachable-center grid dilated by the set of contact-satisfying
packet offsets, and is the reported metric.

## Setup

The robot meshes live under `resources/fairino_description/` (extracted from
`resources/fairino20_v6_description.zip`). The URDF's `package://` paths resolve
relative to that directory automatically.

```bash
uv sync
```

## Run the full simulation

```bash
uv run python src/bin_reach.py
```

The default sweep is wide on all axes — `11 (x) × 11 (y) × 11 (z) × 4 (yaw) =
5324` mount poses, each tested against `10 × 10 × 6 = 600` pick targets. The yaw
axis rotates the base about the vertical: the arm's reachable envelope is not
axisymmetric over a rectangular bin (joint 1 has a ~±175° dead wedge and the
shoulder/elbow offset clears the walls differently per heading), so heading
changes coverage. The XY/height heatmaps show the **best coverage over all yaws**
at each cell, and the reported best base includes its winning yaw. Every target
is checked with multiple IK seeds (see below).

The sweep runs in parallel across `N_WORKERS` processes (default = all cores), each
its own headless PyBullet world; results are identical regardless of worker count.
On a 12-core machine the full default sweep takes **~65–80 minutes** with the
`10 × 10 × 6 = 600` target grid; it scales linearly with the target count
(`N_X·N_Y·N_Z`), the number of `TOOL_YAW_DEG` clockings, and `N_IK_SEEDS`, and inversely
with core count. The progress bar shows elapsed time and ETA. For quicker iteration,
shrink the `BASE_*_RANGE` arrays (set `BASE_YAW_RANGE = [0.0]` to disable the yaw
search), lower `N_X/N_Y/N_Z`, drop a `TOOL_YAW_DEG` clocking, or lower `N_IK_SEEDS`.

### The vacuum gripper

The end effector is a **Schmalz FQE/FXCB 400×280 vacuum gripper for cobots** (ISO
9409-1 flange, mounts directly on the FR20). Its foam face stays parallel to the
ground, so the tool is **strictly down** (`TILT_CONE_DEG = 0`; `ORI_TOL_DEG` is slack
for the IK orientation residual). It's modeled as a conservative **two-part envelope**:
a thin `GRIPPER_POST_RADIUS` cylinder over the top half of the stand-off (the slim
mounting body just below the flange) and the full `GRIPPER_LENGTH × GRIPPER_WIDTH`
plate (400 × 280 mm) over the bottom half, down to the foam face. Because the plate is
large and rectangular, a
pick near a wall only fits at some **clockings** — the tool is rotated about the
vertical through `TOOL_YAW_DEG` (0° and 90°) and a target counts if it fits at any of
them. You don't need the manufacturer's CAD/URDF files; just these CONFIG numbers.

> The 400 × 280 footprint is from the Schmalz datasheet. The **stand-off** (flange →
> foam face, `GRIPPER_STANDOFF = 0.12 m`) is an **estimate** — the exact height is only
> in the downloadable STEP/2D drawing behind the retailer; replace it with the real
> value (a one-line edit) when you have it.

Set **`USE_GRIPPER = False`** to remove the tool entirely (bare flange: no stand-off,
no footprint collision, no clocking search) — a useful baseline for "what could the
arm reach with no gripper?" and for isolating how much the 400 × 280 plate costs.

### Top-N and the "best of different sections"

The top-N poses (`N_BEST`) tend to **cluster** around one spot — small perturbations of
the same mount. So the run also reports the **best of different sections** (`N_DIVERSE`):
high-coverage mounts that are far apart in the base-pose space, picked greedily so each
is at least `DIVERSE_MIN_DIST` (normalized x/y/z/yaw distance) from the others. These
are genuinely different mounts that reach the bin with similar efficacy — the
quality-diversity view, like different mutations reaching similar fitness with very
different weights. Both sets are printed and saved (with full per-pick data) to
`best_versions.json` / `.npz`.

### Why multiple IK seeds

A 6-DOF arm can usually reach the same tool pose in several joint configurations
(elbow up/down, wrist flips). `calculateInverseKinematics` returns just one, so a
target that *is* reachable can look blocked if that single config happens to
collide or exceed a joint limit. `N_IK_SEEDS` controls how many seeded IK
attempts are made per pose; a target counts as reachable if **any** seed yields a
collision-free, in-limits configuration that lands on it. More seeds = fewer
false negatives, proportionally slower — coverage is a lower bound that rises (a
little) with more seeds.

### Outputs

`run()` returns the **top-N base poses** as an array (`N_BEST`, best first), and
every run writes a timestamped folder `out/run_<timestamp>/` containing:

- `coverage_vs_base.png` — coverage % (pickable packets) across the base XY grid, one panel per mount height, with per-cell values, the bin footprint, and the best base starred
- `best_pos_slices.png` — pickable packets at the best base, sliced by depth, with the bin outline and per-slice counts
- `reach_3d.png` — 3D scatter of pickable vs not-pickable packets at the best base, with the bin wireframe and the robot base
- `coverage_vs_height.png` — best-base and mean coverage as a function of mount height
- `target_reachability.png` — for every packet position, the % of all swept base positions that can pick it (highlights intrinsically hard bin regions), sliced by depth
- `best_pick_cycle.gif` — **reproducible** end animation of the best base driving the arm through its real plate **placements**, packets drawn as boxes (markers: green = pickable, red = not), rendered offline so it's produced after every run (no GUI) and is byte-identical each time
- `best_versions.json` — config snapshot (incl. the `packet` block) plus, for each pose in **`top_bests`** and **`diverse_bests`**: coverage, per-depth counts, and **every packet's** `pickable`/`is_placement` flags with the joint solution (rad + deg), FK error, tool tilt, and the winning footprint clocking (`tool_yaw_deg`) at each placement
- `best_versions.npz` — raw arrays: coverage grid, target frequency, packet-center coords, and per top/diverse pose the pickable mask, the centered-placement mask, and joint configs (NaN where not a placement)

## Interactive 3D viewer (three.js)

A browser viewer renders a run's `best_versions.json` in 3D: the bin, the
**articulated FR20** (loaded from the URDF + STL meshes), the vacuum-gripper plate,
and the **packets** drawn as boxes coloured by outcome (green = a centered
**placement**, amber = pickable off-centre, red = not pickable). Pick any reported
base pose (top-N or diverse), slice the packet grid by depth, toggle layers, and
**play the arm through its real placements** — driven by the joint solutions in the
JSON, so the poses match the study exactly.

```bash
uv run python src/serve_viz.py                 # newest out/run_* folder
uv run python src/serve_viz.py out/run_xxxx     # a specific run
uv run python src/serve_viz.py --port 8123 --no-browser
```

The launcher serves the repo over HTTP (browsers can't `fetch` the JSON/meshes
from a `file://` page) and opens the viewer pointed at the run. The scene is kept
in the simulator's native **Z-up** frame, so no axis conversion is needed. three.js
and `urdf-loader` load from a CDN, so the first load needs internet access; the run
data and meshes are served locally. Files live in [`viz/`](viz/).

## Watch the simulation

Set `SHOW_SIM = True` in the `CONFIG` block to open a PyBullet GUI. The arm
visibly scans each mount position during the sweep (settling on a reachable pick
at every base), then parks at the best base, dots every target green (reachable)
or red (not), and cycles through the reachable picks. Close the window or press
`Ctrl-C` to quit.

A live HUD floats above the bin showing the current base position, coverage here
vs. best-so-far, progress, and elapsed/ETA during the sweep — then the best base
and current pick during the final cycle.

Watching the full wide-grid sweep takes a while — for a quick look, narrow the
`BASE_*_RANGE` arrays (or `SIM_DWELL` controls how long the GUI lingers on each
pose).

### Inspect the result after a headless run

To get the fast parallel sweep **and** an interactive look at the winner, leave
`SHOW_SIM = False` but keep `SHOW_BEST_AFTER = True` (the default): the run sweeps headless,
writes all the artifacts, then opens a GUI parked at the best base and
cycles its reachable picks (same view as the final stage of a `SHOW_SIM` run). It
blocks until you close the window or press `Ctrl-C`. Set `SHOW_BEST_AFTER = False`
for fully non-interactive/batch runs. (On a machine with no display the viewer is
skipped with a message rather than failing.)

## Tuning

Everything is in the `CONFIG` block of [src/bin_reach.py](src/bin_reach.py):
bin dimensions, `MOUNT_HEIGHT`, the `BASE_*_RANGE` sweeps, target grid resolution, the
**packet** (`PACKET_L` / `PACKET_W` / `PACKET_H` and the `PACKET_CONTACT_FRAC` grip
threshold), the gripper (`USE_GRIPPER` on/off; `GRIPPER_LENGTH` / `GRIPPER_WIDTH` /
`GRIPPER_STANDOFF` and `TOOL_YAW_DEG`), `N_WORKERS`, `N_BEST` and `N_DIVERSE` /
`DIVERSE_MIN_DIST` (how many top + diverse poses to report and dump), the GUI options
(`SHOW_SIM`,
`SHOW_BEST_AFTER`), the animation settings (`SAVE_ANIMATION`, `ANIM_W/H`, `ANIM_FPS`),
and tolerances. Narrow the `BASE_*_RANGE` arrays or drop their point counts (or the
`TOOL_YAW_DEG` clockings) for a faster run; raise `N_WORKERS` to use more cores
(default = all of them).

### Known limitations

- **Goal-pose only.** Each pick is checked as a static configuration. The gripper's
  own vertical descent is accounted for (vertical walls + a horizontal plate at fixed
  XY), but a full collision-free *arm trajectory* into the bin is not planned —
  coverage is an upper bound on what a real motion planner would achieve.
- **Packets are tested in isolation.** Each packet is evaluated alone in an otherwise
  empty bin (like the old point targets), so neighbouring packets are not modelled as
  obstacles, and the grip test is a geometric foam-overlap fraction, not a force/seal
  simulation. Packets are axis-aligned with the bin at a single nominal size; vary the
  `PACKET_*` dims (per packet, if you extend it) for a size mix.
- **Coverage is a lower bound.** Numerical IK with finite seeds yields occasional
  false negatives; the reported percentages rise slightly with `N_IK_SEEDS`.
- **Estimated stand-off.** The 400 × 280 footprint is from the datasheet, but the
  flange→foam height is estimated (`GRIPPER_STANDOFF = 0.12 m`); it shifts depth
  reach by a few cm. Footprint, not stand-off, dominates the near-wall results.

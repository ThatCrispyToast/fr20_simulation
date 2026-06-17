# FR20 bin-reachability feasibility study

Sweeps overhead mount positions for a Fairino **FR20** arm hanging over a pallet
bin and reports how much of the bin interior it can reach. Headless (PyBullet
`DIRECT`); writes two heatmaps and prints a terminal progress bar.

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

The default sweep is wide on all axes — `11 (x) × 11 (y) × 11 (z) = 1331` mount
positions, each tested against `6 × 6 × 4 = 144` pick targets. Every target is
checked with multiple IK seeds (see below), so a full run takes on the order of
**an hour**; the progress bar shows elapsed time and ETA. For quicker iteration,
shrink the `BASE_*_RANGE` arrays or lower `N_IK_SEEDS`.

### Why multiple IK seeds

A 6-DOF arm can usually reach the same tool pose in several joint configurations
(elbow up/down, wrist flips). `calculateInverseKinematics` returns just one, so a
target that *is* reachable can look blocked if that single config happens to
collide or exceed a joint limit. `N_IK_SEEDS` controls how many seeded IK
attempts are made per pose; a target counts as reachable if **any** seed yields a
collision-free, in-limits configuration that lands on it. More seeds = fewer
false negatives, proportionally slower (e.g. on a central base, 1 seed found
56/144 targets vs 73/144 with 8 seeds).

Outputs (written to `out/`):

- `coverage_vs_base.png` — coverage % across the base XY grid, one panel per mount height, with per-cell values, the bin footprint, and the best base starred
- `best_pos_slices.png` — reachable pick points at the best base, sliced by depth, with the bin outline and per-slice counts
- `reach_3d.png` — 3D scatter of reachable vs unreachable pick points at the best base, with the bin wireframe and the robot base
- `coverage_vs_height.png` — best-base and mean coverage as a function of mount height
- `target_reachability.png` — for every pick point, the % of all swept base positions that can reach it (highlights intrinsically hard bin regions), sliced by depth

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

## Tuning

Everything is in the `CONFIG` block of [src/bin_reach.py](src/bin_reach.py):
bin dimensions, `MOUNT_HEIGHT`, the `BASE_*_RANGE` sweeps, target grid
resolution, and tolerances. Narrow the `BASE_*_RANGE` arrays or drop their point
counts for a faster run.

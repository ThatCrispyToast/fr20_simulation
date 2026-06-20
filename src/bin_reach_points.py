"""
Overhead-arm bin reachability feasibility study (PyBullet) -- POINT-TARGET variant.

This is the original *point* model: each grid target is an abstract point, a pick is
valid when the foam-face center lands on it, and a point is "covered" if it falls under
the foam footprint of any reachable placement. The packet model -- where each target is
a real box and a packet is "pickable" when >= a fraction of it sits under the foam --
lives in the sibling `bin_reach.py`. The two are independent, self-contained scripts;
they share only the read-only 3D viewer (`serve_viz.py` + `viz/`), which auto-detects
the mode from the run's JSON (no `config.packet` block => point mode, drawn as spheres).

What it does
------------
- Builds a deep bin (floor + 4 walls) from box collision shapes.
- Mounts a robot arm overhead, hanging down over the bin.
- Models the end effector as a Schmalz FQE/FXCB 400x280 flat vacuum gripper (face
  parallel to the ground, picks straight down): a stand-off from the flange to the
  foam face plus a large rectangular footprint that has to clear the walls. The
  plate is clocked about the vertical (TOOL_YAW_DEG) so it can be rotated to fit.
- Sweeps a grid of base XYZ positions and headings (yaw).
- For each base pose, samples a 3D grid of points inside the bin. A point is a valid
  PLACEMENT (the plate centered on it) if some IK solution (over several seeds) meets
  ALL of:
      (1) the SUCTION FACE lands on the point (FK error < FK_TOL),
      (2) the tool stays parallel to the ground (tilt < TILT_CONE_DEG + ORI_TOL_DEG),
      (3) the solution is within joint limits,
      (4) no robot link penetrates the bin walls/floor,
      (5) the arm does not collide with itself,
      (6) the gripper body does not penetrate the walls (corner/edge clearance; with
          vertical walls this also implies clearance for the straight-down descent).
  A point is COVERED (the reported metric) if it falls under the foam footprint of ANY
  valid placement -- not only the placement centered on it. So an area vacuum gripper
  can pick a point near a wall by sitting the plate inward, with the point under the
  plate's edge. Coverage = the placement grid DILATED by the plate footprint (per
  clocking, within each depth layer); with no tool it collapses to the placements.
- Reports the top-N base poses by coverage (run() returns them as an array) AND the
  "best of different sections": high-coverage mounts that are far apart in base-pose
  space (a quality-diversity view -- different mounts that work about as well, like
  different mutations reaching similar fitness). After every run it writes to
  out/run_<timestamp>/: five diagnostic diagrams, a reproducible end animation of the
  best base cycling its reachable picks (best_pick_cycle.gif, rendered offline -- no
  GUI needed), and a detailed data bundle on both pose sets (best_versions.json +
  best_versions.npz: per-pick joint solutions, FK error, tool tilt, per-depth counts,
  raw arrays). Set USE_GRIPPER=False to model a bare flange (no tool) as a baseline.

Performance: the sweep is embarrassingly parallel across base poses, so it runs on
N_WORKERS processes (each its own headless PyBullet world). Results are identical to a
single-process run. Set SHOW_SIM=True to watch it (forces a single process).

Swap in your own robot / gripper
--------------------------------
Set ROBOT_URDF to your arm's URDF and adjust EE_LINK (or leave None to auto-pick the
last link). Edit the gripper dimensions (GRIPPER_LENGTH / WIDTH / STANDOFF, and
TOOL_YAW_DEG) to match your tool, or set USE_GRIPPER=False for a bare flange. Set the
bin dimensions and the base sweep to match your cell. Everything is in the CONFIG block.
"""

import os
import json
import math
import time
import multiprocessing as mp
import numpy as np
import pybullet as p
import pybullet_data
from PIL import Image
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers 3d projection)

# ----------------------------------------------------------------------------
# CONFIG
# ----------------------------------------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
# Fairino FR20 (fairino20_v6). Mesh STLs are resolved via the URDF's
# package://fairino_description/... paths, relative to the URDF directory:
#   resources/fairino_description/meshes/fairino20_v6/*.STL
ROBOT_URDF = os.path.join(_HERE, "..", "resources", "fairino20_v6.urdf")
EE_LINK    = None        # link index of the tool tip; None = auto (last link)
FIXED_BASE = True

SHOW_SIM  = False         # True = open a PyBullet GUI and watch the arm work
SIM_DWELL = 0.05         # seconds the GUI lingers on each pose so motion is visible
# After a headless run (SHOW_SIM = False), still pop open a GUI at the end to inspect
# the best base (the arm cycling its reachable picks, same view as the live run).
# Blocks until you close the window / press Ctrl-C. No effect when SHOW_SIM is True.
SHOW_BEST_AFTER = True
OUT_DIR   = os.path.join(_HERE, "..", "out")  # base output dir; each run writes
RUN_DIR   = OUT_DIR                            # to a timestamped subfolder (set in run())

# Bin geometry (meters). Interior floor sits at z=0, opening at z=BIN_DEPTH.
# Pallet from resources/bin_info.md: 1092.2 x 1219.2 x 838.2 mm.
BIN_L, BIN_W, BIN_DEPTH = 1.0922, 1.2192, 0.8382   # length(x), width(y), depth(z)
WALL_T = 0.02                                       # wall thickness
BIN_CENTER_XY = (0.0, 0.0)                          # bin center in world XY

# Overhead mount: base hangs upside-down at this height above the bin opening.
# FR20 reach ~1.7 m; base must clear the 0.838 m opening yet still touch the floor.
MOUNT_HEIGHT = 1.35      # world z of the robot base
FLIP_BASE = True         # rotate base 180deg about X so the arm hangs down

# ----------------------------------------------------------------------------
# Vacuum gripper -- SWAP YOUR TOOL HERE
# ----------------------------------------------------------------------------
# Schmalz FQE/FXCB area vacuum gripper for cobots, 400 x 280 (ISO 9409-1 flange,
# fits the FR20 directly). Flat foam suction face stays parallel to the ground and
# picks straight down. The tool adds a STAND-OFF (the flange sits this far above the
# foam face, so the arm reaches deeper) and a large rectangular FOOTPRINT that has
# to clear the bin walls -- a 400x280 plate cannot pick close to a wall, so its
# clocking about the vertical matters (see TOOL_YAW_DEG). Modeled as a thin cylinder
# (top half of the stand-off, the slim mounting body) over the full 400x280 plate
# (bottom half, down to the foam face) -- see GRIPPER_POST_RADIUS. Edit these to swap
# tools; everything downstream follows.
#   Footprint 400 x 280 mm is from the Schmalz datasheet (confirmed).
#   STAND-OFF is an ESTIMATE: the flange->foam height is only in the STEP/2D
#   drawing behind the retailer (Devonics/Inlux); replace 0.12 with the real value.
# Master toggle: False removes the tool entirely (bare flange) -- no stand-off, no
# footprint collision, no clocking search. Useful as a baseline ("what could the arm
# reach with no gripper?") to isolate how much the 400x280 plate costs.
USE_GRIPPER       = True

GRIPPER_LENGTH    = 0.400   # m, footprint long side  (tool-frame x)
GRIPPER_WIDTH     = 0.280   # m, footprint short side (tool-frame y)
GRIPPER_STANDOFF  = 0.12    # m, flange face -> foam suction face (ESTIMATE; see above)
# The tool is two stacked halves down the stand-off: a thin (negligible) cylinder for
# the TOP half (the slim mounting body/connection just below the flange) and the full
# 400x280 plate for the BOTTOM half (down to the foam face). Less bulky / more faithful
# than one solid box; the wide footprint that actually has to clear walls sits low.
GRIPPER_POST_RADIUS = 0.02  # m, radius of the slim top-half cylinder (negligible)

# Clockings (deg, about the vertical) of the rectangular footprint to try. A pick
# counts if the plate fits at ANY clocking, so the arm is free to rotate the tool
# to fit. 0 puts the long side along world x, 90 along world y; the rectangle is
# 180-symmetric so 0..90 covers the distinct fits in an axis-aligned bin.
TOOL_YAW_DEG      = (0.0, 90.0)

# Base position sweep (world XYZ of the mount point). Wide search on all axes.
BASE_X_RANGE = np.linspace(-0.70, 0.70, 11)
BASE_Y_RANGE = np.linspace(-0.70, 0.70, 11)
BASE_Z_RANGE = MOUNT_HEIGHT + np.linspace(-0.40, 0.40, 11)  # mount heights to try

# Base yaw sweep (rotation about the vertical axis, radians). The arm's reachable
# envelope is NOT axisymmetric over a rectangular bin: joint 1 is limited to
# ~+/-175deg (a dead wedge behind the arm) and the shoulder/elbow offset clears
# the four walls differently per heading, so heading changes which targets are
# reachable. The bin is ~symmetric under 180deg, so 0..90deg covers the distinct
# headings. Set to [0.0] to disable the yaw search.
BASE_YAW_RANGE = np.deg2rad(np.linspace(0.0, 90.0, 4))   # [0, 30, 60, 90] deg

# Target sampling inside the bin. Finer = sharper reachability map but the sweep cost
# scales linearly with the point count (N_X*N_Y*N_Z). 10x10x6 = 600 points (~0.11 m
# spacing); was 6x6x4 = 144. Runtime grows ~proportionally (so ~4x vs the old grid).
N_X, N_Y, N_Z = 10, 10, 6        # grid resolution of pick targets (600 points)
MARGIN = 0.06                    # keep targets this far inside the walls/floor

# Tolerances
FK_TOL        = 0.02     # m, how close the suction face must actually get to the target
COLLIDE_TOL   = 0.002    # m, penetration depth that counts as a collision
# A flat vacuum face must stay parallel to the ground, so the tool is strictly
# down (TILT_CONE_DEG = 0 -> only the straight-down approach is tried). ORI_TOL_DEG
# is slack on top of that to absorb the IK solver's small orientation residual; a
# solution whose tool axis tilts more than TILT_CONE_DEG + ORI_TOL_DEG off vertical
# is rejected (the suction seal would break). Raise TILT_CONE_DEG only for a tool
# that can pick on a slope.
TILT_CONE_DEG = 0        # 0 = strictly tool-down; >0 lets it try angled approaches
ORI_TOL_DEG   = 3.0      # extra tool-axis slack (deg) for IK orientation residual

# Skip IK entirely for targets whose flange goal is farther from the base than the
# arm can possibly reach. Must be a SAFE UPPER BOUND on the true reach or it would
# prune reachable targets: the measured max ||flange - base|| is 2.073 m, so 2.15 m
# keeps all reachable goals while still skipping far corners.
REACH_MAX     = 2.15     # m, base->flange distance above which a target is unreachable

# Parallelism: the sweep is embarrassingly parallel across base poses. Each worker
# runs its own headless PyBullet world. Set to 1 (or run with SHOW_SIM) to stay
# single-process. Results are identical regardless of worker count.
N_WORKERS     = os.cpu_count() or 1
if N_WORKERS == 1: print("!! RUNNING SINGLE THREADED !!")

# Reporting: keep the top-N base poses (not just the single best) so near-ties and
# alternative mounts are visible. The first entry is THE best (drives the plots).
N_BEST        = 5        # how many top base poses to report and dump data for

# Also report the "best of different sections": high-coverage mounts that are far
# apart in the base-pose space. The top-N usually cluster around one spot; these are
# the genuinely different mounts that work about as well -- the quality-diversity view
# (different 'genomes' reaching similar fitness, like different mutations in evolution).
N_DIVERSE        = 5     # how many diverse alternatives to report (0 to disable)
DIVERSE_MIN_DIST = 0.30  # min separation in normalized (x,y,z,yaw) space, 0..~2

# After every run, render a reproducible animation of the best base cycling through
# its reachable picks (offline software renderer -> GIF, no GUI needed) and dump a
# JSON/NPZ bundle of detailed per-pick data for the top-N bases.
SAVE_ANIMATION = True
ANIM_W, ANIM_H = 480, 360   # animation frame size (px)
ANIM_FPS       = 8          # frames per second in the saved GIF

# IK is a numerical solver that returns ONE config per call; a 6-DOF arm usually
# has several (elbow up/down, wrist flips). Try multiple seeds per pose and accept
# the target if ANY collision-free, in-limits config reaches it. More seeds = fewer
# false negatives, but proportionally slower.
N_IK_SEEDS = 8           # random seed configs tried per pose (plus the mid-range one)

# Per-point IK-solution dump (for the viewer's click-to-inspect / failure paths).
# enumerate_solutions() re-solves each point of the REPORTED poses with this many seeds
# and keeps every DISTINCT joint config (valid + failed, with the failure reason) so the
# viewer can show "every way the arm can reach this TCP" and how a failed point fails.
DUMP_SOLUTIONS    = True
N_SOLUTION_SEEDS  = 24   # seeds used only by enumerate_solutions (dump phase only)
MAX_SOLUTIONS     = 16   # cap on distinct configs stored per point

# ----------------------------------------------------------------------------
# Setup
# ----------------------------------------------------------------------------
def connect(force_gui=False):
    gui = SHOW_SIM or force_gui
    cid = p.connect(p.GUI if gui else p.DIRECT)
    # Resolve the robot's package:// mesh paths relative to the URDF directory.
    p.setAdditionalSearchPath(os.path.dirname(os.path.abspath(ROBOT_URDF)))
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)          # hide side panels
        p.resetDebugVisualizerCamera(
            cameraDistance=2.6, cameraYaw=50, cameraPitch=-30,
            cameraTargetPosition=[BIN_CENTER_XY[0], BIN_CENTER_XY[1],
                                  BIN_DEPTH / 2])
    return cid

def _fmt_mmss(seconds):
    m, s = divmod(int(seconds), 60)
    return f"{m:d}m {s:02d}s"

def progress_bar(done, total, t0, width=34):
    """Minimal in-place terminal progress bar (no dependencies)."""
    frac = done / total
    filled = int(width * frac)
    bar = "█" * filled + "·" * (width - filled)
    elapsed = time.time() - t0
    eta = elapsed / done * (total - done) if done else 0.0
    end = "\n" if done == total else ""
    print(f"\rsweep |{bar}| {done}/{total} ({frac*100:5.1f}%)  "
          f"elapsed {_fmt_mmss(elapsed)}  eta {_fmt_mmss(eta)}", end=end, flush=True)

def make_bin():
    """Floor + 4 thin walls as one multibody. Interior is free space."""
    cx, cy = BIN_CENTER_XY
    hl, hw, hd = BIN_L / 2, BIN_W / 2, BIN_DEPTH / 2
    t = WALL_T
    # (half_extents, center_pos) for each box, bin-local (floor at z=0)
    parts = [
        ([hl, hw, t / 2],        [0, 0, -t / 2]),                 # floor
        ([t / 2, hw, hd],        [hl + t / 2, 0, hd]),            # +x wall
        ([t / 2, hw, hd],        [-hl - t / 2, 0, hd]),           # -x wall
        ([hl + t, t / 2, hd],    [0, hw + t / 2, hd]),            # +y wall
        ([hl + t, t / 2, hd],    [0, -hw - t / 2, hd]),           # -y wall
    ]
    col_ids, vis_ids, positions = [], [], []
    for he, pos in parts:
        col_ids.append(p.createCollisionShape(p.GEOM_BOX, halfExtents=he))
        vis_ids.append(p.createVisualShape(p.GEOM_BOX, halfExtents=he,
                                           rgbaColor=[0.6, 0.6, 0.65, 0.4]))
        positions.append(pos)
    body = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=col_ids[0],
        baseVisualShapeIndex=vis_ids[0],
        basePosition=[cx, cy, 0],
        linkMasses=[0] * (len(parts) - 1),
        linkCollisionShapeIndices=col_ids[1:],
        linkVisualShapeIndices=vis_ids[1:],
        linkPositions=positions[1:],
        linkOrientations=[[0, 0, 0, 1]] * (len(parts) - 1),
        linkInertialFramePositions=[[0, 0, 0]] * (len(parts) - 1),
        linkInertialFrameOrientations=[[0, 0, 0, 1]] * (len(parts) - 1),
        linkParentIndices=[0] * (len(parts) - 1),
        linkJointTypes=[p.JOINT_FIXED] * (len(parts) - 1),
        linkJointAxis=[[0, 0, 1]] * (len(parts) - 1),
    )
    return body

def base_orientation(yaw=0.0):
    """Base quaternion: the hang-down flip (180deg about X) with `yaw` applied
    about the world vertical axis. getQuaternionFromEuler([r,p,y]) yields
    Rz(y)*Ry(p)*Rx(r), so Rx(pi) flips the arm down and Rz(yaw) then swings its
    heading around the vertical."""
    roll = math.pi if FLIP_BASE else 0.0
    return p.getQuaternionFromEuler([roll, 0, yaw])

def load_robot(base_xy, height=MOUNT_HEIGHT, yaw=0.0):
    cx, cy = BIN_CENTER_XY
    pos = [cx + base_xy[0], cy + base_xy[1], height]
    # URDF_USE_SELF_COLLISION so the arm folding through itself is detectable;
    # PyBullet auto-disables the parent<->child pairs that touch by design.
    rid = p.loadURDF(ROBOT_URDF, pos, base_orientation(yaw),
                     useFixedBase=FIXED_BASE, flags=p.URDF_USE_SELF_COLLISION)
    return rid

def make_gripper():
    """The vacuum tool as one massless body whose origin is the flange and whose two
    shapes stack down the local z-axis (the tool's down direction): a thin cylinder
    over the TOP half of the stand-off (the slim mounting body) and the full 400x280
    box over the BOTTOM half (down to the foam face). Created once and teleported onto
    the flange for each collision test. Half offsets: cylinder centered at STANDOFF/4,
    box at 3*STANDOFF/4 below the flange, so the plate's bottom is the foam face."""
    h = GRIPPER_STANDOFF
    shapes = dict(
        shapeTypes=[p.GEOM_CYLINDER, p.GEOM_BOX],
        radii=[GRIPPER_POST_RADIUS, 0.0],
        halfExtents=[[0, 0, 0], [GRIPPER_LENGTH / 2, GRIPPER_WIDTH / 2, h / 4]],
        lengths=[h / 2, 0.0],
    )
    col = p.createCollisionShapeArray(
        collisionFramePositions=[[0, 0, h / 4], [0, 0, 3 * h / 4]], **shapes)
    vis = p.createVisualShapeArray(
        visualFramePositions=[[0, 0, h / 4], [0, 0, 3 * h / 4]],
        rgbaColors=[[0.6, 0.6, 0.6, 0.6], [0.1, 0.5, 1.0, 0.5]], **shapes)
    # Parked far away until placed on the flange.
    return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                             baseVisualShapeIndex=vis, basePosition=[0, 0, -100])

def place_gripper(gripper_id, flange_pos, tool_orn):
    """Teleport the gripper onto the flange: the body origin sits at the flange at
    `tool_orn` (the commanded level down-orientation, so the plate is exactly parallel
    to the ground and clocked by its yaw). The two shapes carry their own offsets down
    the local z-axis, so the body just needs the flange pose."""
    p.resetBasePositionAndOrientation(gripper_id, list(flange_pos), tool_orn)

def movable_joints(rid):
    js = []
    for j in range(p.getNumJoints(rid)):
        if p.getJointInfo(rid, j)[2] != p.JOINT_FIXED:
            js.append(j)
    return js

def joint_limits(rid, joints):
    lo, hi = [], []
    for j in joints:
        info = p.getJointInfo(rid, j)
        l, u = info[8], info[9]
        if l >= u:           # unlimited joint -> give it a wide range
            l, u = -math.pi, math.pi
        lo.append(l); hi.append(u)
    return lo, hi

# ----------------------------------------------------------------------------
# Target grid
# ----------------------------------------------------------------------------
def bin_targets():
    cx, cy = BIN_CENTER_XY
    xs = np.linspace(cx - BIN_L / 2 + MARGIN, cx + BIN_L / 2 - MARGIN, N_X)
    ys = np.linspace(cy - BIN_W / 2 + MARGIN, cy + BIN_W / 2 - MARGIN, N_Y)
    zs = np.linspace(MARGIN, BIN_DEPTH - MARGIN, N_Z)
    pts = [(x, y, z) for z in zs for y in ys for x in xs]
    return np.array(pts), xs, ys, zs

def down_orientations():
    """Tool z-axis pointing down (foam face parallel to the ground), sampled at each
    footprint clocking in TOOL_YAW_DEG (yaw about the vertical, so the rectangle can
    fit different ways) and, if TILT_CONE_DEG>0, a few tilted alternatives. A pick is
    reachable if ANY of these orientations works. roll=pi flips the tool to point
    -z; the yaw term then rotates the (still-vertical) tool about the world z-axis."""
    tilts = [(0.0, 0.0)]
    if TILT_CONE_DEG > 0:
        a = math.radians(TILT_CONE_DEG)
        tilts += [(a, 0.0), (-a, 0.0), (0.0, a), (0.0, -a)]
    # Clocking only matters for a real (rectangular) footprint; with no tool one
    # down-orientation suffices.
    yaws = TOOL_YAW_DEG if USE_GRIPPER else (0.0,)
    oris = []
    for yaw in yaws:
        yz = math.radians(yaw)
        for roll, pitch in tilts:
            oris.append(p.getQuaternionFromEuler([math.pi + roll, pitch, yz]))
    return oris

def ik_seeds(lo, hi, n=None):
    """Initial joint guesses for the IK solver. The numerical solver converges
    to whichever solution is nearest its seed, so seeding from several postures
    is how we discover the multiple configs (elbow up/down, wrist flips) that
    reach the same TCP. First seed is the mid-range pose for determinism. `n`
    overrides N_IK_SEEDS (the dump uses N_SOLUTION_SEEDS to find more configs)."""
    n = N_IK_SEEDS if n is None else n
    rng = np.random.default_rng(0)
    seeds = [[(l + h) / 2 for l, h in zip(lo, hi)]]
    for _ in range(max(n, 0)):
        seeds.append([float(rng.uniform(l, h)) for l, h in zip(lo, hi)])
    return seeds

# ----------------------------------------------------------------------------
# Reachability test for one target
# ----------------------------------------------------------------------------
def _penetrates(body_a, body_b, exclude_adjacent=False):
    """True if any contact between the two bodies penetrates past COLLIDE_TOL.
    With exclude_adjacent (self-collision), ignore neighbouring links in the
    kinematic chain (|A-B|<=1), which touch by design."""
    for c in p.getClosestPoints(body_a, body_b, distance=0.0):
        if exclude_adjacent and abs(c[3] - c[4]) <= 1:   # c[3],c[4] = link indices
            continue
        if c[8] < -COLLIDE_TOL:                          # c[8] = contactDistance
            return True
    return False

def _tool_tilt_deg(orn):
    """Angle (deg) between the tool approach axis (link z) and straight-down."""
    m = p.getMatrixFromQuaternion(orn)      # row-major 3x3; column 2 = link z-axis
    tool_z = (m[2], m[5], m[8])
    cos = max(-1.0, min(1.0, -tool_z[2]))   # dot(tool_z, world-down (0,0,-1))
    return math.degrees(math.acos(cos))

def solve_pick(rid, bin_id, gripper_id, ee_link, joints, lo, hi, target,
               orientations, seeds):
    """The single source of truth for a CENTERED placement: "can the gripper be put
    here with the foam-face center exactly on the target?" Returns the first valid
    joint configuration (and its diagnostics, incl. `ori_idx` = which clocking won) as
    a dict, or None. 'Valid' = the suction face center lands on the target with the
    tool pointing down, in joint limits, and collision-free against the bin, the arm
    itself, and the gripper body. Each seed is a different starting guess, so we
    explore the arm's multiple IK solutions instead of trusting the single config one
    IK call happens to return. Checks run cheapest-first and bail on the first failure.

    A target is *covered* (the reported coverage metric) if it falls under the foam
    footprint of ANY such collision-free placement, not only the one centered on it --
    see `_covered_mask`. This function finds the placements; coverage dilates them.

    Driving the animation and the data dump through this same function (rather than
    a separate IK call) is what makes them reproducible and consistent with the
    reported coverage."""
    target = np.asarray(target, dtype=float)
    # The flange sits a stand-off above the suction contact point (0 if no tool).
    standoff = GRIPPER_STANDOFF if USE_GRIPPER else 0.0
    flange_goal = target + np.array([0.0, 0.0, standoff])
    base_pos = np.array(p.getBasePositionAndOrientation(rid)[0])
    # Reach prune: if even the flange goal is beyond the arm, skip all IK.
    if np.linalg.norm(flange_goal - base_pos) > REACH_MAX:
        return None
    max_tilt = TILT_CONE_DEG + ORI_TOL_DEG
    for ori_idx, ori in enumerate(orientations):
        for seed in seeds:
            # seed both the solver's starting state and its null-space target
            for j, q in zip(joints, seed):
                p.resetJointState(rid, j, q)
            sol = p.calculateInverseKinematics(
                rid, ee_link, flange_goal.tolist(), ori,
                lowerLimits=lo, upperLimits=hi,
                jointRanges=[h - l for l, h in zip(lo, hi)],
                restPoses=seed,
                maxNumIterations=200, residualThreshold=1e-4,
            )
            # apply solution
            for j, q in zip(joints, sol):
                p.resetJointState(rid, j, q)
            ls = p.getLinkState(rid, ee_link, computeForwardKinematics=True)
            flange_pos, flange_orn = np.array(ls[4]), ls[5]
            # (1) position: did the SUCTION FACE land on the target?
            suction = flange_pos - np.array([0.0, 0.0, standoff])
            fk_err = float(np.linalg.norm(suction - target))
            if fk_err > FK_TOL:
                continue
            # (2) orientation: can the arm actually present the tool level here? The
            # tool is assumed always parallel to the ground, so a config the solver
            # can only bring within ORI_TOL of level (numerical residual) is fine,
            # but a larger miss means this down-pose isn't reachable -> reject.
            tilt = _tool_tilt_deg(flange_orn)
            if tilt > max_tilt:
                continue
            # (3) joint-limit check
            if any(q < l - 1e-3 or q > h + 1e-3 for q, l, h in zip(sol, lo, hi)):
                continue
            # (4) arm vs bin
            if _penetrates(rid, bin_id):
                continue
            # (5) arm vs itself
            if _penetrates(rid, rid, exclude_adjacent=True):
                continue
            # (6) gripper body vs bin (footprint clearance + vertical descent).
            # The tool is assumed exactly parallel to the ground, so the plate is
            # placed at the commanded LEVEL orientation `ori` (not the solver's <=
            # ORI_TOL tilted result): a perfectly horizontal plate at this clocking.
            # Skipped entirely when the tool is toggled off (bare flange).
            if USE_GRIPPER and gripper_id is not None:
                place_gripper(gripper_id, flange_pos, ori)
                if _penetrates(gripper_id, bin_id):
                    continue
            return {"joints": [float(q) for q in sol],
                    "flange_pos": flange_pos.tolist(),
                    "tool_orn": list(ori), "ori_idx": ori_idx,
                    "suction": suction.tolist(),
                    "fk_err": fk_err, "tilt_deg": tilt}
    return None

def reachable(rid, bin_id, gripper_id, ee_link, joints, lo, hi, target,
              orientations, seeds):
    """Boolean reachability -- a thin wrapper over solve_pick (which carries the
    full logic so coverage, the data dump, and the animation never diverge)."""
    return solve_pick(rid, bin_id, gripper_id, ee_link, joints, lo, hi, target,
                      orientations, seeds) is not None

def enumerate_solutions(rid, bin_id, gripper_id, ee_link, joints, lo, hi, target,
                        orientations, seeds):
    """EVERY distinct way the joints can be configured to put the foam face on
    `target` pointing down -- for the viewer's "click a point, see how the arm reaches
    (or fails to reach) it" inspector. Unlike solve_pick (which bails on the first valid
    config), this tries all clockings x all seeds, keeps each distinct joint config once
    (rounded), and annotates WHY it fails (reach/unreachable/tilt/limits/collision_*).
    Returns {joints, fk_err, tilt_deg, ok, fail, ori_idx, accepted} per config, ok ones
    first, capped at MAX_SOLUTIONS."""
    target = np.asarray(target, dtype=float)
    standoff = GRIPPER_STANDOFF if USE_GRIPPER else 0.0
    flange_goal = target + np.array([0.0, 0.0, standoff])
    base_pos = np.array(p.getBasePositionAndOrientation(rid)[0])
    reach_ok = np.linalg.norm(flange_goal - base_pos) <= REACH_MAX
    max_tilt = TILT_CONE_DEG + ORI_TOL_DEG
    sols, seen, accepted_found = [], set(), False
    for ori_idx, ori in enumerate(orientations):
        for seed in seeds:
            for j, q in zip(joints, seed):
                p.resetJointState(rid, j, q)
            sol = p.calculateInverseKinematics(
                rid, ee_link, flange_goal.tolist(), ori,
                lowerLimits=lo, upperLimits=hi,
                jointRanges=[h - l for l, h in zip(lo, hi)],
                restPoses=seed, maxNumIterations=200, residualThreshold=1e-4)
            for j, q in zip(joints, sol):
                p.resetJointState(rid, j, q)
            key = tuple(round(q, 2) for q in sol)
            if key in seen:
                continue
            seen.add(key)
            ls = p.getLinkState(rid, ee_link, computeForwardKinematics=True)
            flange_pos, flange_orn = np.array(ls[4]), ls[5]
            suction = flange_pos - np.array([0.0, 0.0, standoff])
            fk_err = float(np.linalg.norm(suction - target))
            tilt = _tool_tilt_deg(flange_orn)
            fail = None
            if not reach_ok:
                fail = "reach"
            elif fk_err > FK_TOL:
                fail = "unreachable"
            elif tilt > max_tilt:
                fail = "tilt"
            elif any(q < l - 1e-3 or q > h + 1e-3 for q, l, h in zip(sol, lo, hi)):
                fail = "limits"
            elif _penetrates(rid, bin_id):
                fail = "collision_bin"
            elif _penetrates(rid, rid, exclude_adjacent=True):
                fail = "collision_self"
            elif USE_GRIPPER and gripper_id is not None:
                place_gripper(gripper_id, flange_pos, ori)
                if _penetrates(gripper_id, bin_id):
                    fail = "collision_gripper"
            ok = fail is None
            accepted = ok and not accepted_found
            accepted_found = accepted_found or ok
            sols.append({"joints": [float(q) for q in sol], "fk_err": fk_err,
                         "tilt_deg": tilt, "ok": ok, "fail": fail,
                         "ori_idx": ori_idx, "accepted": accepted})
    sols.sort(key=lambda s: (not s["accepted"], not s["ok"], s["fk_err"]))
    return sols[:MAX_SOLUTIONS]

# ----------------------------------------------------------------------------
# Main sweep
# ----------------------------------------------------------------------------
def set_base(rid, base_xy, height, yaw=0.0):
    """Move the (fixed) robot base to a new mount point and heading."""
    cx, cy = BIN_CENTER_XY
    p.resetBasePositionAndOrientation(
        rid, [cx + base_xy[0], cy + base_xy[1], height], base_orientation(yaw))

# ----------------------------------------------------------------------------
# Area coverage: a target is COVERED if it falls under the foam footprint of any
# collision-free placement -- not only the placement centered on it. We get this by
# DILATING the reachable-center grid by the plate footprint (per clocking, within
# each depth layer), so near-wall points the gripper can pick off-center count too.
# ----------------------------------------------------------------------------
def _footprint_offsets(ang, dx, dy, L, W):
    """Grid (di, dj) index offsets in x/y whose world displacement (di*dx, dj*dy)
    from a plate center lies under the L x W footprint clocked by `ang` (the plate's
    long axis heading in world). A center at (ix,iy) therefore covers (ix+di, iy+dj)."""
    if dx <= 0 or dy <= 0 or (L <= 0 and W <= 0):
        return [(0, 0)]
    c, s = math.cos(ang), math.sin(ang)
    rmax = max(L, W) / 2.0
    nx, ny = int(rmax / dx), int(rmax / dy)
    offs = []
    for di in range(-nx, nx + 1):
        for dj in range(-ny, ny + 1):
            wx, wy = di * dx, dj * dy
            along =  c * wx + s * wy        # projection on the plate long (L) axis
            across = -s * wx + c * wy       # projection on the plate short (W) axis
            if abs(along) <= L / 2 + 1e-9 and abs(across) <= W / 2 + 1e-9:
                offs.append((di, dj))
    return offs

def _cover_offsets(orientations, dx, dy):
    """Footprint offsets for each clocking in `orientations` (index-aligned). With no
    tool, every clocking covers just its own cell."""
    if not USE_GRIPPER:
        return [[(0, 0)] for _ in orientations]
    offs = []
    for ori in orientations:
        m = p.getMatrixFromQuaternion(ori)         # plate long axis (local x) in world
        ang = math.atan2(m[3], m[0])
        offs.append(_footprint_offsets(ang, dx, dy, GRIPPER_LENGTH, GRIPPER_WIDTH))
    return offs

def _dilate_layerwise(mask3, offsets):
    """OR-dilate a (N_Z, N_Y, N_X) center mask by grid offsets, within each depth
    layer only (the foam footprint is horizontal at the target's depth)."""
    out = np.zeros_like(mask3)
    nz, ny, nx = mask3.shape
    for di, dj in offsets:                          # center (ix,iy) -> covers (ix+di,iy+dj)
        xd0, xd1 = max(0, di), nx + min(0, di)
        yd0, yd1 = max(0, dj), ny + min(0, dj)
        out[:, yd0:yd1, xd0:xd1] |= mask3[:, max(0, -dj):ny + min(0, -dj),
                                          max(0, -di):nx + min(0, -di)]
    return out

def _covered_mask(center_by_clk, cover_offsets):
    """Union over clockings of the dilated reachable-center masks -> covered grid
    (flat, target order). center_by_clk[k] is the centers reachable at clocking k."""
    covered = np.zeros(center_by_clk.shape[1], dtype=bool)
    cov3 = covered.reshape(N_Z, N_Y, N_X)           # view; writes hit `covered`
    for k in range(center_by_clk.shape[0]):
        cov3 |= _dilate_layerwise(center_by_clk[k].reshape(N_Z, N_Y, N_X),
                                  cover_offsets[k])
    return covered

# The FR20 meshes are heavy, so each process builds the world ONCE and just
# relocates the base for every sweep point. `_W` holds the current process's world.
_W = None

def build_world(force_gui=False):
    """Build one PyBullet world (bin + gripper + robot) plus the cached joint and
    IK-seed metadata. Called once per process (each parallel worker gets its own).
    force_gui opens a GUI window even when SHOW_SIM is off (for the post-run viewer)."""
    connect(force_gui=force_gui)
    bin_id = make_bin()
    gripper_id = make_gripper() if USE_GRIPPER else None
    rid = load_robot((0.0, 0.0), MOUNT_HEIGHT)
    joints = movable_joints(rid)
    lo, hi = joint_limits(rid, joints)
    ee = EE_LINK if EE_LINK is not None else p.getNumJoints(rid) - 1
    targets, xs, ys, _ = bin_targets()
    dx = float(xs[1] - xs[0]) if len(xs) > 1 else BIN_L
    dy = float(ys[1] - ys[0]) if len(ys) > 1 else BIN_W
    orientations = down_orientations()
    return {"rid": rid, "bin_id": bin_id, "gripper_id": gripper_id,
            "joints": joints, "lo": lo, "hi": hi, "ee": ee,
            "seeds": ik_seeds(lo, hi), "orientations": orientations,
            "targets": targets, "cover_offsets": _cover_offsets(orientations, dx, dy)}

def _worker_init():
    global _W
    _W = build_world()

def _eval_pose(args):
    """Evaluate one base pose -> (pose_idx, covered_mask, center_mask). center_mask =
    grid points where the plate CAN be centered collision-free; covered_mask = points
    that fall under the footprint of some such placement (center_mask dilated by the
    plate). Coverage is reported from covered_mask; center_mask drives the animation
    and the joint-config dump (those are the real, achievable arm poses)."""
    pose_idx, bx, by, bz, byaw = args
    w = _W
    set_base(w["rid"], (bx, by), bz, byaw)
    n, nclk = len(w["targets"]), len(w["orientations"])
    center_by_clk = np.zeros((nclk, n), dtype=bool)
    for i, t in enumerate(w["targets"]):
        sol = solve_pick(w["rid"], w["bin_id"], w["gripper_id"], w["ee"], w["joints"],
                         w["lo"], w["hi"], t, w["orientations"], w["seeds"])
        if sol is not None:
            center_by_clk[sol["ori_idx"], i] = True
    covered = _covered_mask(center_by_clk, w["cover_offsets"])
    return pose_idx, covered, center_by_clk.any(axis=0)

def _pose_list():
    """All base poses in nested (yaw, z, y, x) order. pose_idx is the position in
    this list, so aggregation order -- and the result -- is independent of the
    order in which workers happen to finish."""
    poses, meta, idx = [], [], 0
    for byaw in BASE_YAW_RANGE:
        for iz, bz in enumerate(BASE_Z_RANGE):
            for iy, by in enumerate(BASE_Y_RANGE):
                for ix, bx in enumerate(BASE_X_RANGE):
                    poses.append((idx, bx, by, bz, byaw))
                    meta.append((iz, iy, ix, bx, by, bz, byaw))
                    idx += 1
    return poses, meta

def _aggregate(targets, meta, results):
    """Fold per-pose masks into coverage / target_hits / a fully ranked pose list,
    all deterministically in pose order. coverage[iz,iy,ix] keeps the BEST coverage
    over all yaws at that XY/height. `ranked` is every base pose sorted by coverage
    desc, ties broken by earliest pose (matching the serial run); ranked[0] is THE
    best. Callers slice ranked[:N_BEST] for the top list and run _select_diverse on
    it for the spread-out alternatives."""
    coverage = np.zeros((len(BASE_Z_RANGE), len(BASE_Y_RANGE), len(BASE_X_RANGE)))
    target_hits = np.zeros(len(targets))         # how many bases COVER each target
    ranked = []
    for idx in range(len(meta)):
        covered, center = results[idx]
        iz, iy, ix, bx, by, bz, byaw = meta[idx]
        cov = float(covered.mean())
        coverage[iz, iy, ix] = max(coverage[iz, iy, ix], cov)
        target_hits += covered
        ranked.append((cov, idx, bx, by, bz, byaw, covered, center))
    ranked.sort(key=lambda r: (-r[0], r[1]))
    ranked = [{"cov": cov, "xy": (bx, by), "z": bz, "yaw": byaw,
               "mask": covered, "center_mask": center}
              for cov, idx, bx, by, bz, byaw, covered, center in ranked]
    return coverage, target_hits, ranked

def _norm01(v, rng):
    """Normalize a value to [0,1] over a sweep range (0 if the range is a point)."""
    lo, hi = float(np.min(rng)), float(np.max(rng))
    return 0.0 if hi <= lo else (float(v) - lo) / (hi - lo)

def _genome(b):
    """A base pose as a normalized (x, y, z, yaw) vector -- its 'weights'."""
    return np.array([_norm01(b["xy"][0], BASE_X_RANGE),
                     _norm01(b["xy"][1], BASE_Y_RANGE),
                     _norm01(b["z"], BASE_Z_RANGE),
                     _norm01(b["yaw"], BASE_YAW_RANGE)])

def _select_diverse(ranked, n=None, min_dist=None):
    """Best-of-different-sections: greedily pick high-coverage poses that are far
    apart in the normalized base-pose ('genome') space. Walking `ranked` from the
    top, a pose joins the set only if it is at least `min_dist` from every pose
    already chosen -- so each is the best in its own region. The result is several
    mounts that reach the bin with similar efficacy via very different base poses
    (the quality-diversity / 'different mutations, similar fitness' view), as opposed
    to the top-N which usually cluster around one spot."""
    n = N_DIVERSE if n is None else n
    min_dist = DIVERSE_MIN_DIST if min_dist is None else min_dist
    picks = []
    for b in ranked:                       # ranked is coverage-desc, so picks[0]=best
        g = _genome(b)
        if all(np.linalg.norm(g - _genome(q)) >= min_dist for q in picks):
            picks.append(b)
            if len(picks) >= n:
                break
    return picks

def _sweep_gui(targets, poses, meta, total):
    """Single-process sweep with the live GUI/HUD (a shared GUI can't be driven
    from worker processes). Settles the arm on a reachable pick at each base so the
    scan is watchable, then aggregates the same way as the headless paths."""
    global _W
    _W = build_world()
    hud = hud_create()
    print("GUI: watch the arm scan each mount position; the full sweep takes "
          "a while, so use a smaller grid if you just want a quick look.")
    results, t0, done, best = {}, time.time(), 0, {"cov": -1}
    for ps in poses:
        pose_idx, covered, center = _eval_pose(ps)
        results[pose_idx] = (covered, center)
        _, bx, by, bz, byaw = ps
        cov = covered.mean()
        if cov > best["cov"]:
            best = {"cov": cov, "xy": (bx, by), "z": bz, "yaw": byaw}
        done += 1
        progress_bar(done, total, t0)
        place_pts = targets[center]            # real, achievable plate placements
        if len(place_pts):
            pose_at(_W["rid"], _W["ee"], _W["joints"], _W["lo"], _W["hi"],
                    place_pts[len(place_pts) // 2], _W["orientations"],
                    _W["gripper_id"])
        elapsed = time.time() - t0
        eta = elapsed / done * (total - done)
        hud_update(hud, [
            "FR20 bin reachability  -  SWEEP",
            f"base   x={bx:+.2f}  y={by:+.2f}  z={bz:.2f} m  "
            f"yaw={math.degrees(byaw):+.0f}deg",
            f"coverage here:   {cov*100:4.1f}%   "
            f"({int(covered.sum())}/{len(targets)} covered)",
            f"best so far:   {best['cov']*100:4.1f}%   "
            f"@ x={best['xy'][0]:+.2f} y={best['xy'][1]:+.2f} "
            f"z={best['z']:.2f} yaw={math.degrees(best['yaw']):+.0f}",
            f"progress:   {done}/{total}  ({done/total*100:4.1f}%)",
            f"elapsed {_fmt_mmss(elapsed)}    eta {_fmt_mmss(eta)}",
        ])
        time.sleep(SIM_DWELL)
    coverage, target_hits, ranked = _aggregate(targets, meta, results)
    return coverage, target_hits, ranked, hud

def run():
    # Each run writes its diagrams to out/run_<timestamp>/ so results don't clobber.
    global RUN_DIR, _W
    RUN_DIR = os.path.join(OUT_DIR, time.strftime("run_%Y%m%d_%H%M%S"))
    targets, xs, ys, zs = bin_targets()
    poses, meta = _pose_list()
    total = len(poses)
    hud = None

    if SHOW_SIM:
        coverage, target_hits, ranked, hud = _sweep_gui(targets, poses, meta, total)
    else:
        results, t0, done = {}, time.time(), 0
        if N_WORKERS and N_WORKERS > 1:
            print(f"Sweeping {total} base poses x {len(targets)} targets "
                  f"on {N_WORKERS} workers...")
            with mp.Pool(N_WORKERS, initializer=_worker_init) as pool:
                for pose_idx, covered, center in pool.imap_unordered(_eval_pose, poses):
                    results[pose_idx] = (covered, center)
                    done += 1
                    progress_bar(done, total, t0)
        else:
            _worker_init()
            for ps in poses:
                pose_idx, covered, center = _eval_pose(ps)
                results[pose_idx] = (covered, center)
                done += 1
                progress_bar(done, total, t0)
        coverage, target_hits, ranked = _aggregate(targets, meta, results)

    bests = ranked[:max(N_BEST, 1)]
    diverse = _select_diverse(ranked) if N_DIVERSE > 0 else []
    best = bests[0]
    target_freq = target_hits / total      # fraction of bases reaching each target

    def _print_pose(tag, b):
        print(f"  {tag}: x={b['xy'][0]:+.3f} y={b['xy'][1]:+.3f} z={b['z']:.3f} m  "
              f"yaw={math.degrees(b['yaw']):+5.1f} deg  ->  {b['cov']*100:5.1f}%  "
              f"({int(b['mask'].sum())}/{len(targets)})")

    print(f"\nTop {len(bests)} base poses by coverage:")
    for r, b in enumerate(bests, 1):
        _print_pose(f"#{r}", b)
    if diverse:
        print(f"Best of {len(diverse)} different sections (high coverage, far apart "
              f"in base pose -- similar efficacy, very different mounts):")
        for r, b in enumerate(diverse, 1):
            _print_pose(f"D{r}", b)
    print(f"Heights tried: {[round(float(z), 3) for z in BASE_Z_RANGE]} m | "
          f"yaws tried: {[round(math.degrees(y), 1) for y in BASE_YAW_RANGE]} deg | "
          f"targets evaluated: {len(targets)}")

    # --- diagrams ---
    os.makedirs(RUN_DIR, exist_ok=True)
    saved = [
        plot_coverage_grid(coverage, best),
        plot_best_slices(best, xs, ys, zs),
        plot_reach_3d(best, targets),
        plot_coverage_vs_height(coverage),
        plot_target_frequency(target_freq, xs, ys, zs),
    ]

    # --- reproducible end animation + detailed data on the top-N bases ---
    # Needs a world in THIS process. SHOW_SIM already has one (_W); the parallel
    # path leaves the parent worldless, so build one here.
    if _W is None:
        _worker_init()
    saved.append(dump_best_data(_W, bests, diverse, targets, coverage, target_freq))
    if SAVE_ANIMATION:
        saved.append(render_animation(_W, best, targets))

    print(f"Saved {len(saved)} artifacts to {RUN_DIR}:")
    for pth in saved:
        print(f"  - {os.path.basename(pth)}")

    # --- view the best base in the GUI ---
    # SHOW_SIM already has a live GUI world (_W); a headless run can still pop one
    # open afterwards (SHOW_BEST_AFTER) by swapping its DIRECT world for a GUI one.
    if not SHOW_SIM and SHOW_BEST_AFTER:
        try:
            if p.isConnected():
                p.disconnect()
            print("\nOpening the best base in a GUI "
                  "(SHOW_BEST_AFTER) -- close the window or press Ctrl-C to quit.")
            _W = build_world(force_gui=True)
            hud = hud_create()
        except p.error as e:                 # no display available -> skip gracefully
            print(f"Could not open a GUI ({e}); skipping the post-run viewer.")
            return bests
    if SHOW_SIM or SHOW_BEST_AFTER:
        visualize_best(_W["rid"], _W["ee"], _W["joints"], _W["lo"], _W["hi"],
                       targets, _W["orientations"], best, hud, _W["gripper_id"])
    return bests

# ----------------------------------------------------------------------------
# Diagrams
# ----------------------------------------------------------------------------
def _cell_extent(xs, ys):
    """imshow extent that centers each grid sample in its own cell."""
    dx = (xs[1] - xs[0]) if len(xs) > 1 else BIN_L
    dy = (ys[1] - ys[0]) if len(ys) > 1 else BIN_W
    return [xs[0] - dx / 2, xs[-1] + dx / 2, ys[0] - dy / 2, ys[-1] + dy / 2]

def _bin_footprint(ax, ec, lw=1.0, ls="-", alpha=0.6):
    ax.add_patch(Rectangle((BIN_CENTER_XY[0] - BIN_L / 2,
                            BIN_CENTER_XY[1] - BIN_W / 2),
                           BIN_L, BIN_W, fill=False, ec=ec, lw=lw, ls=ls,
                           alpha=alpha))

def plot_coverage_grid(coverage, best):
    """Coverage % over the base XY grid, one panel per mount height,
    with per-cell values, the bin footprint, and the best base starred."""
    nz = len(BASE_Z_RANGE)
    vmax = max(coverage.max() * 100, 1e-6)
    annotate = len(BASE_X_RANGE) <= 9 and len(BASE_Y_RANGE) <= 9
    fig, axes = plt.subplots(1, nz, figsize=(4.6 * nz, 4.9), squeeze=False)
    im = None
    for k, bz in enumerate(BASE_Z_RANGE):
        ax = axes[0][k]
        data = coverage[k] * 100
        im = ax.imshow(data, origin="lower", cmap="viridis", vmin=0, vmax=vmax,
                       extent=[BASE_X_RANGE[0], BASE_X_RANGE[-1],
                               BASE_Y_RANGE[0], BASE_Y_RANGE[-1]], aspect="auto")
        _bin_footprint(ax, ec="white", lw=1.2, ls="--", alpha=0.7)
        if annotate:
            for iy, by in enumerate(BASE_Y_RANGE):
                for ix, bx in enumerate(BASE_X_RANGE):
                    v = data[iy, ix]
                    ax.text(bx, by, f"{v:.0f}", ha="center", va="center",
                            fontsize=7,
                            color="white" if v < vmax * 0.55 else "black")
        ax.set_xlim(BASE_X_RANGE[0], BASE_X_RANGE[-1])
        ax.set_ylim(BASE_Y_RANGE[0], BASE_Y_RANGE[-1])
        ax.set_title(f"mount z = {bz:.2f} m  (max {data.max():.0f}%)", fontsize=9)
        ax.set_xlabel("base x offset (m)")
        if k == 0:
            ax.set_ylabel("base y offset (m)")
        if abs(bz - best["z"]) < 1e-9:
            ax.scatter([best["xy"][0]], [best["xy"][1]], c="red", marker="*",
                       s=240, edgecolors="white", linewidths=0.8, zorder=5,
                       label="best base")
            ax.legend(loc="upper right", fontsize=8)
    fig.suptitle(f"Bin coverage vs. base position (best over yaw per cell)   |   "
                 f"best: x={best['xy'][0]:+.2f}, y={best['xy'][1]:+.2f}, "
                 f"z={best['z']:.2f} m, yaw={math.degrees(best['yaw']):+.0f}deg  ->  "
                 f"{best['cov']*100:.1f}%   (dashed = bin footprint)", fontsize=11)
    fig.colorbar(im, ax=axes.ravel().tolist(), label="coverage %")
    path = os.path.join(RUN_DIR, "coverage_vs_base.png")
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path

def plot_best_slices(best, xs, ys, zs):
    """Covered pick points at the best base, sliced by depth, with the
    bin outline, the sample grid ticks, and per-slice counts."""
    mask = best["mask"].reshape(N_Z, N_Y, N_X)
    extent = _cell_extent(xs, ys)
    fig, axes = plt.subplots(1, N_Z, figsize=(3.4 * N_Z, 3.9), squeeze=False)
    for k in range(N_Z):
        a = axes[0][k]
        a.imshow(mask[k], origin="lower", cmap="RdYlGn", vmin=0, vmax=1,
                 extent=extent, aspect="equal")
        _bin_footprint(a, ec="black", lw=1.0, alpha=0.5)
        a.set_xticks(np.round(xs, 2)); a.set_yticks(np.round(ys, 2))
        a.tick_params(labelsize=7)
        for lbl in a.get_xticklabels():
            lbl.set_rotation(90)
        n_ok, n_tot = int(mask[k].sum()), mask[k].size
        a.set_title(f"z = {zs[k]:.2f} m\n{n_ok}/{n_tot} covered "
                    f"({mask[k].mean()*100:.0f}%)", fontsize=9)
        a.set_xlabel("x (m)")
        if k == 0:
            a.set_ylabel("y (m)")
    fig.suptitle(f"Covered pick points by depth at best base  "
                 f"(x={best['xy'][0]:+.2f}, y={best['xy'][1]:+.2f}, "
                 f"z={best['z']:.2f} m, yaw={math.degrees(best['yaw']):+.0f}deg)   "
                 f"green = covered (foam can reach it), red = not", fontsize=11)
    fig.tight_layout()
    path = os.path.join(RUN_DIR, "best_pos_slices.png")
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path

def _draw_bin_wireframe(ax):
    cx, cy = BIN_CENTER_XY
    x0, x1 = cx - BIN_L / 2, cx + BIN_L / 2
    y0, y1 = cy - BIN_W / 2, cy + BIN_W / 2
    z0, z1 = 0.0, BIN_DEPTH
    for z in (z0, z1):                                   # floor + opening rings
        ring = np.array([(x0, y0, z), (x1, y0, z), (x1, y1, z),
                         (x0, y1, z), (x0, y0, z)])
        ax.plot(ring[:, 0], ring[:, 1], ring[:, 2], c="gray", lw=1.0)
    for xx, yy in [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]:   # vertical edges
        ax.plot([xx, xx], [yy, yy], [z0, z1], c="gray", lw=1.0)

def plot_reach_3d(best, targets):
    """3D scatter of covered vs not-covered pick points at the best base,
    with the bin drawn as a wireframe and the robot base marked."""
    reach = targets[best["mask"]]
    miss = targets[~best["mask"]]
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(projection="3d")
    _draw_bin_wireframe(ax)
    if len(miss):
        ax.scatter(miss[:, 0], miss[:, 1], miss[:, 2], c="red", marker="x",
                   s=30, alpha=0.6, label=f"not covered ({len(miss)})")
    if len(reach):
        ax.scatter(reach[:, 0], reach[:, 1], reach[:, 2], c="green", marker="o",
                   s=35, alpha=0.9, label=f"covered ({len(reach)})")
    ax.scatter([best["xy"][0]], [best["xy"][1]], [best["z"]], c="black",
               marker="^", s=130, label=f"robot base (z={best['z']:.2f} m)")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
    ax.set_title(f"Covered pick points in 3D at best base  "
                 f"({best['cov']*100:.1f}% coverage)")
    ax.legend(loc="upper left", fontsize=8)
    ax.view_init(elev=22, azim=-60)
    path = os.path.join(RUN_DIR, "reach_3d.png")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path

def plot_coverage_vs_height(coverage):
    """Best-base and mean coverage as a function of mount height."""
    heights = np.array(BASE_Z_RANGE)
    flat = coverage.reshape(len(heights), -1) * 100
    max_cov, mean_cov = flat.max(axis=1), flat.mean(axis=1)
    kbest = int(np.argmax(max_cov))
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot(heights, max_cov, "-o", color="C0", label="best base at this height")
    ax.plot(heights, mean_cov, "--s", color="C1", label="mean over all bases")
    ax.scatter([heights[kbest]], [max_cov[kbest]], c="red", marker="*", s=260,
               zorder=5, label="overall best")
    ax.annotate(f"{max_cov[kbest]:.1f}% @ {heights[kbest]:.2f} m",
                (heights[kbest], max_cov[kbest]), textcoords="offset points",
                xytext=(8, 8), color="red", fontsize=9)
    ax.set_xlabel("mount height z (m)"); ax.set_ylabel("coverage %")
    ax.set_title("Coverage vs. mount height")
    ax.grid(True, alpha=0.3); ax.legend()
    path = os.path.join(RUN_DIR, "coverage_vs_height.png")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path

def plot_target_frequency(target_freq, xs, ys, zs):
    """For every pick point, the % of all swept base positions that can reach
    it -- highlights bin regions that are intrinsically hard, sliced by depth."""
    freq = target_freq.reshape(N_Z, N_Y, N_X) * 100
    extent = _cell_extent(xs, ys)
    fig, axes = plt.subplots(1, N_Z, figsize=(3.4 * N_Z, 3.9), squeeze=False)
    im = None
    for k in range(N_Z):
        a = axes[0][k]
        im = a.imshow(freq[k], origin="lower", cmap="magma", vmin=0, vmax=100,
                      extent=extent, aspect="equal")
        for iy, yv in enumerate(ys):
            for ix, xv in enumerate(xs):
                v = freq[k, iy, ix]
                a.text(xv, yv, f"{v:.0f}", ha="center", va="center",
                       fontsize=7, color="white" if v < 55 else "black")
        _bin_footprint(a, ec="cyan", lw=1.0, alpha=0.6)
        a.set_title(f"z = {zs[k]:.2f} m\n(mean {freq[k].mean():.0f}%)", fontsize=9)
        a.set_xlabel("x (m)")
        if k == 0:
            a.set_ylabel("y (m)")
    fig.suptitle("Target coverage across ALL base positions "
                 "(% of swept bases whose foam can reach each point)", fontsize=11)
    fig.colorbar(im, ax=axes.ravel().tolist(), label="% of bases")
    path = os.path.join(RUN_DIR, "target_reachability.png")
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path

# ----------------------------------------------------------------------------
# Detailed data dump + reproducible end animation (run after every sweep)
# ----------------------------------------------------------------------------
def _sol_rec(s):
    """JSON-friendly record for one enumerated IK solution (viewer click-inspect)."""
    return {"joints_rad": [round(q, 5) for q in s["joints"]],
            "fk_err_mm": round(s["fk_err"] * 1000, 2),
            "tilt_deg": round(s["tilt_deg"], 2),
            "ok": bool(s["ok"]), "fail": s["fail"],
            "tool_yaw_deg": (round(float(TOOL_YAW_DEG[s["ori_idx"]]), 1)
                             if USE_GRIPPER else 0.0),
            "accepted": bool(s["accepted"])}

def _pick_records(world, base, targets, covered, sol_seeds=None):
    """Per-point detail for one base pose. `covered` marks points the foam footprint
    reaches (the reported metric). Each point is flagged where the plate can be CENTERED
    -- a real placement -- with its joint solution / FK error / tool tilt. When
    `sol_seeds` is given (DUMP_SOLUTIONS), every distinct IK config for the TCP is
    enumerated into `solutions` (valid + failed, with the reason) for the viewer's
    click-to-inspect / failure paths; the accepted one supplies the top-level fields."""
    set_base(world["rid"], base["xy"], base["z"], base["yaw"])
    recs = []
    for t, cov in zip(targets, covered):
        rec = {"target_m": [round(float(v), 4) for v in t], "covered": bool(cov)}
        if sol_seeds is not None:
            sols = enumerate_solutions(world["rid"], world["bin_id"],
                                       world["gripper_id"], world["ee"], world["joints"],
                                       world["lo"], world["hi"], t,
                                       world["orientations"], sol_seeds)
            acc = next((s for s in sols if s["accepted"]), None)
            rec["is_placement"] = acc is not None
            rec["solutions"] = [_sol_rec(s) for s in sols]
            sol = acc
        else:
            sol = solve_pick(world["rid"], world["bin_id"], world["gripper_id"],
                             world["ee"], world["joints"], world["lo"], world["hi"],
                             t, world["orientations"], world["seeds"])
            rec["is_placement"] = sol is not None
        if sol is not None:
            rec["joints_rad"] = [round(q, 5) for q in sol["joints"]]
            rec["joints_deg"] = [round(math.degrees(q), 2) for q in sol["joints"]]
            rec["fk_err_mm"] = round(sol["fk_err"] * 1000, 2)
            rec["tool_tilt_deg"] = round(sol["tilt_deg"], 2)
            # Which footprint clocking won (deg about the vertical) -- lets a viewer
            # draw the plate at the actual rotation it was placed at. 0 with no tool.
            rec["tool_yaw_deg"] = (round(float(TOOL_YAW_DEG[sol["ori_idx"]]), 1)
                                   if USE_GRIPPER else 0.0)
        recs.append(rec)
    return recs

def _best_block(world, b, rank, targets, zs, sol_seeds=None):
    """One base pose's full record (summary + per-depth COVERED counts + every pick's
    covered/placement flags and joint solution) plus its (n_targets x n_joints) joint
    array for the NPZ (filled at the centered placements)."""
    recs = _pick_records(world, b, targets, b["mask"], sol_seeds)
    mask3 = b["mask"].reshape(N_Z, N_Y, N_X)
    per_depth = [{"z_m": round(float(zs[k]), 3),
                  "covered": int(mask3[k].sum()),
                  "total": int(mask3[k].size),
                  "pct": round(float(mask3[k].mean()) * 100, 1)}
                 for k in range(N_Z)]
    block = {
        "rank": rank,
        "base": {"x": round(float(b["xy"][0]), 4),
                 "y": round(float(b["xy"][1]), 4),
                 "z": round(float(b["z"]), 4),
                 "yaw_deg": round(math.degrees(b["yaw"]), 2)},
        "coverage_pct": round(b["cov"] * 100, 2),
        "covered": int(b["mask"].sum()), "total": int(b["mask"].size),
        "placements": int(b["center_mask"].sum()),
        "per_depth": per_depth,
        "picks": recs,
    }
    jc = np.full((len(targets), len(world["joints"])), np.nan)
    for i, rec in enumerate(recs):
        if rec["is_placement"]:
            jc[i] = rec["joints_rad"]
    return block, jc

def dump_best_data(world, bests, diverse, targets, coverage, target_freq):
    """Write a detailed, reproducible bundle on the top-N base poses AND the diverse
    "best of different sections":
      best_versions.json -- config, then for each pose a summary, per-depth counts,
                            and the joint solution / FK error / tool tilt per pick;
      best_versions.npz  -- raw arrays (coverage grid, target frequency, and per
                            pose the reach mask + joint configs, NaN where blocked).
    """
    os.makedirs(RUN_DIR, exist_ok=True)
    zs = np.linspace(MARGIN, BIN_DEPTH - MARGIN, N_Z)
    total_poses = (len(BASE_X_RANGE) * len(BASE_Y_RANGE)
                   * len(BASE_Z_RANGE) * len(BASE_YAW_RANGE))
    gripper_cfg = ({"enabled": True,
                    "model": "Schmalz FQE/FXCB 400x280 (cylinder post + plate)",
                    "length_m": GRIPPER_LENGTH, "width_m": GRIPPER_WIDTH,
                    "standoff_m": GRIPPER_STANDOFF,
                    "post_radius_m": GRIPPER_POST_RADIUS,
                    "tool_yaw_deg": list(TOOL_YAW_DEG),
                    "note": "thin post over top half, 400x280 plate over bottom half; "
                            "standoff is an estimate; footprint from datasheet"}
                   if USE_GRIPPER else
                   {"enabled": False, "note": "bare flange (USE_GRIPPER = False)"})
    config = {
        "robot_urdf": os.path.basename(ROBOT_URDF),
        "bin_LxWxD_m": [BIN_L, BIN_W, BIN_DEPTH],
        "gripper": gripper_cfg,
        "tolerances": {"fk_tol_m": FK_TOL, "collide_tol_m": COLLIDE_TOL,
                       "tilt_cone_deg": TILT_CONE_DEG, "ori_tol_deg": ORI_TOL_DEG,
                       "reach_max_m": REACH_MAX},
        "target_grid": {"nx": N_X, "ny": N_Y, "nz": N_Z, "margin_m": MARGIN},
        "base_sweep": {"x": [round(float(v), 4) for v in BASE_X_RANGE],
                       "y": [round(float(v), 4) for v in BASE_Y_RANGE],
                       "z": [round(float(v), 4) for v in BASE_Z_RANGE],
                       "yaw_deg": [round(math.degrees(v), 1) for v in BASE_YAW_RANGE]},
        "n_ik_seeds": N_IK_SEEDS, "n_workers": N_WORKERS,
        "diverse_min_dist": DIVERSE_MIN_DIST,
    }
    sol_seeds = (ik_seeds(world["lo"], world["hi"], N_SOLUTION_SEEDS)
                 if DUMP_SOLUTIONS else None)
    npz = {"coverage": coverage,
           "target_freq": target_freq.reshape(N_Z, N_Y, N_X),
           "targets_m": targets}
    top_blocks = []
    for r, b in enumerate(bests, 1):
        block, jc = _best_block(world, b, r, targets, zs, sol_seeds)
        top_blocks.append(block)
        npz[f"best{r}_covered_mask"] = b["mask"]
        npz[f"best{r}_center_mask"] = b["center_mask"]
        npz[f"best{r}_joints_rad"] = jc
    diverse_blocks = []
    for r, b in enumerate(diverse, 1):
        block, jc = _best_block(world, b, r, targets, zs, sol_seeds)
        diverse_blocks.append(block)
        npz[f"diverse{r}_covered_mask"] = b["mask"]
        npz[f"diverse{r}_center_mask"] = b["center_mask"]
        npz[f"diverse{r}_joints_rad"] = jc
    bundle = {
        "config": config,
        "summary": {"total_base_poses": total_poses, "total_targets": len(targets),
                    "n_best_reported": len(bests),
                    "n_diverse_reported": len(diverse)},
        "top_bests": top_blocks,
        "diverse_bests": diverse_blocks,
    }
    jpath = os.path.join(RUN_DIR, "best_versions.json")
    with open(jpath, "w") as f:
        json.dump(bundle, f, indent=2)
    np.savez_compressed(os.path.join(RUN_DIR, "best_versions.npz"), **npz)
    return jpath

def _add_target_markers(targets, mask):
    """Visual-only spheres (green=reachable, red=not) so they show up in the
    rendered animation (debug points/lines are not captured by getCameraImage)."""
    ids = []
    for t, ok in zip(targets, mask):
        color = [0.1, 0.9, 0.2, 1] if ok else [0.9, 0.15, 0.15, 1]
        vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.018, rgbaColor=color)
        ids.append(p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1,
                                     baseVisualShapeIndex=vis,
                                     basePosition=[float(t[0]), float(t[1]),
                                                   float(t[2])]))
    return ids

def render_animation(world, best, targets):
    """Reproducible end animation: at the best base, drive the arm through its
    reachable picks with the offline software renderer and save a GIF -- no GUI
    needed, so it is produced after every run and is identical each time."""
    os.makedirs(RUN_DIR, exist_ok=True)
    rid = world["rid"]
    set_base(rid, best["xy"], best["z"], best["yaw"])
    # markers: green = covered (the reported metric); the arm visits the real
    # collision-free plate placements (center_mask), which cover that whole region.
    _add_target_markers(targets, best["mask"])
    reach_pts = targets[best.get("center_mask", best["mask"])]
    view = p.computeViewMatrixFromYawPitchRoll(
        [BIN_CENTER_XY[0], BIN_CENTER_XY[1], BIN_DEPTH / 2], 2.9, 50, -35, 0, 2)
    proj = p.computeProjectionMatrixFOV(60, ANIM_W / ANIM_H, 0.1, 10)

    def frame():
        img = p.getCameraImage(ANIM_W, ANIM_H, view, proj,
                               renderer=p.ER_TINY_RENDERER)
        rgb = np.reshape(img[2], (ANIM_H, ANIM_W, 4))[:, :, :3].astype(np.uint8)
        return Image.fromarray(rgb)

    frames = []
    for t in reach_pts:
        sol = solve_pick(rid, world["bin_id"], world["gripper_id"], world["ee"],
                         world["joints"], world["lo"], world["hi"], t,
                         world["orientations"], world["seeds"])
        if sol is None:                      # consistent with the mask; shouldn't fire
            continue
        for j, q in zip(world["joints"], sol["joints"]):
            p.resetJointState(rid, j, q)
        if world["gripper_id"] is not None:
            place_gripper(world["gripper_id"], np.array(sol["flange_pos"]),
                          sol["tool_orn"])
        frames.append(frame())
    path = os.path.join(RUN_DIR, "best_pick_cycle.gif")
    if not frames:                           # no reachable picks -> still emit a frame
        frames = [frame()]
    frames[0].save(path, save_all=True, append_images=frames[1:],
                   duration=int(1000 / max(ANIM_FPS, 1)), loop=0)
    return path

HUD_COLOR = [0.15, 1.0, 0.35]   # on-screen text color
HUD_SIZE  = 1.3

def _hud_anchor(i):
    """World position of HUD line i: stacked downward above the bin opening."""
    return [BIN_CENTER_XY[0] - BIN_L / 2,
            BIN_CENTER_XY[1] + BIN_W / 2,
            BIN_DEPTH + 0.45 - i * 0.09]

def hud_create(n=6):
    """Create n stacked, camera-facing text lines floating above the bin."""
    return [p.addUserDebugText(" ", _hud_anchor(i), textColorRGB=HUD_COLOR,
                               textSize=HUD_SIZE) for i in range(n)]

def hud_update(ids, lines):
    """Replace the HUD text in place (no flicker, no leaked debug items)."""
    if not ids:
        return
    for i, txt in enumerate(lines[:len(ids)]):
        ids[i] = p.addUserDebugText(txt, _hud_anchor(i), textColorRGB=HUD_COLOR,
                                    textSize=HUD_SIZE, replaceItemUniqueId=ids[i])

def pose_at(rid, ee, joints, lo, hi, target, orientations, gripper_id=None):
    """Drive the arm so the suction face lands on `target` (tool pointing down) and
    place the gripper body if one is given. Returns True if the face reached within
    FK_TOL. The flange is commanded a stand-off above the contact point."""
    target = np.asarray(target, dtype=float)
    standoff = GRIPPER_STANDOFF if USE_GRIPPER else 0.0
    flange_goal = (target + np.array([0.0, 0.0, standoff])).tolist()
    for ori in orientations:
        sol = p.calculateInverseKinematics(
            rid, ee, flange_goal, ori,
            lowerLimits=lo, upperLimits=hi,
            jointRanges=[h - l for l, h in zip(lo, hi)],
            restPoses=[(l + h) / 2 for l, h in zip(lo, hi)],
            maxNumIterations=200, residualThreshold=1e-4)
        for j, q in zip(joints, sol):
            p.resetJointState(rid, j, q)
        flange_pos = np.array(
            p.getLinkState(rid, ee, computeForwardKinematics=True)[4])
        if gripper_id is not None:
            # tool assumed exactly level -> place the plate at the commanded
            # (perfectly horizontal) orientation, not the solver's residual tilt
            place_gripper(gripper_id, flange_pos, ori)
        suction = flange_pos - np.array([0.0, 0.0, standoff])
        if np.linalg.norm(suction - target) <= FK_TOL:
            return True
    return False

def visualize_best(rid, ee, joints, lo, hi, targets, orientations, best, hud=None,
                   gripper_id=None):
    """At the best base, mark every target (green=reachable, red=not) and
    drive the arm through the reachable points so you can watch it work."""
    set_base(rid, best["xy"], best["z"], best["yaw"])
    reach_pts = targets[best["mask"]]
    miss_pts  = targets[~best["mask"]]
    if len(reach_pts):
        p.addUserDebugPoints(reach_pts.tolist(),
                             [[0, 1, 0]] * len(reach_pts), pointSize=6)
    if len(miss_pts):
        p.addUserDebugPoints(miss_pts.tolist(),
                             [[1, 0, 0]] * len(miss_pts), pointSize=6)
    n = len(reach_pts)
    print(f"\nGUI: arm parked at best base, cycling {n} reachable "
          f"points. Close the window or press Ctrl-C to quit.")
    try:
        while p.isConnected():
            for i, t in enumerate(reach_pts):
                pose_at(rid, ee, joints, lo, hi, t, orientations, gripper_id)
                hud_update(hud, [
                    "FR20 bin reachability  -  BEST BASE",
                    f"base   x={best['xy'][0]:+.2f}  y={best['xy'][1]:+.2f}  "
                    f"z={best['z']:.2f} m  yaw={math.degrees(best['yaw']):+.0f}deg",
                    f"coverage:   {best['cov']*100:4.1f}%",
                    f"reachable picks:   {n}/{len(targets)}",
                    f"picking target   {i+1}/{n}",
                    "close window or press Ctrl-C to quit",
                ])
                time.sleep(0.4)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    run()

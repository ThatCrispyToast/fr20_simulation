"""
Overhead-arm bin reachability feasibility study (PyBullet).

What it does
------------
- Builds a deep bin (floor + 4 walls) from box collision shapes.
- Mounts a robot arm overhead, hanging down over the bin.
- Sweeps a grid of base XY positions (and you can add height).
- For each base position, samples a 3D grid of target points inside the bin,
  solves IK with the tool pointing down, and counts a target as REACHABLE only if:
      (1) IK returns a solution,
      (2) forward kinematics confirms the tool actually reaches it (FK error < tol),
      (3) no robot link penetrates the bin walls/floor,
      (4) the solution is within joint limits.
- Reports the best base position by coverage and saves two heatmaps:
      coverage_vs_base.png   -> coverage % across the base-position grid
      best_pos_slices.png    -> which target points are reachable, layer by layer

Swap in your own robot
----------------------
Set ROBOT_URDF to your arm's URDF and adjust EE_LINK (or leave None to auto-pick
the last link). Set the bin dimensions and the base sweep to match your cell.
Everything you'll touch is in the CONFIG block.
"""

import os
import math
import time
import numpy as np
import pybullet as p
import pybullet_data
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
OUT_DIR   = os.path.join(_HERE, "..", "out")  # heatmap images are written here

# Bin geometry (meters). Interior floor sits at z=0, opening at z=BIN_DEPTH.
# Pallet from resources/bin_info.md: 1092.2 x 1219.2 x 838.2 mm.
BIN_L, BIN_W, BIN_DEPTH = 1.0922, 1.2192, 0.8382   # length(x), width(y), depth(z)
WALL_T = 0.02                                       # wall thickness
BIN_CENTER_XY = (0.0, 0.0)                          # bin center in world XY

# Overhead mount: base hangs upside-down at this height above the bin opening.
# FR20 reach ~1.7 m; base must clear the 0.838 m opening yet still touch the floor.
MOUNT_HEIGHT = 1.35      # world z of the robot base
FLIP_BASE = True         # rotate base 180deg about X so the arm hangs down

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

# Target sampling inside the bin
N_X, N_Y, N_Z = 6, 6, 4          # grid resolution of pick targets
MARGIN = 0.06                    # keep targets this far inside the walls/floor

# Tolerances
FK_TOL        = 0.02     # m, how close the tool must actually get to the target
COLLIDE_TOL   = 0.002    # m, penetration depth that counts as a collision
TILT_CONE_DEG = 15       # 0 = strictly tool-down; >0 lets it try angled approaches

# IK is a numerical solver that returns ONE config per call; a 6-DOF arm usually
# has several (elbow up/down, wrist flips). Try multiple seeds per pose and accept
# the target if ANY collision-free, in-limits config reaches it. More seeds = fewer
# false negatives, but proportionally slower.
N_IK_SEEDS = 8           # random seed configs tried per pose (plus the mid-range one)

# ----------------------------------------------------------------------------
# Setup
# ----------------------------------------------------------------------------
def connect():
    cid = p.connect(p.GUI if SHOW_SIM else p.DIRECT)
    # Resolve the robot's package:// mesh paths relative to the URDF directory.
    p.setAdditionalSearchPath(os.path.dirname(os.path.abspath(ROBOT_URDF)))
    if SHOW_SIM:
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
    rid = p.loadURDF(ROBOT_URDF, pos, base_orientation(yaw), useFixedBase=FIXED_BASE)
    return rid

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
    """Tool z-axis pointing down, optionally with a few tilted alternatives."""
    base = p.getQuaternionFromEuler([math.pi, 0, 0])   # tool points -z
    if TILT_CONE_DEG <= 0:
        return [base]
    oris = [base]
    a = math.radians(TILT_CONE_DEG)
    for roll, pitch in [(a, 0), (-a, 0), (0, a), (0, -a)]:
        oris.append(p.getQuaternionFromEuler([math.pi + roll, pitch, 0]))
    return oris

def ik_seeds(lo, hi):
    """Initial joint guesses for the IK solver. The numerical solver converges
    to whichever solution is nearest its seed, so seeding from several postures
    is how we discover the multiple configs (elbow up/down, wrist flips) that
    reach the same TCP. First seed is the mid-range pose for determinism."""
    rng = np.random.default_rng(0)
    seeds = [[(l + h) / 2 for l, h in zip(lo, hi)]]
    for _ in range(max(N_IK_SEEDS, 0)):
        seeds.append([float(rng.uniform(l, h)) for l, h in zip(lo, hi)])
    return seeds

# ----------------------------------------------------------------------------
# Reachability test for one target
# ----------------------------------------------------------------------------
def reachable(rid, bin_id, ee_link, joints, lo, hi, target, orientations, seeds):
    """Reachable if ANY orientation has ANY collision-free, in-limits joint
    configuration whose tool actually lands on the target. Each seed is a
    different starting guess, so we explore the arm's multiple IK solutions
    instead of trusting the single config one IK call happens to return."""
    for ori in orientations:
        for seed in seeds:
            # seed both the solver's starting state and its null-space target
            for j, q in zip(joints, seed):
                p.resetJointState(rid, j, q)
            sol = p.calculateInverseKinematics(
                rid, ee_link, target, ori,
                lowerLimits=lo, upperLimits=hi,
                jointRanges=[h - l for l, h in zip(lo, hi)],
                restPoses=seed,
                maxNumIterations=200, residualThreshold=1e-4,
            )
            # apply solution
            for j, q in zip(joints, sol):
                p.resetJointState(rid, j, q)
            # (1) FK check: did the tool actually reach?
            ee_pos = p.getLinkState(rid, ee_link, computeForwardKinematics=True)[4]
            if np.linalg.norm(np.array(ee_pos) - np.array(target)) > FK_TOL:
                continue
            # (2) joint-limit check
            if any(q < l - 1e-3 or q > h + 1e-3 for q, l, h in zip(sol, lo, hi)):
                continue
            # (3) collision check vs the bin
            p.performCollisionDetection()
            pts = p.getClosestPoints(rid, bin_id, distance=0.0)
            penetration = min([c[8] for c in pts], default=1.0)  # c[8]=contactDistance
            if penetration < -COLLIDE_TOL:
                continue
            return True
    return False

# ----------------------------------------------------------------------------
# Main sweep
# ----------------------------------------------------------------------------
def set_base(rid, base_xy, height, yaw=0.0):
    """Move the (fixed) robot base to a new mount point and heading."""
    cx, cy = BIN_CENTER_XY
    p.resetBasePositionAndOrientation(
        rid, [cx + base_xy[0], cy + base_xy[1], height], base_orientation(yaw))

def run():
    connect()
    bin_id = make_bin()
    targets, xs, ys, zs = bin_targets()
    orientations = down_orientations()

    # Load the robot once; the FR20 meshes are heavy, so we relocate the base
    # for each sweep point instead of reloading the URDF every time.
    rid = load_robot((0.0, 0.0), MOUNT_HEIGHT)
    joints = movable_joints(rid)
    lo, hi = joint_limits(rid, joints)
    ee = EE_LINK if EE_LINK is not None else p.getNumJoints(rid) - 1
    seeds = ik_seeds(lo, hi)

    # coverage[iz,iy,ix] holds the BEST coverage at that XY/height over all yaws,
    # so the heatmaps stay 2D-per-height; `best` tracks the winning yaw too.
    coverage = np.zeros((len(BASE_Z_RANGE), len(BASE_Y_RANGE), len(BASE_X_RANGE)))
    target_hits = np.zeros(len(targets))   # how many base poses reach each target
    best = {"cov": -1}

    hud = hud_create() if SHOW_SIM else None
    if SHOW_SIM:
        print("GUI: watch the arm scan each mount position; the full sweep takes "
              "a while, so use a smaller grid if you just want a quick look.")
    total = (len(BASE_YAW_RANGE) * len(BASE_Z_RANGE)
             * len(BASE_Y_RANGE) * len(BASE_X_RANGE))
    done, t0 = 0, time.time()
    for iyaw, byaw in enumerate(BASE_YAW_RANGE):
        for iz, bz in enumerate(BASE_Z_RANGE):
            for iy, by in enumerate(BASE_Y_RANGE):
                for ix, bx in enumerate(BASE_X_RANGE):
                    set_base(rid, (bx, by), bz, byaw)
                    mask = np.array([
                        reachable(rid, bin_id, ee, joints, lo, hi, t,
                                  orientations, seeds)
                        for t in targets
                    ])
                    cov = mask.mean()
                    coverage[iz, iy, ix] = max(coverage[iz, iy, ix], cov)
                    target_hits += mask
                    if cov > best["cov"]:
                        best = {"cov": cov, "xy": (bx, by), "z": bz,
                                "yaw": byaw, "mask": mask}
                    done += 1
                    progress_bar(done, total, t0)
                    if SHOW_SIM:
                        # settle the arm on a reachable pick so the scan is watchable
                        reach_pts = targets[mask]
                        if len(reach_pts):
                            pose_at(rid, ee, joints, lo, hi,
                                    reach_pts[len(reach_pts) // 2], orientations)
                        elapsed = time.time() - t0
                        eta = elapsed / done * (total - done)
                        hud_update(hud, [
                            "FR20 bin reachability  -  SWEEP",
                            f"base   x={bx:+.2f}  y={by:+.2f}  z={bz:.2f} m  "
                            f"yaw={math.degrees(byaw):+.0f}deg",
                            f"coverage here:   {cov*100:4.1f}%   "
                            f"({int(mask.sum())}/{len(targets)} picks)",
                            f"best so far:   {best['cov']*100:4.1f}%   "
                            f"@ x={best['xy'][0]:+.2f} y={best['xy'][1]:+.2f} "
                            f"z={best['z']:.2f} yaw={math.degrees(best['yaw']):+.0f}",
                            f"progress:   {done}/{total}  ({done/total*100:4.1f}%)",
                            f"elapsed {_fmt_mmss(elapsed)}    eta {_fmt_mmss(eta)}",
                        ])
                        time.sleep(SIM_DWELL)
    target_freq = target_hits / total      # fraction of bases reaching each target

    print(f"\nBest base offset: x={best['xy'][0]:+.3f} m, "
          f"y={best['xy'][1]:+.3f} m, z={best['z']:.3f} m, "
          f"yaw={math.degrees(best['yaw']):+.1f} deg  ->  "
          f"coverage {best['cov']*100:.1f}%")
    print(f"Heights tried: {[round(float(z), 3) for z in BASE_Z_RANGE]} m | "
          f"yaws tried: {[round(math.degrees(y), 1) for y in BASE_YAW_RANGE]} deg | "
          f"targets evaluated: {len(targets)}")

    # --- diagrams ---
    os.makedirs(OUT_DIR, exist_ok=True)
    saved = [
        plot_coverage_grid(coverage, best),
        plot_best_slices(best, xs, ys, zs),
        plot_reach_3d(best, targets),
        plot_coverage_vs_height(coverage),
        plot_target_frequency(target_freq, xs, ys, zs),
    ]
    print(f"Saved {len(saved)} diagrams to {OUT_DIR}:")
    for pth in saved:
        print(f"  - {os.path.basename(pth)}")

    # --- optional: watch the arm work the best base in the GUI ---
    if SHOW_SIM:
        visualize_best(rid, ee, joints, lo, hi, targets, orientations, best, hud)

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
    path = os.path.join(OUT_DIR, "coverage_vs_base.png")
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path

def plot_best_slices(best, xs, ys, zs):
    """Reachable pick points at the best base, sliced by depth, with the
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
        a.set_title(f"z = {zs[k]:.2f} m\n{n_ok}/{n_tot} reachable "
                    f"({mask[k].mean()*100:.0f}%)", fontsize=9)
        a.set_xlabel("x (m)")
        if k == 0:
            a.set_ylabel("y (m)")
    fig.suptitle(f"Reachable pick points by depth at best base  "
                 f"(x={best['xy'][0]:+.2f}, y={best['xy'][1]:+.2f}, "
                 f"z={best['z']:.2f} m, yaw={math.degrees(best['yaw']):+.0f}deg)   "
                 f"green = reachable, red = not", fontsize=11)
    fig.tight_layout()
    path = os.path.join(OUT_DIR, "best_pos_slices.png")
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
    """3D scatter of reachable vs unreachable pick points at the best base,
    with the bin drawn as a wireframe and the robot base marked."""
    reach = targets[best["mask"]]
    miss = targets[~best["mask"]]
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(projection="3d")
    _draw_bin_wireframe(ax)
    if len(miss):
        ax.scatter(miss[:, 0], miss[:, 1], miss[:, 2], c="red", marker="x",
                   s=30, alpha=0.6, label=f"unreachable ({len(miss)})")
    if len(reach):
        ax.scatter(reach[:, 0], reach[:, 1], reach[:, 2], c="green", marker="o",
                   s=35, alpha=0.9, label=f"reachable ({len(reach)})")
    ax.scatter([best["xy"][0]], [best["xy"][1]], [best["z"]], c="black",
               marker="^", s=130, label=f"robot base (z={best['z']:.2f} m)")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
    ax.set_title(f"Reachable pick points in 3D at best base  "
                 f"({best['cov']*100:.1f}% coverage)")
    ax.legend(loc="upper left", fontsize=8)
    ax.view_init(elev=22, azim=-60)
    path = os.path.join(OUT_DIR, "reach_3d.png")
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
    path = os.path.join(OUT_DIR, "coverage_vs_height.png")
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
    fig.suptitle("Target reachability across ALL base positions "
                 "(% of swept bases that can reach each point)", fontsize=11)
    fig.colorbar(im, ax=axes.ravel().tolist(), label="% of bases")
    path = os.path.join(OUT_DIR, "target_reachability.png")
    fig.savefig(path, dpi=130)
    plt.close(fig)
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

def pose_at(rid, ee, joints, lo, hi, target, orientations):
    """Drive the arm to `target` with the tool pointing down. Returns True if
    forward kinematics confirms it reached (within FK_TOL)."""
    target = list(target)
    for ori in orientations:
        sol = p.calculateInverseKinematics(
            rid, ee, target, ori,
            lowerLimits=lo, upperLimits=hi,
            jointRanges=[h - l for l, h in zip(lo, hi)],
            restPoses=[(l + h) / 2 for l, h in zip(lo, hi)],
            maxNumIterations=200, residualThreshold=1e-4)
        for j, q in zip(joints, sol):
            p.resetJointState(rid, j, q)
        ee_pos = p.getLinkState(rid, ee, computeForwardKinematics=True)[4]
        if np.linalg.norm(np.array(ee_pos) - np.array(target)) <= FK_TOL:
            return True
    return False

def visualize_best(rid, ee, joints, lo, hi, targets, orientations, best, hud=None):
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
                pose_at(rid, ee, joints, lo, hi, t, orientations)
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

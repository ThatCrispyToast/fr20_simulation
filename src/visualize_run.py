#!/usr/bin/env python
"""
Interactive 3D web view of the most recent bin_reach run.

Reads a run's data bundle (out/run_<timestamp>/best_versions.{json,npz}) and writes
a single self-contained HTML file you can orbit / zoom / pan in the browser. It shows
the bin, the pick points (covered vs not covered), the real plate placements, and the
robot base -- with a dropdown to flip between the top-N and the diverse base poses.

This is a *separate* viewer: it consumes bin_reach.py's output, it does not re-run the
simulation (no PyBullet needed).

Usage
-----
    uv run python src/visualize_run.py                 # most recent out/run_*/
    uv run python src/visualize_run.py out/run_2026... # a specific run
    uv run python src/visualize_run.py --no-open       # write HTML, don't open browser
"""
import os
import sys
import glob
import json
import argparse
import webbrowser
import numpy as np
import plotly.graph_objects as go

_HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(_HERE, "..", "out")

# point styles
NOT_COVERED = dict(size=2.0, color="#d62728", opacity=0.22, symbol="x")
COVERED     = dict(size=3.2, color="#2ca02c", opacity=0.85)
PLACEMENT   = dict(size=4.5, color="#1f77b4", symbol="diamond", opacity=0.95)
BASE        = dict(size=7.0, color="black", symbol="square")


def latest_run(arg):
    """The run dir to view: the CLI arg, else the newest out/run_*/."""
    if arg:
        if not os.path.isdir(arg):
            sys.exit(f"Not a directory: {arg}")
        return arg
    runs = sorted(glob.glob(os.path.join(OUT_DIR, "run_*")))
    if not runs:
        sys.exit("No out/run_*/ found -- run src/bin_reach.py first.")
    return runs[-1]


def load_run(run_dir):
    jpath = os.path.join(run_dir, "best_versions.json")
    npath = os.path.join(run_dir, "best_versions.npz")
    for pth in (jpath, npath):
        if not os.path.exists(pth):
            sys.exit(f"Missing {os.path.basename(pth)} in {run_dir} "
                     "(was the run saved with the data dump enabled?)")
    with open(jpath) as f:
        meta = json.load(f)
    return meta, np.load(npath)


def _bin_edge_lines(cx, cy, L, W, D):
    """The 12 edges of the bin box as a single line trace (None-separated)."""
    x0, x1 = cx - L / 2, cx + L / 2
    y0, y1 = cy - W / 2, cy + W / 2
    z0, z1 = 0.0, D
    c = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
             (0, 4), (1, 5), (2, 6), (3, 7)]
    xs, ys, zs = [], [], []
    for a, b in edges:
        xs += [c[a][0], c[b][0], None]
        ys += [c[a][1], c[b][1], None]
        zs += [c[a][2], c[b][2], None]
    return xs, ys, zs


def _poses(meta, npz):
    """(label, base-dict, npz-key-prefix) for every saved pose that has masks."""
    out = []
    for r, b in enumerate(meta.get("top_bests", []), 1):
        key = f"best{r}"
        if f"{key}_covered_mask" in npz.files:
            out.append((f"Top #{r}", b, key))
    for r, b in enumerate(meta.get("diverse_bests", []), 1):
        key = f"diverse{r}"
        if f"{key}_covered_mask" in npz.files:
            out.append((f"Diverse #{r}", b, key))
    return out


def _scatter(pts, name, marker, **kw):
    return go.Scatter3d(x=pts[:, 0], y=pts[:, 1], z=pts[:, 2], mode="markers",
                        marker=marker, name=name, visible=False, **kw)


def build_figure(run_dir, meta, npz):
    targets = npz["targets_m"]
    cx, cy = float(targets[:, 0].mean()), float(targets[:, 1].mean())  # ~ bin center
    L, W, D = meta["config"]["bin_LxWxD_m"]

    fig = go.Figure()
    # --- always-on bin geometry ---
    ex, ey, ez = _bin_edge_lines(cx, cy, L, W, D)
    fig.add_trace(go.Scatter3d(x=ex, y=ey, z=ez, mode="lines",
                               line=dict(color="#555", width=4), name="bin",
                               hoverinfo="skip"))
    fig.add_trace(go.Mesh3d(                                   # faint floor
        x=[cx - L / 2, cx + L / 2, cx + L / 2, cx - L / 2],
        y=[cy - W / 2, cy - W / 2, cy + W / 2, cy + W / 2],
        z=[0, 0, 0, 0], i=[0, 0], j=[1, 2], k=[2, 3],
        color="#cccccc", opacity=0.15, name="floor", hoverinfo="skip"))
    n_always = len(fig.data)

    poses = _poses(meta, npz)
    if not poses:
        sys.exit("No pose masks in the bundle -- nothing to show.")
    ranges = []
    for label, b, key in poses:
        cov = npz[f"{key}_covered_mask"].astype(bool)
        cen = npz[f"{key}_center_mask"].astype(bool)
        bx, by = cx + b["base"]["x"], cy + b["base"]["y"]
        bz = b["base"]["z"]
        start = len(fig.data)
        fig.add_trace(_scatter(targets[~cov], "not covered", NOT_COVERED,
                               hovertemplate="not covered<extra></extra>"))
        fig.add_trace(_scatter(targets[cov], "covered (foam reaches)", COVERED,
                               hovertemplate="covered<br>(%{x:.2f}, %{y:.2f}, %{z:.2f})"
                                             "<extra></extra>"))
        fig.add_trace(_scatter(targets[cen], "plate placement (centered)", PLACEMENT,
                               hovertemplate="placement center<extra></extra>"))
        fig.add_trace(go.Scatter3d(
            x=[bx], y=[by], z=[bz], mode="markers+text", marker=BASE,
            text=[f"  base ({b['coverage_pct']:.0f}%)"], textposition="middle right",
            name="robot base", visible=False,
            hovertemplate=f"robot base<br>x={b['base']['x']:.2f} y={b['base']['y']:.2f} "
                          f"z={b['base']['z']:.2f} m<br>yaw={b['base']['yaw_deg']:.0f}"
                          "deg<extra></extra>"))
        ranges.append((start, len(fig.data)))

    total = len(fig.data)

    def _title(label, b):
        return (f"{os.path.basename(run_dir)} &nbsp;|&nbsp; <b>{label}</b> &nbsp;—&nbsp; "
                f"coverage {b['coverage_pct']:.1f}%  "
                f"({b['covered']}/{b['total']} covered, {b['placements']} placements)  "
                f"@ base x={b['base']['x']:+.2f} y={b['base']['y']:+.2f} "
                f"z={b['base']['z']:.2f} m  yaw={b['base']['yaw_deg']:+.0f}°")

    # default: first pose visible
    for t in range(*ranges[0]):
        fig.data[t].visible = True

    buttons = []
    for i, (label, b, key) in enumerate(poses):
        vis = [False] * total
        for t in range(n_always):
            vis[t] = True
        for t in range(*ranges[i]):
            vis[t] = True
        buttons.append(dict(label=label, method="update",
                            args=[{"visible": vis}, {"title.text": _title(label, b)}]))

    fig.update_layout(
        title=dict(text=_title(poses[0][0], poses[0][1]), x=0.5, xanchor="center"),
        updatemenus=[dict(buttons=buttons, direction="down", showactive=True,
                          x=0.01, y=0.99, xanchor="left", yanchor="top",
                          bgcolor="#f2f2f2")],
        scene=dict(aspectmode="data", xaxis_title="x (m)", yaxis_title="y (m)",
                   zaxis_title="z (m)",
                   camera=dict(eye=dict(x=1.6, y=-1.6, z=1.2))),
        legend=dict(x=0.99, y=0.99, xanchor="right", yanchor="top"),
        margin=dict(l=0, r=0, t=60, b=0))
    return fig


def main():
    ap = argparse.ArgumentParser(description="Interactive 3D web view of a bin_reach run.")
    ap.add_argument("run_dir", nargs="?", help="run dir (default: most recent out/run_*/)")
    ap.add_argument("--no-open", action="store_true", help="write the HTML but don't open it")
    args = ap.parse_args()

    run_dir = latest_run(args.run_dir)
    meta, npz = load_run(run_dir)
    fig = build_figure(run_dir, meta, npz)

    out_html = os.path.join(run_dir, "view.html")
    fig.write_html(out_html, include_plotlyjs=True, full_html=True)
    print(f"Wrote {out_html}")
    if not args.no_open:
        webbrowser.open("file://" + os.path.abspath(out_html))
        print("Opened in your browser. Drag to orbit, scroll to zoom, "
              "use the dropdown to switch base poses.")


if __name__ == "__main__":
    main()

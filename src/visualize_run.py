#!/usr/bin/env python
"""
Interactive 3D web view of the most recent bin_reach run.

Reads a run's data bundle (out/run_<timestamp>/best_versions.{json,npz}) and writes
a single self-contained HTML file you can orbit / zoom / pan in the browser. It shows
the bin, the pick points (covered vs not covered), the real plate placements, the
robot base, AND the FR20 arm posed at a representative placement -- with a dropdown to
flip between the top-N and the diverse base poses (the arm moves with the selection).

This is a *separate* viewer: it consumes bin_reach.py's output. It does not re-run the
sweep; it only loads the URDF/meshes via PyBullet to pose the arm for display.

Usage
-----
    uv run python src/visualize_run.py                 # most recent out/run_*/
    uv run python src/visualize_run.py out/run_2026... # a specific run
    uv run python src/visualize_run.py --no-open       # write HTML, don't open browser
    uv run python src/visualize_run.py --no-arm        # skip the arm meshes (smaller)
    uv run python src/visualize_run.py --arm-detail 64 # arm mesh resolution (default 48)
"""
import os
import sys
import glob
import json
import math
import struct
import argparse
import webbrowser
import numpy as np
import plotly.graph_objects as go

_HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(_HERE, "..", "out")
sys.path.insert(0, _HERE)   # so `import bin_reach` works when run as a script

# point styles
NOT_COVERED = dict(size=2.0, color="#d62728", opacity=0.22, symbol="x")
COVERED     = dict(size=3.2, color="#2ca02c", opacity=0.85)
PLACEMENT   = dict(size=4.5, color="#1f77b4", symbol="diamond", opacity=0.95)
BASE        = dict(size=7.0, color="black", symbol="square")
ARM_COLOR   = "#9aa7b4"


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


# --- arm rendering ----------------------------------------------------------
def _load_stl(path):
    """Vertices of a binary STL as an (n_tri, 3, 3) array (the Fairino meshes)."""
    with open(path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        arr = np.frombuffer(f.read(50 * n), dtype="<u1").reshape(n, 50)
    floats = arr[:, :48].copy().view("<f4").reshape(n, 4, 3)   # [normal, v0, v1, v2]
    return floats[:, 1:4, :].astype(np.float64)


def _cluster_decimate(tris, res):
    """Dependency-free vertex-clustering decimation: snap vertices to a coarse grid,
    collapse each cell to its mean, drop degenerate faces. Cuts the triangle count a
    lot while keeping the silhouette and staying watertight (no stride holes)."""
    V = tris.reshape(-1, 3)
    mn = V.min(0)
    span = float((V.max(0) - mn).max()) or 1.0
    cell = span / max(res, 1)
    keys = np.floor((V - mn) / cell).astype(np.int64)
    _, inv = np.unique(keys, axis=0, return_inverse=True)
    ncell = int(inv.max()) + 1
    reps = np.zeros((ncell, 3))
    cnt = np.zeros(ncell)
    np.add.at(reps, inv, V)
    np.add.at(cnt, inv, 1)
    reps /= cnt[:, None]
    faces = inv.reshape(-1, 3)
    good = ((faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2])
            & (faces[:, 0] != faces[:, 2]))
    return reps, faces[good]


def _qmat(p, q):
    return np.array(p.getMatrixFromQuaternion(q)).reshape(3, 3)


def _parse_chain(p, urdf_path):
    """The revolute joints in chain order (j1..j6) as {pfp, pfo, axis}, taken from the
    URDF <origin>/<axis> -- the unambiguous source for FK (PyBullet's getJointInfo
    frames are relative to the parent's *inertial* frame, which doesn't chain cleanly).
    Reproduces getLinkState's link frames exactly when chained from the base pose."""
    import xml.etree.ElementTree as ET
    chain = []
    for jt in ET.parse(urdf_path).getroot().findall("joint"):
        if jt.get("type") not in ("revolute", "continuous"):
            continue
        o, a = jt.find("origin"), jt.find("axis")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0").split() if o is not None
                                  else ["0", "0", "0"])]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0").split() if o is not None
                                  else ["0", "0", "0"])]
        axis = [float(v) for v in (a.get("xyz", "0 0 1").split() if a is not None
                                   else ["0", "0", "1"])]
        chain.append({"pfp": xyz, "pfo": list(p.getQuaternionFromEuler(rpy)),
                      "axis": axis})
    return chain


def build_arm(res):
    """Load the FR20 once and pre-compute each visual mesh (decimated) expressed in
    its LINK frame, plus the kinematic chain, so the arm can be re-posed at any joint
    config (in Python for the embedded views, in JS for click-to-pose). Returns a dict,
    or None if PyBullet / the robot model isn't available."""
    try:
        import pybullet as p
        import bin_reach as br
    except Exception as e:                       # noqa: BLE001
        print(f"(arm disabled: {e})")
        return None
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(os.path.dirname(os.path.abspath(br.ROBOT_URDF)))
    rid = p.loadURDF(br.ROBOT_URDF, [0, 0, 0], useFixedBase=True)
    joints = br.movable_joints(rid)
    chain = _parse_chain(p, br.ROBOT_URDF)
    links = []
    for vs in p.getVisualShapeData(rid):
        li, scale = vs[1], np.array(vs[3])
        mesh = vs[4].decode() if isinstance(vs[4], bytes) else vs[4]
        if not mesh or not os.path.exists(mesh):
            continue
        verts, faces = _cluster_decimate(_load_stl(mesh) * scale, res)
        verts = verts @ _qmat(p, vs[6]).T + np.array(vs[5])   # visual frame -> link frame
        links.append((li, verts, faces))
    # resetBasePositionAndOrientation places the base COM, so the URDF base frame the
    # link transforms chain from is offset from the mount point. Measure that constant
    # offset (reset to identity, zero joints -> link0 frame == base frame composed with
    # j1's origin) so FK can start from the true URDF base frame.
    p.resetBasePositionAndOrientation(rid, [0, 0, 0], [0, 0, 0, 1])
    for j in joints:
        p.resetJointState(rid, j, 0.0)
    ls0 = p.getLinkState(rid, 0, computeForwardKinematics=True)
    inv_j1 = p.invertTransform(chain[0]["pfp"], chain[0]["pfo"])
    toff = p.multiplyTransforms(ls0[4], ls0[5], inv_j1[0], inv_j1[1])
    return {"p": p, "rid": rid, "joints": joints, "links": links, "br": br,
            "chain": chain, "toff": (list(toff[0]), list(toff[1]))}


def arm_traces(arm, mount_pos, mount_orn, urdf_base, jcfg):
    """Mesh3d traces for the arm at one mount pose + joint config (hidden by default).
    The base reset uses the mount pose (matching the sim); the base-link mesh sits at the
    true URDF base frame; the moving links come from getLinkState -- the exact frames the
    JS click handler reproduces by chaining from `urdf_base`."""
    p, rid = arm["p"], arm["rid"]
    p.resetBasePositionAndOrientation(rid, list(mount_pos), mount_orn)
    for j, q in zip(arm["joints"], jcfg):
        p.resetJointState(rid, j, q)
    traces = []
    for li, verts, faces in arm["links"]:
        if li == -1:
            wp, wo = urdf_base                   # true URDF base frame (not mount/COM)
        else:
            st = p.getLinkState(rid, li, computeForwardKinematics=True)
            wp, wo = st[4], st[5]
        vw = verts @ _qmat(p, wo).T + np.array(wp)
        traces.append(go.Mesh3d(
            x=vw[:, 0], y=vw[:, 1], z=vw[:, 2],
            i=faces[:, 0], j=faces[:, 1], k=faces[:, 2],
            color=ARM_COLOR, opacity=1.0, flatshading=True, name="FR20 arm",
            hoverinfo="skip", visible=False, showlegend=(li == -1),
            lighting=dict(ambient=0.55, diffuse=0.6, specular=0.1)))
    return traces


def _poses(meta, npz):
    """(label, base-dict, npz-key-prefix) for every saved pose that has masks."""
    out = []
    for r, b in enumerate(meta.get("top_bests", []), 1):
        if f"best{r}_covered_mask" in npz.files:
            out.append((f"Top #{r}", b, f"best{r}"))
    for r, b in enumerate(meta.get("diverse_bests", []), 1):
        if f"diverse{r}_covered_mask" in npz.files:
            out.append((f"Diverse #{r}", b, f"diverse{r}"))
    return out


def _scatter(pts, name, marker, **kw):
    return go.Scatter3d(x=pts[:, 0], y=pts[:, 1], z=pts[:, 2], mode="markers",
                        marker=marker, name=name, visible=False, **kw)


def _representative_cfg(npz, key, targets, center):
    """Joint config of the placement nearest the bin center (a natural central reach)."""
    cen = npz[f"{key}_center_mask"].astype(bool)
    J = npz.get(f"{key}_joints_rad")
    idxs = np.where(cen)[0]
    if J is None or len(idxs) == 0:
        return None
    d = np.linalg.norm(targets[idxs, :2] - center, axis=1)
    return J[idxs[int(d.argmin())]]


def build_figure(run_dir, meta, npz, arm=None):
    targets = npz["targets_m"]
    L, W, D = meta["config"]["bin_LxWxD_m"]
    if arm is not None:
        cx, cy = arm["br"].BIN_CENTER_XY         # match the sim's bin center exactly
    else:
        cx, cy = float(targets[:, 0].mean()), float(targets[:, 1].mean())

    fig = go.Figure()
    ex, ey, ez = _bin_edge_lines(cx, cy, L, W, D)
    fig.add_trace(go.Scatter3d(x=ex, y=ey, z=ez, mode="lines",
                               line=dict(color="#555", width=4), name="bin",
                               hoverinfo="skip"))
    fig.add_trace(go.Mesh3d(
        x=[cx - L / 2, cx + L / 2, cx + L / 2, cx - L / 2],
        y=[cy - W / 2, cy - W / 2, cy + W / 2, cy + W / 2],
        z=[0, 0, 0, 0], i=[0, 0], j=[1, 2], k=[2, 3],
        color="#cccccc", opacity=0.15, name="floor", hoverinfo="skip"))
    sel_idx = len(fig.data)                       # marker for the clicked point
    fig.add_trace(go.Scatter3d(x=[], y=[], z=[], mode="markers", name="selected pick",
                               marker=dict(size=9, color="#ff00ff", symbol="circle",
                                           line=dict(color="black", width=1)),
                               hoverinfo="skip", showlegend=False))
    n_always = len(fig.data)

    poses = _poses(meta, npz)
    if not poses:
        sys.exit("No pose masks in the bundle -- nothing to show.")
    arm_js = None if arm is None else {
        "div": "binview", "sel": sel_idx, "chain": arm["chain"],
        "linkidx": [int(li) for li, _, _ in arm["links"]],
        "vlink": [[round(float(v), 4) for v in vt.reshape(-1)]
                  for _, vt, _ in arm["links"]],
        "poses": [], "click2pose": {}, "allArm": []}
    ranges = []
    for pi, (label, b, key) in enumerate(poses):
        cov = npz[f"{key}_covered_mask"].astype(bool)
        cen = npz[f"{key}_center_mask"].astype(bool)
        bx, by, bz = cx + b["base"]["x"], cy + b["base"]["y"], b["base"]["z"]
        start = len(fig.data)
        fig.add_trace(_scatter(targets[~cov], "not covered", NOT_COVERED,
                               hovertemplate="not covered — click for nearest reach"
                                             "<extra></extra>"))
        fig.add_trace(_scatter(targets[cov], "covered (foam reaches)", COVERED,
                               hovertemplate="covered<br>(%{x:.2f}, %{y:.2f}, %{z:.2f})"
                                             "<br>click to pose the arm<extra></extra>"))
        fig.add_trace(_scatter(targets[cen], "plate placement (centered)", PLACEMENT,
                               hovertemplate="placement center<br>click to pose the arm"
                                             "<extra></extra>"))
        click_traces = list(range(start, start + 3))     # the three point traces
        fig.add_trace(go.Scatter3d(
            x=[bx], y=[by], z=[bz], mode="markers", marker=BASE, name="robot base",
            visible=False,
            hovertemplate=f"robot base<br>x={b['base']['x']:.2f} y={b['base']['y']:.2f} "
                          f"z={b['base']['z']:.2f} m<br>yaw={b['base']['yaw_deg']:.0f}"
                          "deg<extra></extra>"))
        if arm is not None:
            yaw = math.radians(b["base"]["yaw_deg"])
            mount_q = list(arm["br"].base_orientation(yaw))
            ub_p, ub_q = arm["p"].multiplyTransforms([bx, by, bz], mount_q,
                                                     arm["toff"][0], arm["toff"][1])
            jcfg = _representative_cfg(npz, key, targets, (cx, cy))
            arm_start = len(fig.data)
            if jcfg is not None:
                for t in arm_traces(arm, (bx, by, bz), mount_q, (ub_p, ub_q), jcfg):
                    fig.add_trace(t)
            J = npz.get(f"{key}_joints_rad")
            placements = [{"p": [round(float(x), 4) for x in targets[k]],
                           "q": [round(float(x), 5) for x in J[k]]}
                          for k in np.where(cen)[0]] if J is not None else []
            arm_t = list(range(arm_start, len(fig.data)))
            arm_js["poses"].append({
                "base_pos": list(ub_p), "base_quat": list(ub_q),   # true URDF base frame
                "placements": placements, "armTraces": arm_t,
                "probe": start + 1})              # covered trace: visible iff pose active
            arm_js["allArm"].extend(arm_t)
            for ti in click_traces:
                arm_js["click2pose"][str(ti)] = pi
        ranges.append((start, len(fig.data)))

    total = len(fig.data)

    def _title(label, b):
        return (f"{os.path.basename(run_dir)} &nbsp;|&nbsp; <b>{label}</b> &nbsp;—&nbsp; "
                f"coverage {b['coverage_pct']:.1f}%  "
                f"({b['covered']}/{b['total']} covered, {b['placements']} placements)  "
                f"@ base x={b['base']['x']:+.2f} y={b['base']['y']:+.2f} "
                f"z={b['base']['z']:.2f} m  yaw={b['base']['yaw_deg']:+.0f}°")

    for t in range(*ranges[0]):                  # first pose visible by default
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
                   zaxis_title="z (m)", camera=dict(eye=dict(x=1.7, y=-1.7, z=1.3))),
        legend=dict(x=0.99, y=0.99, xanchor="right", yanchor="top"),
        margin=dict(l=0, r=0, t=60, b=0))
    return fig, arm_js


# Client-side FK: on click, recompute each link's world transform from the clicked
# placement's joint angles (chaining the URDF joint origins from the base pose -- the
# exact same math verified against PyBullet's getLinkState) and restyle the arm meshes.
_CLICK_JS = r"""
<script>
var ARM = __ARM_JSON__;
(function(){
  function qmul(a,b){return [
    a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1],
    a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0],
    a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3],
    a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]];}
  function qrot(q,v){var x=q[0],y=q[1],z=q[2],w=q[3],vx=v[0],vy=v[1],vz=v[2];
    var tx=2*(y*vz-z*vy), ty=2*(z*vx-x*vz), tz=2*(x*vy-y*vx);
    return [vx+w*tx+(y*tz-z*ty), vy+w*ty+(z*tx-x*tz), vz+w*tz+(x*ty-y*tx)];}
  function comp(pa,qa,pb,qb){var r=qrot(qa,pb);
    return [[pa[0]+r[0],pa[1]+r[1],pa[2]+r[2]], qmul(qa,qb)];}
  function axang(ax,th){var n=Math.hypot(ax[0],ax[1],ax[2])||1, s=Math.sin(th/2)/n;
    return [ax[0]*s,ax[1]*s,ax[2]*s,Math.cos(th/2)];}
  function fk(bp,bq,q){var cp=bp.slice(),cq=bq.slice(),fr=[],r;
    for(var j=0;j<ARM.chain.length;j++){var c=ARM.chain[j];
      r=comp(cp,cq,c.pfp,c.pfo); cp=r[0]; cq=r[1];
      r=comp(cp,cq,[0,0,0],axang(c.axis,q[j])); cp=r[0]; cq=r[1];
      fr.push([cp.slice(),cq.slice()]);}
    return fr;}
  function xform(vf,fr){var p=fr[0],q=fr[1],n=vf.length,m=n/3;
    var X=new Array(m),Y=new Array(m),Z=new Array(m);
    for(var i=0,k=0;i<n;i+=3,k++){var r=qrot(q,[vf[i],vf[i+1],vf[i+2]]);
      X[k]=r[0]+p[0]; Y[k]=r[1]+p[1]; Z[k]=r[2]+p[2];}
    return [X,Y,Z];}
  var SHOW_ARM=true, guard=false, chk=null;
  function poseArm(gd,pi,q,sp){
    var P=ARM.poses[pi], fr=fk(P.base_pos,P.base_quat,q);
    var xs=[],ys=[],zs=[],idx=[];
    for(var k=0;k<ARM.vlink.length;k++){
      var li=ARM.linkidx[k], frame=(li<0)?[P.base_pos,P.base_quat]:fr[li];
      var t=xform(ARM.vlink[k],frame);
      xs.push(t[0]); ys.push(t[1]); zs.push(t[2]); idx.push(P.armTraces[k]);}
    guard=true;
    Plotly.restyle(gd,{x:xs,y:ys,z:zs},idx).then(function(){guard=false;});
    Plotly.restyle(gd,{x:[[sp[0]]],y:[[sp[1]]],z:[[sp[2]]]},[ARM.sel]);
  }
  function activePose(gd){
    for(var i=0;i<ARM.poses.length;i++)
      if(gd.data[ARM.poses[i].probe].visible!==false) return i;
    return 0;
  }
  function applyArm(gd){            // arm visible only for the active pose, and only if shown
    guard=true;
    var i=activePose(gd), on=SHOW_ARM?ARM.poses[i].armTraces:[];
    Plotly.restyle(gd,{visible:false},ARM.allArm).then(function(){
      if(on.length) Plotly.restyle(gd,{visible:true},on).then(function(){guard=false;});
      else guard=false;
    });
  }
  function onClick(gd,ev){
    if(!ev.points||!ev.points.length) return;
    var pt=ev.points[0], pi=ARM.click2pose[pt.curveNumber];
    if(pi===undefined||pi===null) return;
    var P=ARM.poses[pi], best=-1, bd=1e18;
    for(var i=0;i<P.placements.length;i++){var p=P.placements[i].p;
      var d=(p[0]-pt.x)*(p[0]-pt.x)+(p[1]-pt.y)*(p[1]-pt.y)+(p[2]-pt.z)*(p[2]-pt.z);
      if(d<bd){bd=d; best=i;}}
    if(best<0) return;
    if(chk && !chk.checked){ chk.checked=true; SHOW_ARM=true; }   // clicking implies "show"
    poseArm(gd,pi,P.placements[best].q,P.placements[best].p);
    Plotly.restyle(gd,{visible:true},P.armTraces);
  }
  function makeToggle(gd){
    var d=document.createElement('div');
    d.style.cssText='position:absolute;top:10px;right:14px;z-index:1000;background:#f2f2f2;'+
      'padding:4px 9px;border-radius:4px;font-family:sans-serif;font-size:13px;'+
      'box-shadow:0 1px 3px rgba(0,0,0,.25)';
    d.innerHTML='<label style="cursor:pointer"><input type="checkbox" id="armChk" checked>'+
      ' show arm</label>';
    document.body.appendChild(d);
    chk=d.querySelector('#armChk');
    chk.addEventListener('change',function(){ SHOW_ARM=chk.checked; applyArm(gd); });
  }
  function ready(){
    var gd=document.getElementById(ARM.div);
    if(!gd||!gd.on){return setTimeout(ready,80);}
    makeToggle(gd);
    gd.on('plotly_click',function(ev){onClick(gd,ev);});
    // the pose dropdown re-shows that pose's arm; re-apply the toggle (e.g. keep it hidden)
    gd.on('plotly_restyle',function(){ if(!guard && !SHOW_ARM) applyArm(gd); });
  }
  ready();
})();
</script>
"""


def write_html(fig, arm_js, out_html):
    html = fig.to_html(include_plotlyjs=True, full_html=True, div_id="binview")
    if arm_js is not None:
        script = _CLICK_JS.replace("__ARM_JSON__", json.dumps(arm_js))
        html = html.replace("</body>", script + "\n</body>")
    with open(out_html, "w") as f:
        f.write(html)


def main():
    ap = argparse.ArgumentParser(description="Interactive 3D web view of a bin_reach run.")
    ap.add_argument("run_dir", nargs="?", help="run dir (default: most recent out/run_*/)")
    ap.add_argument("--no-open", action="store_true", help="write the HTML but don't open it")
    ap.add_argument("--no-arm", action="store_true", help="don't render the FR20 arm meshes")
    ap.add_argument("--arm-detail", type=int, default=48,
                    help="arm mesh resolution (vertex-cluster cells along longest axis)")
    args = ap.parse_args()

    run_dir = latest_run(args.run_dir)
    meta, npz = load_run(run_dir)
    arm = None if args.no_arm else build_arm(args.arm_detail)
    fig, arm_js = build_figure(run_dir, meta, npz, arm)

    out_html = os.path.join(run_dir, "view.html")
    write_html(fig, arm_js, out_html)
    print(f"Wrote {out_html}  ({os.path.getsize(out_html) // 1024} KB)")
    if not args.no_open:
        webbrowser.open("file://" + os.path.abspath(out_html))
        msg = ("Opened in your browser. Drag to orbit, scroll to zoom, dropdown to "
               "switch base poses.")
        if arm_js is not None:
            msg += " Click any pick point to pose the arm at the nearest placement."
        print(msg)


if __name__ == "__main__":
    main()

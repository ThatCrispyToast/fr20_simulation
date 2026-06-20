// FR20 bin-reachability 3D viewer.
//
// Renders the output of src/bin_reach.py (best_versions.json) in the browser:
// the bin, the articulated FR20 arm (loaded from the URDF + STL meshes via
// urdf-loader), the vacuum-gripper plate, and the swept pick targets coloured by
// outcome. You can pick any reported base pose (top-N or diverse), slice the
// target grid by depth, toggle layers, and play the arm through its real,
// collision-free PLACEMENTS (driven by the joint solutions in the JSON).
//
// Everything is kept in the simulator's native Z-up world frame so poses match
// the study exactly (no axis conversion): bin floor at z=0, opening at z=DEPTH,
// base at (x,y,z), base orientation = Rz(yaw)*Rx(pi) (the hang-down flip).

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import URDFLoader from 'urdf-loader';

// ---------------------------------------------------------------------------
// Where to find the data. The launcher passes ?run=out/run_xxx/best_versions.json
// (a directory is also accepted; best_versions.json is appended). Paths are
// relative to the static-server root (the repo root).
// ---------------------------------------------------------------------------
const params = new URLSearchParams(location.search);
let runArg = params.get('run');
const URDF_URL = params.get('urdf') || '/resources/fairino20_v6.urdf';
const PKG_ROOT = '/resources/fairino_description';   // package://fairino_description -> here
const JOINT_NAMES = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'];
const EE_LINK = 'wrist3_link';                        // flange (matches EE_LINK=None -> last link)
const WALL_T = 0.02;                                  // bin wall thickness (cosmetic)

const COL = { place: 0x1ae62a, cover: 0xffc83d, miss: 0xff4d4d, buried: 0x6b7280 };

// ---------------------------------------------------------------------------
// Scene scaffolding
// ---------------------------------------------------------------------------
const wrap = document.getElementById('canvas-wrap');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
wrap.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0c0e12);

const camera = new THREE.PerspectiveCamera(55, window.innerWidth / window.innerHeight, 0.05, 50);
camera.up.set(0, 0, 1);                               // Z-up, like the sim

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

scene.add(new THREE.HemisphereLight(0xffffff, 0x404048, 1.05));
const key = new THREE.DirectionalLight(0xffffff, 1.3);
key.position.set(2, -2.5, 4);
scene.add(key);

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

// ---------------------------------------------------------------------------
// Global state, filled once data + robot are loaded
// ---------------------------------------------------------------------------
let CFG = null;                 // config block from the JSON
let GRID = null;                // {nx, ny, nz}
let PKT = null;                 // packet dims {length_m, width_m, height_m}
let POINT_MODE = false;         // true = original point sim (no config.packet): draw spheres
let STEP_X = 1, STEP_Y = 1;     // packed-view stride (grid cells per packet) so boxes don't overlap
let poses = [];                 // [{label, ...best_block}]
let robot = null;               // URDFRobot
let gripper = null;             // THREE.Group (post + plate)
let targetMeshes = [];          // one sphere per pick, index-aligned to picks
let cursor = null;              // wireframe highlight on the active placement
let selMarker = null;           // wireframe highlight on the clicked/selected packet
let curPose = null;
let placementIdx = [];          // indices into picks where is_placement is true
let playStep = 0, playing = false, playAccum = 0, picksPerSec = 6;
let selIdx = -1, solList = [], solIdx = 0;   // click-to-inspect selection + its IK configs

// quaternion Rz(yaw)*Rx(pi): the hang-down flip clocked by `yaw` (radians).
// Matches pybullet getQuaternionFromEuler([pi, 0, yaw]) used for the base and,
// with yaw = tool clocking, for the level gripper plate.
function flipQuat(yaw) {
  const qx = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI);
  const qz = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), yaw);
  return qz.multiply(qx);
}

// ---------------------------------------------------------------------------
// Bin: translucent floor + 4 walls, mirroring make_bin() (floor at z=0).
// ---------------------------------------------------------------------------
let binGroup = null;
function buildBin() {
  if (binGroup) scene.remove(binGroup);
  const [L, W, D] = CFG.bin_LxWxD_m;
  binGroup = new THREE.Group();
  const wall = new THREE.MeshStandardMaterial({
    color: 0x9aa6bf, transparent: true, opacity: 0.18, side: THREE.DoubleSide,
    metalness: 0.0, roughness: 0.9 });
  const add = (sx, sy, sz, x, y, z) => {
    const m = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), wall);
    m.position.set(x, y, z);
    binGroup.add(m);
  };
  add(L, W, WALL_T, 0, 0, -WALL_T / 2);                       // floor
  add(WALL_T, W, D, L / 2 + WALL_T / 2, 0, D / 2);            // +x wall
  add(WALL_T, W, D, -L / 2 - WALL_T / 2, 0, D / 2);           // -x wall
  add(L + 2 * WALL_T, WALL_T, D, 0, W / 2 + WALL_T / 2, D / 2);  // +y wall
  add(L + 2 * WALL_T, WALL_T, D, 0, -W / 2 - WALL_T / 2, D / 2); // -y wall

  // crisp edge outline of the interior cavity
  const edges = new THREE.LineSegments(
    new THREE.EdgesGeometry(new THREE.BoxGeometry(L, W, D)),
    new THREE.LineBasicMaterial({ color: 0x6b7689 }));
  edges.position.set(0, 0, D / 2);
  binGroup.add(edges);
  scene.add(binGroup);

  controls.target.set(0, 0, D / 2);
  camera.position.set(L * 1.5, -W * 1.7, D + 1.6);
  controls.update();
}

// ---------------------------------------------------------------------------
// Gripper plate: thin post (top half of the stand-off) + the L x W plate
// (bottom half, down to the foam face) -- the same envelope as make_gripper().
// Local +z is the tool down-axis; the group is oriented level (Rx pi) so +z
// maps to world -z, and clocked by the winning TOOL_YAW.
// ---------------------------------------------------------------------------
function buildGripper() {
  if (gripper) scene.remove(gripper);
  if (!CFG.gripper.enabled) { gripper = null; return; }
  const g = CFG.gripper, h = g.standoff_m;
  gripper = new THREE.Group();

  const postGeo = new THREE.CylinderGeometry(g.post_radius_m, g.post_radius_m, h / 2, 16);
  postGeo.rotateX(Math.PI / 2);                       // cylinder axis Y -> local Z
  const post = new THREE.Mesh(postGeo, new THREE.MeshStandardMaterial({
    color: 0x9aa0aa, transparent: true, opacity: 0.7 }));
  post.position.set(0, 0, h / 4);

  const plate = new THREE.Mesh(
    new THREE.BoxGeometry(g.length_m, g.width_m, h / 2),
    new THREE.MeshStandardMaterial({ color: 0x2f7fff, transparent: true, opacity: 0.6 }));
  plate.position.set(0, 0, 3 * h / 4);

  gripper.add(post, plate);
  gripper.visible = false;
  scene.add(gripper);
}

// Place the gripper at the flange (read from FK) at the commanded level
// orientation clocked by toolYawDeg -- exactly what place_gripper() does.
const _flange = new THREE.Vector3();
function placeGripper(toolYawDeg) {
  if (!gripper || !robot) return;
  const link = robot.links[EE_LINK];
  link.getWorldPosition(_flange);
  gripper.position.copy(_flange);
  gripper.quaternion.copy(flipQuat(THREE.MathUtils.degToRad(toolYawDeg || 0)));
}

// ---------------------------------------------------------------------------
// Targets: one mesh per grid target (600), coloured by outcome. In the packet sim
// each is a real packet drawn as a BOX (top face at target z, the pick plane); in
// the original point sim each is a SPHERE at the point. Kept index-aligned to
// pose.picks so depth slicing and playback are simple lookups.
// ---------------------------------------------------------------------------
function classOf(pick) {
  if (pick.buried) return 'buried';                // packed sim: stacked below the top
  if (pick.is_placement) return 'place';
  // `pickable` (packet sim) or `covered` (point sim) -- whichever the JSON has.
  const ok = pick.pickable !== undefined ? pick.pickable : pick.covered;
  return ok ? 'cover' : 'miss';
}
function depthOf(i) { return Math.floor(i / (GRID.nx * GRID.ny)); }  // z-major order

// A packet's dimensions [l, w, h]: a run can carry a per-pick `size_m` (mixed packet
// sizes); otherwise every packet uses the run's nominal CFG.packet. So the viewer
// renders whatever size(s) the run actually used -- no hard-coded packet size.
function pickDims(pick) {
  return pick.size_m || [PKT.length_m, PKT.width_m, PKT.height_m];
}

// Geometry cache: one BoxGeometry per distinct size (so a uniform run makes one box,
// a mixed-size run makes a few) -- shared across poses, never disposed per mesh.
const geoCache = new Map();
function targetGeo(pick) {
  if (POINT_MODE) {
    if (!geoCache.has('pt')) geoCache.set('pt', new THREE.SphereGeometry(0.018, 12, 8));
    return geoCache.get('pt');
  }
  const [l, w, h] = pickDims(pick);
  const key = `${l},${w},${h}`;
  if (!geoCache.has(key)) geoCache.set(key, new THREE.BoxGeometry(l, w, h));
  return geoCache.get(key);
}

// Place a target mesh: a point sits AT target z; a packet box sits with its TOP at
// target z (center half a thickness below), using THIS packet's own height.
function setTargetPos(mesh, x, y, z, h) {
  mesh.position.set(x, y, POINT_MODE ? z : z - (h ?? PKT.height_m) / 2);
}

function buildTargets(pose) {
  for (const m of targetMeshes) { scene.remove(m); m.material.dispose(); }  // geo is cached/shared
  targetMeshes = [];
  pose.picks.forEach((pick, idx) => {
    const cls = classOf(pick);
    const mat = new THREE.MeshStandardMaterial({
      color: COL[cls], transparent: !POINT_MODE, opacity: POINT_MODE ? 1.0 : 0.85,
      roughness: 0.8 });
    const m = new THREE.Mesh(targetGeo(pick), mat);
    const h = pickDims(pick)[2];
    m.userData.cls = cls;
    m.userData.h = h;
    m.userData.idx = idx;                           // index into pose.picks (for click-inspect)
    m.userData.ix = idx % GRID.nx;                 // grid column / row (z-major order)
    m.userData.iy = Math.floor(idx / GRID.nx) % GRID.ny;
    setTargetPos(m, ...pick.target_m, h);
    scene.add(m);
    targetMeshes.push(m);
  });
  if (!cursor) {                                  // wireframe highlights; unit boxes scaled per pick
    const mk = (color) => {
      const g = POINT_MODE ? new THREE.SphereGeometry(0.03, 16, 12)
                           : new THREE.BoxGeometry(1, 1, 1);
      const mesh = new THREE.Mesh(g, new THREE.MeshBasicMaterial({ color, wireframe: true }));
      mesh.visible = false; scene.add(mesh); return mesh;
    };
    cursor = mk(0xffffff);                          // playback highlight (white)
    selMarker = mk(0x00e5ff);                       // click-selected packet (cyan)
  }
  applyFilters();
}

function applyFilters() {
  const show = {
    place: document.getElementById('t-place').checked,
    cover: document.getElementById('t-cover').checked,
    miss: document.getElementById('t-miss').checked,
    buried: document.getElementById('t-buried').checked,
  };
  const layer = parseInt(document.getElementById('depth').value, 10);  // -1 = all
  // "Packed view": the grid is sampled finer than a packet, so drawing every cell
  // overlaps. Show only every STEP_X/STEP_Y-th packet (>= one packet apart) so the
  // real-size boxes don't intersect -- a realistic packed-bin layout, each box still
  // coloured by its own computed pickability. Off => the full sampled field (overlaps).
  const packed = !POINT_MODE && document.getElementById('t-packed').checked;
  targetMeshes.forEach((m, i) => {
    const inSub = !packed || (m.userData.ix % STEP_X === 0 && m.userData.iy % STEP_Y === 0);
    m.visible = inSub && show[m.userData.cls] && (layer < 0 || depthOf(i) === layer);
  });
  if (gripper) gripper.visible = document.getElementById('t-grip').checked;
  if (robot) robot.visible = document.getElementById('t-arm').checked;
  if (binGroup) binGroup.visible = document.getElementById('t-bin').checked;
}

// ---------------------------------------------------------------------------
// Robot pose: base + joints (the gripper follows the FK flange).
// ---------------------------------------------------------------------------
function setBasePose(pose) {
  robot.position.set(pose.base.x, pose.base.y, pose.base.z);
  robot.quaternion.copy(flipQuat(THREE.MathUtils.degToRad(pose.base.yaw_deg)));
}
function setJoints(jointsRad, toolYawDeg) {
  JOINT_NAMES.forEach((nm, k) => robot.setJointValue(nm, jointsRad[k]));
  robot.updateMatrixWorld(true);
  placeGripper(toolYawDeg);
}

// Show a static, representative configuration (the middle placement) so the arm
// isn't sitting at the zero pose when not playing.
function showRestPose(pose) {
  if (!placementIdx.length) return;
  const pick = pose.picks[placementIdx[Math.floor(placementIdx.length / 2)]];
  setJoints(pick.joints_rad, pick.tool_yaw_deg);
  cursor.visible = false;
}

// ---------------------------------------------------------------------------
// Pose selection + readout
// ---------------------------------------------------------------------------
function selectPose(i) {
  curPose = poses[i];
  placementIdx = curPose.picks.map((p, k) => (p.is_placement ? k : -1)).filter((k) => k >= 0);
  setBasePose(curPose);
  buildTargets(curPose);
  selIdx = -1; solList = []; if (selMarker) selMarker.visible = false;
  document.getElementById('sel-info').textContent = 'click a packet to inspect';
  document.getElementById('sol-info').textContent = '';
  showRestPose(curPose);
  playStep = 0; setPlaying(false);

  const b = curPose.base;
  const npick = curPose.pickable !== undefined ? curPose.pickable : curPose.covered;
  document.getElementById('readout').innerHTML =
    `<span class="big">${curPose.coverage_pct}%</span> ${POINT_MODE ? 'covered' : 'pickable'} ` +
    `(${npick}/${curPose.total} ${POINT_MODE ? 'points' : 'packets'})<br>` +
    `<b>${curPose.placements}</b> centered placements<br>` +
    `base &nbsp;x=<b>${b.x}</b> y=<b>${b.y}</b> z=<b>${b.z}</b> m<br>` +
    `yaw=<b>${b.yaw_deg}&deg;</b>`;
  document.getElementById('pick-info').textContent =
    `${placementIdx.length} placements to cycle`;
}

// ---------------------------------------------------------------------------
// Playback: advance through the placement list, driving the arm each step.
// ---------------------------------------------------------------------------
function setPlaying(on) {
  playing = on;
  document.getElementById('btn-play').textContent = on ? '❚❚ Pause' : '▶ Play';
  if (on && selMarker) selMarker.visible = false;   // playback overrides a selection
  if (!on) { cursor.visible = false; showRestPose(curPose); }
}
function gotoPlacement(step) {
  if (!placementIdx.length) return;
  playStep = ((step % placementIdx.length) + placementIdx.length) % placementIdx.length;
  const idx = placementIdx[playStep];
  const pick = curPose.picks[idx];
  setJoints(pick.joints_rad, pick.tool_yaw_deg);
  const [pl, pw, ph] = pickDims(pick);           // size the highlight to THIS packet
  if (!POINT_MODE) cursor.scale.set(pl * 1.06, pw * 1.06, ph * 1.06);
  setTargetPos(cursor, ...pick.target_m, ph);    // match the target mesh (sphere or box)
  cursor.visible = true;
  document.getElementById('pick-info').innerHTML =
    `pick <b>${playStep + 1}</b>/${placementIdx.length} &nbsp; ` +
    `fk err ${pick.fk_err_mm} mm &nbsp; tilt ${pick.tool_tilt_deg}&deg;`;
}

// ---------------------------------------------------------------------------
// Click-to-inspect: select a packet, then step through EVERY joint config that
// reaches its TCP (valid placements and failed attempts, with the failure reason)
// so you can see exactly how the arm reaches it -- or how it fails.
// ---------------------------------------------------------------------------
const FAIL_TEXT = {
  reach: 'out of reach', unreachable: "IK can't reach the TCP", tilt: 'tool not level',
  limits: 'joint limit', collision_bin: 'arm hits bin', collision_self: 'arm hits itself',
  collision_gripper: 'plate hits bin',
};

// All recorded IK configs for a pick: the dumped `solutions`, or (older/point JSON
// without them) the single accepted placement, or nothing.
function solutionsOf(pick) {
  if (pick.solutions && pick.solutions.length) return pick.solutions;
  if (pick.is_placement && pick.joints_rad)
    return [{ joints_rad: pick.joints_rad, tool_yaw_deg: pick.tool_yaw_deg,
              fk_err_mm: pick.fk_err_mm, tilt_deg: pick.tool_tilt_deg,
              ok: true, fail: null, accepted: true }];
  return [];
}

function selectPacket(idx) {
  if (idx == null || idx < 0) return;
  setPlaying(false);
  selIdx = idx;
  solList = solutionsOf(curPose.picks[idx]);
  solIdx = Math.max(0, solList.findIndex((s) => s.accepted));  // start on the valid one
  const pick = curPose.picks[idx];
  const [pl, pw, ph] = pickDims(pick);
  if (!POINT_MODE) selMarker.scale.set(pl * 1.12, pw * 1.12, ph * 1.12);
  setTargetPos(selMarker, ...pick.target_m, ph);
  selMarker.visible = true;
  showSolution();
}

function showSolution() {
  const pick = curPose.picks[selIdx];
  const ok = pick.pickable !== undefined ? pick.pickable : pick.covered;
  const status = pick.is_placement ? 'placement (centered pick)'
               : ok ? 'pickable off-center' : 'NOT pickable';
  const di = Math.floor(selIdx / (GRID.nx * GRID.ny));
  let info = `packet [${pick.target_m.map((v) => v.toFixed(2)).join(', ')}] m ` +
             `&nbsp; depth ${di} &nbsp;&mdash; <b>${status}</b>`;
  if (!solList.length) {
    info += '<br><i>no IK config recorded for this TCP (out of reach)</i>';
    document.getElementById('sol-info').textContent = '';
  } else {
    const s = solList[solIdx];
    setJoints(s.joints_rad, s.tool_yaw_deg);
    const tag = s.ok ? '<span style="color:var(--green)">VALID ✓</span>'
                     : `<span style="color:var(--red)">✗ ${FAIL_TEXT[s.fail] || s.fail}</span>`;
    document.getElementById('sol-info').innerHTML =
      `config <b>${solIdx + 1}</b>/${solList.length} &nbsp; ${tag}` +
      `${s.accepted ? ' (used)' : ''}<br>fk ${s.fk_err_mm} mm &nbsp; tilt ${s.tilt_deg}&deg;`;
  }
  document.getElementById('sel-info').innerHTML = info;
}

function stepSolution(d) {
  if (selIdx < 0 || !solList.length) return;
  solIdx = (solIdx + d + solList.length) % solList.length;
  showSolution();
}

// Jump the selection to the next/previous NOT-pickable packet (path the failures).
// Buried packets are skipped -- they're never scored, so there's nothing to inspect.
function stepFailed(d) {
  const picks = curPose.picks;
  const isFail = (p) => !p.buried && !(p.pickable !== undefined ? p.pickable : p.covered);
  const n = picks.length;
  let start = selIdx < 0 ? (d > 0 ? -1 : n) : selIdx;
  for (let k = 1; k <= n; k++) {
    const j = ((start + d * k) % n + n) % n;
    if (isFail(picks[j])) { selectPacket(j); return; }
  }
}

function clearSelection() {
  selIdx = -1; solList = []; selMarker.visible = false;
  document.getElementById('sel-info').textContent = 'click a packet to inspect';
  document.getElementById('sol-info').textContent = '';
  showRestPose(curPose);
}

// Click vs orbit-drag: only treat a near-stationary press/release as a click.
const raycaster = new THREE.Raycaster();
let downXY = null;
renderer.domElement.addEventListener('pointerdown', (e) => { downXY = [e.clientX, e.clientY]; });
renderer.domElement.addEventListener('pointerup', (e) => {
  if (!downXY) return;
  const moved = Math.hypot(e.clientX - downXY[0], e.clientY - downXY[1]);
  downXY = null;
  if (moved > 5 || !curPose) return;             // a drag (orbit), not a click
  const r = renderer.domElement.getBoundingClientRect();
  const ndc = new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1,
                                -((e.clientY - r.top) / r.height) * 2 + 1);
  raycaster.setFromCamera(ndc, camera);
  const hits = raycaster.intersectObjects(targetMeshes.filter((m) => m.visible), false);
  if (hits.length) selectPacket(hits[0].object.userData.idx);
});

// ---------------------------------------------------------------------------
// Data + robot loading
// ---------------------------------------------------------------------------
function fail(detail) {
  document.getElementById('error').style.display = 'grid';
  document.getElementById('error-detail').textContent = detail || '';
}

async function loadData() {
  if (!runArg) { fail('No ?run= parameter in the URL.'); return null; }
  if (!runArg.endsWith('.json')) runArg = runArg.replace(/\/?$/, '/') + 'best_versions.json';
  const res = await fetch('/' + runArg.replace(/^\//, ''));
  if (!res.ok) { fail(`Could not fetch ${runArg} (HTTP ${res.status}).`); return null; }
  return res.json();
}

function loadRobot() {
  return new Promise((resolve, reject) => {
    const loader = new URDFLoader(new THREE.LoadingManager());
    loader.packages = { fairino_description: PKG_ROOT };
    loader.load(URDF_URL, (r) => {
      r.traverse((c) => { if (c.isMesh) { c.castShadow = false; c.material.metalness = 0.1; } });
      resolve(r);
    }, undefined, (e) => reject(e));
  });
}

// ---------------------------------------------------------------------------
// UI wiring
// ---------------------------------------------------------------------------
function wireUI(bundle) {
  const sel = document.getElementById('pose-select');
  const mk = (b, tag) => ({ ...b, label: `${tag}${b.rank} — ${b.coverage_pct}%  ` +
    `(x=${b.base.x}, y=${b.base.y}, z=${b.base.z}, yaw=${b.base.yaw_deg}°)` });
  poses = [
    ...bundle.top_bests.map((b) => mk(b, '#')),
    ...bundle.diverse_bests.map((b) => mk(b, 'D')),
  ];
  poses.forEach((p, i) => {
    const o = document.createElement('option');
    o.value = i; o.textContent = p.label; sel.appendChild(o);
  });
  sel.addEventListener('change', () => selectPose(parseInt(sel.value, 10)));

  ['t-place', 't-cover', 't-miss', 't-buried', 't-packed', 't-arm', 't-grip', 't-bin']
    .forEach((id) => document.getElementById(id).addEventListener('change', applyFilters));

  const depth = document.getElementById('depth');
  depth.max = String(GRID.nz - 1);
  depth.addEventListener('input', () => {
    const v = parseInt(depth.value, 10);
    document.getElementById('depth-val').textContent = v < 0 ? 'all' : `${v} (z idx)`;
    applyFilters();
  });

  document.getElementById('btn-play').addEventListener('click', () => setPlaying(!playing));
  document.getElementById('btn-step').addEventListener('click', () => {
    setPlaying(false); gotoPlacement(playStep + 1);
  });
  const speed = document.getElementById('speed');
  speed.addEventListener('input', () => {
    picksPerSec = parseInt(speed.value, 10);
    document.getElementById('speed-val').textContent = picksPerSec;
  });

  document.getElementById('sol-prev').addEventListener('click', () => stepSolution(-1));
  document.getElementById('sol-next').addEventListener('click', () => stepSolution(+1));
  document.getElementById('fail-prev').addEventListener('click', () => stepFailed(-1));
  document.getElementById('fail-next').addEventListener('click', () => stepFailed(+1));
  document.getElementById('sel-clear').addEventListener('click', clearSelection);
}

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------
async function main() {
  const bundle = await loadData();
  if (!bundle) return;
  CFG = bundle.config;
  GRID = CFG.target_grid;
  POINT_MODE = !CFG.packet;                       // no packet block => original point sim
  PKT = CFG.packet || { length_m: 0.2413, width_m: 0.3302, height_m: 0.0318 };
  // Packed-view stride: how many grid cells span one packet (so drawn boxes don't
  // overlap). Derived from the actual grid pitch and the LARGEST packet in the run
  // (so mixed sizes still don't clip), not a hard-coded size.
  if (!POINT_MODE) {
    const pk = bundle.top_bests[0].picks;
    const dx = Math.abs(pk[1].target_m[0] - pk[0].target_m[0]) || PKT.length_m;
    const dy = Math.abs(pk[GRID.nx].target_m[1] - pk[0].target_m[1]) || PKT.width_m;
    let maxL = PKT.length_m, maxW = PKT.width_m;
    for (const p of pk) if (p.size_m) { maxL = Math.max(maxL, p.size_m[0]); maxW = Math.max(maxW, p.size_m[1]); }
    STEP_X = Math.max(1, Math.ceil(maxL / dx - 1e-9));
    STEP_Y = Math.max(1, Math.ceil(maxW / dy - 1e-9));
  } else {
    document.getElementById('row-packed').style.display = 'none';  // points don't overlap
  }
  // The 'buried' layer toggle only matters for the fully-packed sim.
  if (!bundle.top_bests[0].picks.some((p) => p.buried))
    document.getElementById('row-buried').style.display = 'none';
  // Adapt the legend wording to the sim that produced this run.
  const L = POINT_MODE
    ? { place: 'placement (foam centered)', cover: 'covered off-center', miss: 'not covered' }
    : { place: 'placement packets (plate centered)', cover: 'pickable off-center', miss: 'not pickable' };
  document.getElementById('lbl-place').textContent = L.place;
  document.getElementById('lbl-cover').textContent = L.cover;
  document.getElementById('lbl-miss').textContent = L.miss;
  document.getElementById('run-label').textContent =
    `${runArg.split('/').slice(-2, -1)[0] || 'run'} · ${POINT_MODE ? 'points' : 'packets'} · ` +
    `${bundle.summary.total_base_poses} poses × ${bundle.summary.total_targets} targets`;

  buildBin();
  buildGripper();
  try {
    robot = await loadRobot();
    scene.add(robot);
  } catch (e) {
    fail('Failed to load the robot URDF/meshes: ' + e);
    return;
  }
  wireUI(bundle);
  selectPose(0);
}

let last = performance.now();
function tick(now) {
  const dt = (now - last) / 1000; last = now;
  if (playing && placementIdx.length) {
    playAccum += dt;
    const interval = 1 / picksPerSec;
    if (playAccum >= interval) {
      playAccum = 0;
      gotoPlacement(playStep + 1);
    }
  }
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(tick);
}

main();
requestAnimationFrame(tick);

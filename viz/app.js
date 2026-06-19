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

const COL = { place: 0x1ae62a, cover: 0xffc83d, miss: 0xff4d4d };

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
let poses = [];                 // [{label, ...best_block}]
let robot = null;               // URDFRobot
let gripper = null;             // THREE.Group (post + plate)
let targetMeshes = [];          // one sphere per pick, index-aligned to picks
let cursor = null;              // wireframe highlight on the active placement
let curPose = null;
let placementIdx = [];          // indices into picks where is_placement is true
let playStep = 0, playing = false, playAccum = 0, picksPerSec = 6;

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
// Targets: one small sphere per pick (600), coloured by outcome. Kept
// index-aligned to pose.picks so depth slicing and playback are simple lookups.
// ---------------------------------------------------------------------------
function classOf(pick) {
  if (pick.is_placement) return 'place';
  return pick.covered ? 'cover' : 'miss';
}
function depthOf(i) { return Math.floor(i / (GRID.nx * GRID.ny)); }  // z-major order

function buildTargets(pose) {
  for (const m of targetMeshes) scene.remove(m);
  targetMeshes = [];
  const geo = new THREE.SphereGeometry(0.016, 12, 8);
  pose.picks.forEach((pick) => {
    const cls = classOf(pick);
    const mat = new THREE.MeshBasicMaterial({ color: COL[cls] });
    const m = new THREE.Mesh(geo, mat);
    m.position.set(...pick.target_m);
    m.userData.cls = cls;
    scene.add(m);
    targetMeshes.push(m);
  });
  if (!cursor) {
    cursor = new THREE.Mesh(
      new THREE.SphereGeometry(0.03, 16, 12),
      new THREE.MeshBasicMaterial({ color: 0xffffff, wireframe: true }));
    cursor.visible = false;
    scene.add(cursor);
  }
  applyFilters();
}

function applyFilters() {
  const show = {
    place: document.getElementById('t-place').checked,
    cover: document.getElementById('t-cover').checked,
    miss: document.getElementById('t-miss').checked,
  };
  const layer = parseInt(document.getElementById('depth').value, 10);  // -1 = all
  targetMeshes.forEach((m, i) => {
    m.visible = show[m.userData.cls] && (layer < 0 || depthOf(i) === layer);
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
  showRestPose(curPose);
  playStep = 0; setPlaying(false);

  const b = curPose.base;
  document.getElementById('readout').innerHTML =
    `<span class="big">${curPose.coverage_pct}%</span> covered ` +
    `(${curPose.covered}/${curPose.total})<br>` +
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
  if (!on) { cursor.visible = false; showRestPose(curPose); }
}
function gotoPlacement(step) {
  if (!placementIdx.length) return;
  playStep = ((step % placementIdx.length) + placementIdx.length) % placementIdx.length;
  const idx = placementIdx[playStep];
  const pick = curPose.picks[idx];
  setJoints(pick.joints_rad, pick.tool_yaw_deg);
  cursor.position.set(...pick.target_m);
  cursor.visible = true;
  document.getElementById('pick-info').innerHTML =
    `pick <b>${playStep + 1}</b>/${placementIdx.length} &nbsp; ` +
    `fk err ${pick.fk_err_mm} mm &nbsp; tilt ${pick.tool_tilt_deg}&deg;`;
}

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

  ['t-place', 't-cover', 't-miss', 't-arm', 't-grip', 't-bin']
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
}

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------
async function main() {
  const bundle = await loadData();
  if (!bundle) return;
  CFG = bundle.config;
  GRID = CFG.target_grid;
  document.getElementById('run-label').textContent =
    `${runArg.split('/').slice(-2, -1)[0] || 'run'} · ` +
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

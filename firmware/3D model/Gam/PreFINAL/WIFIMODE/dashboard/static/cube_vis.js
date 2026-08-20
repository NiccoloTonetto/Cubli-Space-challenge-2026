/* cube_vis.js -- the 3D attitude view.
 *
 * Draws the cube in its real balancing pose and shows the resolved pivot.
 *
 * TWO THINGS THIS DELIBERATELY DOES NOT DO
 *
 * 1. It does not treat phi as Euler pitch/roll/yaw. phi_x/y/z is the tilt
 *    ERROR VECTOR away from the resolved equilibrium, so the honest reading is
 *    axis-angle: spin about phi/|phi| by |phi|. |phi| is also exactly the
 *    scalar the firmware's arm gate tests, which is why it gets its own tile.
 *
 * 2. It does not trust the pivot NAME to say which way is down. Corner names
 *    agree with their gB octant, but the edge names do not -- 'Y[+1,+1]
 *    (+X,+Z up)' carries gB=(0.707,0,0.707), i.e. +X/+Z is DOWN. gB is
 *    documented in cubli_gains.h:49 as the body-frame gravity direction at
 *    balance, so gB is the single source of truth here and the parenthetical
 *    is ignored. The server ships gB in the status message.
 */

import * as THREE from "three";

const CYAN  = 0x22d3ee;
const AMBER = 0xfbbf24;
const SIDE  = 1.0;
const H     = SIDE / 2;

export function initCube(canvas) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(42, 1, 0.1, 100);

  // --- lighting: cool key, warm rim, so the amber pivot reads against cyan ---
  scene.add(new THREE.AmbientLight(0x22d3ee, 0.35));
  const key = new THREE.DirectionalLight(0xffffff, 1.5);
  key.position.set(3, 5, 4);
  scene.add(key);
  const rim = new THREE.DirectionalLight(AMBER, 0.7);
  rim.position.set(-4, 1, -3);
  scene.add(rim);

  // --- ground: the table the cube balances on ---
  const grid = new THREE.GridHelper(8, 32, 0x1e4a55, 0x0d2730);
  grid.material.transparent = true;
  grid.material.opacity = 0.5;
  scene.add(grid);

  // --- the cube itself: `body` carries the body frame, so everything added
  //     to it can be positioned in raw body coordinates ---
  const body = new THREE.Group();
  scene.add(body);

  const box = new THREE.Mesh(
    new THREE.BoxGeometry(SIDE, SIDE, SIDE),
    new THREE.MeshStandardMaterial({
      color: 0x0d3d4d, transparent: true, opacity: 0.17,
      roughness: 0.25, metalness: 0.6, depthWrite: false,
      side: THREE.DoubleSide,
    }));
  body.add(box);

  const wireframe = new THREE.LineSegments(
    new THREE.EdgesGeometry(box.geometry),
    new THREE.LineBasicMaterial({ color: CYAN, transparent: true, opacity: 0.85 }));
  body.add(wireframe);

  // --- reaction wheels: one per body axis, spun at the measured rho so wheel
  //     speed is visible as motion and not only as a number ---
  const wheels = [];
  const axes = [
    { axis: new THREE.Vector3(1, 0, 0), rot: [0, 0, Math.PI / 2] },
    { axis: new THREE.Vector3(0, 1, 0), rot: [0, 0, 0] },
    { axis: new THREE.Vector3(0, 0, 1), rot: [Math.PI / 2, 0, 0] },
  ];
  for (const a of axes) {
    const g = new THREE.Group();
    const disc = new THREE.Mesh(
      new THREE.CylinderGeometry(H * 0.62, H * 0.62, 0.022, 40, 1, true),
      new THREE.MeshStandardMaterial({
        color: CYAN, transparent: true, opacity: 0.2,
        roughness: 0.3, metalness: 0.8, side: THREE.DoubleSide }));
    const spoke = new THREE.LineSegments(
      new THREE.EdgesGeometry(new THREE.CylinderGeometry(H * 0.62, H * 0.62, 0.022, 8)),
      new THREE.LineBasicMaterial({ color: CYAN, transparent: true, opacity: 0.5 }));
    g.add(disc, spoke);
    g.rotation.set(...a.rot);
    body.add(g);
    wheels.push({ group: g, angle: 0 });
  }

  // --- pivot highlight: rebuilt whenever the firmware resolves a new one ---
  let pivotGroup = null;
  let pivotBody = new THREE.Vector3(0, -H, 0);   // body-frame contact point
  let qBase = new THREE.Quaternion();            // equilibrium orientation
  let schema = null;

  function disposeGroup(g) {
    if (!g) return;
    g.traverse((o) => {
      if (o.geometry) o.geometry.dispose();
      if (o.material) o.material.dispose();
    });
    body.remove(g);
  }

  /** Rebuild the highlight from the server's pivot descriptor (gB-driven). */
  function setPivot(pivot, activeSchema) {
    schema = activeSchema;
    disposeGroup(pivotGroup);
    pivotGroup = null;

    if (!pivot || !pivot.gB) {
      qBase.identity();
      pivotBody.set(0, -H, 0);
      return;
    }

    const gB = new THREE.Vector3(...pivot.gB).normalize();
    // Equilibrium pose: whatever body direction gravity points along must end
    // up pointing at the floor.
    qBase.setFromUnitVectors(gB, new THREE.Vector3(0, -1, 0));

    pivotGroup = new THREE.Group();
    const sx = Math.sign(gB.x) || 0, sy = Math.sign(gB.y) || 0, sz = Math.sign(gB.z) || 0;

    if (pivot.kind === "edge" && pivot.e) {
      // Contact LINE: runs along e; the two transverse coords are pinned to
      // the face gB points at.
      const e = new THREE.Vector3(...pivot.e).normalize();
      const off = new THREE.Vector3(
        e.x ? 0 : sx * H, e.y ? 0 : sy * H, e.z ? 0 : sz * H);
      const a = off.clone().addScaledVector(e, -H);
      const b = off.clone().addScaledVector(e,  H);
      pivotBody.copy(off);

      const tube = new THREE.Mesh(
        new THREE.CylinderGeometry(0.022, 0.022, SIDE, 14),
        new THREE.MeshBasicMaterial({ color: AMBER }));
      tube.position.copy(off);
      // Cylinders are built along +Y; aim it down the edge direction.
      tube.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), e);
      pivotGroup.add(tube);
      for (const p of [a, b]) {
        const cap = new THREE.Mesh(
          new THREE.SphereGeometry(0.038, 16, 12),
          new THREE.MeshBasicMaterial({ color: AMBER }));
        cap.position.copy(p);
        pivotGroup.add(cap);
      }
    } else {
      // Contact VERTEX: the corner of the box in gB's octant.
      const v = new THREE.Vector3(sx * H, sy * H, sz * H);
      pivotBody.copy(v);

      const knob = new THREE.Mesh(
        new THREE.SphereGeometry(0.058, 20, 16),
        new THREE.MeshBasicMaterial({ color: AMBER }));
      knob.position.copy(v);
      pivotGroup.add(knob);

      const halo = new THREE.Mesh(
        new THREE.SphereGeometry(0.11, 20, 16),
        new THREE.MeshBasicMaterial({ color: AMBER, transparent: true, opacity: 0.16 }));
      halo.position.copy(v);
      pivotGroup.add(halo);

      // The three edges meeting at that vertex -- they are what you sight
      // along when placing the cube by hand.
      const pts = [];
      for (const d of [[-sx * SIDE, 0, 0], [0, -sy * SIDE, 0], [0, 0, -sz * SIDE]]) {
        pts.push(v, new THREE.Vector3(v.x + d[0], v.y + d[1], v.z + d[2]));
      }
      pivotGroup.add(new THREE.LineSegments(
        new THREE.BufferGeometry().setFromPoints(pts),
        new THREE.LineBasicMaterial({ color: AMBER, transparent: true, opacity: 0.95 })));
    }
    body.add(pivotGroup);
  }

  // --- per-frame state, written by update(), consumed by the render loop ---
  const qTilt = new THREE.Quaternion();
  const qTotal = new THREE.Quaternion();
  const phiVec = new THREE.Vector3();
  const contactWorld = new THREE.Vector3();
  let rho = [0, 0, 0];
  let lastFrame = performance.now();

  /**
   * @param {number[]} phiDeg  corner: [x,y,z] tilt error. edge: [theta] about e.
   * @param {number[]} rhoRadS wheel speeds, for the disc animation.
   * @param {number[]} edgeDir edge build only: the contact-line direction.
   */
  function update(phiDeg, rhoRadS, edgeDir) {
    if (schema === "edge") {
      const e = edgeDir ? new THREE.Vector3(...edgeDir).normalize()
                        : new THREE.Vector3(0, 1, 0);
      qTilt.setFromAxisAngle(e, THREE.MathUtils.degToRad(phiDeg[0] || 0));
    } else {
      phiVec.set(phiDeg[0] || 0, phiDeg[1] || 0, phiDeg[2] || 0);
      const mag = phiVec.length();
      if (mag > 1e-9) {
        qTilt.setFromAxisAngle(phiVec.divideScalar(mag),
                               THREE.MathUtils.degToRad(mag));
      } else {
        qTilt.identity();
      }
    }
    // Body-frame perturbation applied to the equilibrium pose: at phi = 0 the
    // cube sits exactly balanced.
    qTotal.copy(qBase).multiply(qTilt);
    body.quaternion.copy(qTotal);

    // Keep the contact feature on the floor instead of letting the cube swim.
    contactWorld.copy(pivotBody).applyQuaternion(qTotal);
    body.position.y = -contactWorld.y;

    if (rhoRadS) rho = rhoRadS;
  }

  // --- minimal orbit: drag to rotate, wheel to zoom. Written out rather than
  //     vendoring OrbitControls for one more file. ---
  let camYaw = 0.72, camPitch = 0.42, camDist = 3.2, dragging = false, px = 0, py = 0;
  canvas.addEventListener("pointerdown", (e) => {
    dragging = true; px = e.clientX; py = e.clientY; canvas.setPointerCapture(e.pointerId);
  });
  canvas.addEventListener("pointerup", (e) => {
    dragging = false; canvas.releasePointerCapture(e.pointerId);
  });
  canvas.addEventListener("pointermove", (e) => {
    if (!dragging) return;
    camYaw -= (e.clientX - px) * 0.008;
    camPitch = Math.max(-0.25, Math.min(1.4, camPitch + (e.clientY - py) * 0.006));
    px = e.clientX; py = e.clientY;
  });
  canvas.addEventListener("wheel", (e) => {
    e.preventDefault();
    camDist = Math.max(1.6, Math.min(8, camDist + e.deltaY * 0.002));
  }, { passive: false });

  function resize() {
    // clientWidth/Height can read 0 while the page is still laying out, or if
    // a future style change leaves the parent auto-height. Fall back to the
    // parent's box, then to a floor -- a zero here silently renders nothing,
    // which is indistinguishable from a broken scene.
    let w = canvas.clientWidth, h = canvas.clientHeight;
    if (!w || !h) {
      const r = canvas.parentElement.getBoundingClientRect();
      w = w || Math.round(r.width);
      h = h || Math.round(r.height);
    }
    w = Math.max(w, 1);
    h = Math.max(h, 1);
    if (canvas.width !== w || canvas.height !== h) {
      renderer.setSize(w, h, false);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
    }
  }

  function frame() {
    requestAnimationFrame(frame);
    resize();

    const now = performance.now();
    const dt = Math.min((now - lastFrame) / 1000, 0.1);
    lastFrame = now;

    // Spin the discs at the real wheel rate, scaled down so 500 rad/s does not
    // just strobe into visual noise.
    for (let i = 0; i < wheels.length; i++) {
      wheels[i].angle += (rho[i] || 0) * dt * 0.12;
      wheels[i].group.rotation.y = wheels[i].angle;
    }

    camera.position.set(
      Math.sin(camYaw) * Math.cos(camPitch) * camDist,
      Math.sin(camPitch) * camDist + 0.35,
      Math.cos(camYaw) * Math.cos(camPitch) * camDist);
    camera.lookAt(0, 0.42, 0);
    renderer.render(scene, camera);
  }
  frame();

  return { setPivot, update };
}

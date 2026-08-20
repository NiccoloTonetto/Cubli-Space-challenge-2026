/* app.js -- WebSocket client, live charts, metric tiles, command buttons.
 *
 * Talks to app.py over /ws. Three message kinds arrive: `telemetry` (a batch
 * of raw rows in the active schema's column order), `console` (one firmware
 * '#' line), and `status` (the server's view of the cube). Outbound is only
 * ever {type:"cmd"|"record"|"mode"} -- the server validates every command
 * against the firmware's real grammar before anything reaches the wire.
 */

import { initCube } from "/static/cube_vis.js";

const $ = (id) => document.getElementById(id);

const WINDOW_S = 10;        // matches WINDOW_S in the matplotlib scripts
const CHART_HZ = 60;        // points/s kept per series; 250-500 Hz would melt Chart.js
const COL = { x: "#22d3ee", y: "#fbbf24", z: "#f472b6", k: "#e2e8f0", d: "#64748b" };

let ws = null;
let schema = null;          // exact schema, e.g. 'corner_trim_filt'
let family = null;          // 'corner' | 'edge' -- drives layout and 3D scene
let haltCmd = null;         // 'p' (legacy) or 'h' (AutoTrim); null = no halt
let charts = [];            // [{chart, series:[{key,idx}], t0}]
let lastChartPush = -1e9;
let cube = null;
let pivot = null;
let armGate = 0.5;
let phiNorm = null;
let armPending = false;
let armPendingTimer = null;

// ---------------------------------------------------------------------------
// Chart plumbing
// ---------------------------------------------------------------------------

Chart.defaults.color = "#64748b";
Chart.defaults.borderColor = "rgba(56,189,248,.08)";
Chart.defaults.font.family = "ui-monospace, Consolas, monospace";
Chart.defaults.font.size = 9;

function chartOpts(unit) {
  return {
    responsive: true, maintainAspectRatio: false,
    animation: false, parsing: false, normalized: true,
    interaction: { mode: null }, spanGaps: true,
    elements: { point: { radius: 0 }, line: { borderWidth: 1.4, tension: 0 } },
    scales: {
      x: { type: "linear", title: { display: true, text: "t (s)" },
           ticks: { maxTicksLimit: 7, callback: (v) => v.toFixed(1) },
           grid: { color: "rgba(56,189,248,.06)" } },
      y: { title: { display: true, text: unit },
           grid: { color: "rgba(56,189,248,.06)" } },
    },
    plugins: {
      legend: { display: true, position: "top", align: "end",
                labels: { boxWidth: 8, boxHeight: 8, padding: 6, usePointStyle: true } },
      tooltip: { enabled: false },
    },
  };
}

/** Column layouts, mirroring the panel split the matplotlib scripts use. */
const LAYOUTS = {
  corner: [
    { title: "Tilt φ", unit: "deg", series: [
      { label: "φx", idx: 1, color: COL.x }, { label: "φy", idx: 2, color: COL.y },
      { label: "φz", idx: 3, color: COL.z }, { label: "|φ|", idx: -1, color: COL.k, width: 2 }] },
    { title: "Body rate ω", unit: "deg/s", series: [
      { label: "ωx", idx: 4, color: COL.x }, { label: "ωy", idx: 5, color: COL.y },
      { label: "ωz", idx: 6, color: COL.z }] },
    { title: "Wheels ρ (dashed = 5 s low-pass)", unit: "rad/s", series: [
      { label: "ρx", idx: 7, color: COL.x }, { label: "ρy", idx: 8, color: COL.y },
      { label: "ρz", idx: 9, color: COL.z },
      { label: "ρx_lp", idx: 10, color: COL.x, dash: [4, 3] },
      { label: "ρy_lp", idx: 11, color: COL.y, dash: [4, 3] },
      { label: "ρz_lp", idx: 12, color: COL.z, dash: [4, 3] }] },
    { title: "Torque τ (dashed = commanded)", unit: "N·m", series: [
      { label: "τx", idx: 13, color: COL.x }, { label: "τy", idx: 14, color: COL.y },
      { label: "τz", idx: 15, color: COL.z },
      { label: "τx_cmd", idx: 16, color: COL.x, dash: [4, 3] },
      { label: "τy_cmd", idx: 17, color: COL.y, dash: [4, 3] },
      { label: "τz_cmd", idx: 18, color: COL.z, dash: [4, 3] }] },
  ],
  edge: [
    { title: "Tilt θ", unit: "deg", series: [{ label: "θ", idx: 1, color: COL.x, width: 2 }] },
    { title: "Tilt rate θ̇", unit: "deg/s", series: [{ label: "θ̇", idx: 2, color: COL.y }] },
    { title: "Torque τ (dashed = commanded)", unit: "N·m", series: [
      { label: "τ", idx: 3, color: COL.x },
      { label: "τ_cmd", idx: 4, color: COL.y, dash: [4, 3] }] },
    { title: "Wheel", unit: "rad/s · rev/s", series: [
      { label: "ω_lp", idx: 7, color: COL.x }, { label: "vel", idx: 9, color: COL.z }] },
  ],
};

function buildCharts(which) {
  for (const c of charts) c.chart.destroy();
  charts = [];
  const host = $("charts");
  host.innerHTML = "";

  for (const spec of LAYOUTS[which]) {
    const wrap = document.createElement("div");
    wrap.className = "panel rounded-lg p-3";
    wrap.innerHTML =
      `<div class="text-[10px] tracking-[0.18em] text-cyan-400/70 uppercase mb-1">${spec.title}</div>
       <div style="height:190px"><canvas></canvas></div>`;
    host.appendChild(wrap);

    const chart = new Chart(wrap.querySelector("canvas"), {
      type: "line",
      data: { datasets: spec.series.map((s) => ({
        label: s.label, data: [], borderColor: s.color,
        borderWidth: s.width || 1.4, borderDash: s.dash || [],
        backgroundColor: "transparent",
      })) },
      options: chartOpts(spec.unit),
    });
    charts.push({ chart, series: spec.series, t0: null });
  }
}

/** Push one telemetry row into every chart, thinned to CHART_HZ. */
function pushRow(row) {
  const tms = row[0];
  if (tms - lastChartPush < 1000 / CHART_HZ) return;
  lastChartPush = tms;

  for (const c of charts) {
    if (c.t0 === null) c.t0 = tms;
    const t = (tms - c.t0) / 1000;
    c.series.forEach((s, i) => {
      const v = s.idx === -1
        ? Math.hypot(row[1], row[2], row[3])   // |φ|, the number the gate tests
        : row[s.idx];
      const d = c.chart.data.datasets[i].data;
      d.push({ x: t, y: v });
      while (d.length && d[0].x < t - WINDOW_S) d.shift();
    });
    const xmax = Math.max(WINDOW_S, t);
    c.chart.options.scales.x.min = xmax - WINDOW_S;
    c.chart.options.scales.x.max = xmax;
  }
}

const redraw = () => { for (const c of charts) c.chart.update("none"); };

// ---------------------------------------------------------------------------
// Tiles
// ---------------------------------------------------------------------------

const RPM = (radps) => radps * 60 / (2 * Math.PI);
const f = (v, n = 2) => (v === undefined || v === null || Number.isNaN(v) ? "—" : v.toFixed(n));
const trio = (a, b, c, n = 2) =>
  `<span style="color:${COL.x}">${f(a, n)}</span> / ` +
  `<span style="color:${COL.y}">${f(b, n)}</span> / ` +
  `<span style="color:${COL.z}">${f(c, n)}</span>`;

function setTileLabels(which) {
  const L = which === "corner"
    ? ["φ x / y / z — deg", "ω x / y / z — deg/s", "Wheels — RPM", "τ x / y / z — N·m"]
    : ["θ — deg", "θ̇ — deg/s", "Wheel — RPM", "τ / τ_cmd — N·m"];
  ["a", "b", "c", "d"].forEach((k, i) => { $("lbl-" + k).textContent = L[i]; });
}

function updateTiles(row) {
  if (family === "corner") {
    phiNorm = Math.hypot(row[1], row[2], row[3]);
    $("m-a").innerHTML = trio(row[1], row[2], row[3], 3);
    $("m-b").innerHTML = trio(row[4], row[5], row[6], 1);
    $("m-c").innerHTML = trio(RPM(row[7]), RPM(row[8]), RPM(row[9]), 0);
    $("m-d").innerHTML = trio(row[13], row[14], row[15], 4);
    if (cube) cube.update([row[1], row[2], row[3]], [row[7], row[8], row[9]]);
  } else {
    phiNorm = Math.abs(row[1]);
    $("m-a").innerHTML = `<span style="color:${COL.x}">${f(row[1], 3)}</span>`;
    $("m-b").innerHTML = `<span style="color:${COL.y}">${f(row[2], 1)}</span>`;
    $("m-c").innerHTML = `<span style="color:${COL.x}">${f(RPM(row[7]), 0)}</span>`;
    $("m-d").innerHTML = `<span style="color:${COL.x}">${f(row[3], 4)}</span> / ` +
                         `<span style="color:${COL.y}">${f(row[4], 4)}</span>`;
    if (cube) cube.update([row[1]], [0, 0, 0], pivot && pivot.e);
  }

  $("m-phi").textContent = f(phiNorm, 3);
  const pct = Math.min(100, (phiNorm / Math.max(armGate, 1e-6)) * 100);
  const bar = $("phi-bar");
  bar.style.width = pct + "%";
  bar.style.background = phiNorm < armGate ? "#34d399" : pct > 300 ? "#f43f5e" : "#fbbf24";
  $("m-phi").style.color = phiNorm < armGate ? "#34d399" : "#22d3ee";
}

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------

function pill(el, text, tone) {
  const tones = {
    ok:   "border-emerald-500/50 text-emerald-300 bg-emerald-500/10",
    live: "border-cyan-500/50 text-cyan-300 bg-cyan-500/10",
    warn: "border-amber-500/50 text-amber-300 bg-amber-500/10",
    bad:  "border-rose-500/60 text-rose-300 bg-rose-500/10",
    off:  "border-slate-700 text-slate-500",
  };
  el.className = `mono text-[11px] px-2.5 py-1 rounded border ${tones[tone]}` +
                 (el.id === "pill-armed" ? " font-bold" : "");
  el.textContent = text;
}

function applyStatus(st) {
  // Five corner widths (21/26/29/33/36) share one layout, so switch on the
  // family; the exact schema is shown in the pill for identification only.
  if (st.family && st.family !== family) {
    family = st.family;
    buildCharts(family);
    setTileLabels(family);
    lastChartPush = -1e9;
  }
  schema = st.schema;
  haltCmd = st.halt_cmd;

  // Buttons that a given build genuinely lacks are disabled rather than left
  // to silently do nothing. The edge build has no halt; the AutoTrim builds
  // replaced the manual z-tare with automatic trim.
  const isCorner = st.family === "corner";
  $("btn-halt").disabled = !haltCmd;
  $("btn-resume").disabled = !haltCmd;
  const hasZTare = isCorner && st.schema === "corner";
  $("btn-tare").disabled = !hasZTare;
  $("btn-untare").disabled = !hasZTare;

  pill($("pill-link"),
       st.stale ? "○ NO LINK" : `● ${f(st.rate_hz, 0)} Hz`,
       st.stale ? "off" : "live");
  pill($("pill-mode"),
       st.schema ? `${st.schema.toUpperCase()}${st.schema_locked ? " (LOCKED)" : ""}` : "SCHEMA —",
       st.schema ? "ok" : "off");
  pill($("pill-armed"), st.armed ? "▲ ARMED" : st.halted ? "■ HALTED" : "DISARMED",
       st.armed ? "bad" : st.halted ? "warn" : "off");
  $("pill-armed").classList.toggle("pulse", !!st.armed);

  const rec = $("pill-rec");
  rec.classList.toggle("hidden", !st.recording);
  if (st.recording) pill(rec, `● REC ${st.recorded_rows}`, "warn");
  $("rec-status").textContent = st.recording
    ? `${st.recording} — ${st.recorded_rows} rows` : "idle";

  armGate = st.arm_gate_deg;
  $("m-gate").textContent =
    `gate ${f(armGate, 2)} deg (${st.arm_gate_source})`;

  // The firmware is the authority on its own gate, and the three copies in
  // the tree disagree. If it reports something the generated gains cannot
  // justify, say so rather than drawing a reassuring green bar.
  const gw = $("gate-warn");
  gw.classList.toggle("hidden", !st.arm_gate_suspect);
  if (st.arm_gate_suspect) {
    gw.textContent =
      `⚠  firmware reports ARM_GATE = ${f(armGate, 2)} deg. cubli_gains.h ` +
      `specifies 0.50 deg. EdgeBalance_WiFi.ino:484 holds 0.1672664619 rad ` +
      `(9.58 deg), which looks like a typo of 0.00872664619 — verify before arming.`;
  }

  if (st.pivot && (!pivot || pivot.name !== st.pivot.name)) {
    pivot = st.pivot;
    $("pivot-label").textContent = pivot.name;
    $("pivot-sub").textContent = pivot.gB
      ? `${pivot.kind} · place offset ${f(pivot.place_offset_deg, 3)}°`
      : "unrecognised — no highlight";
    if (cube) cube.setPivot(pivot, family);
  }

  $("m-rate").textContent = st.stale ? "—" : f(st.rate_hz, 0) + " Hz";
  $("m-age").textContent = st.link_age_s === null ? "—" : f(st.link_age_s * 1000, 0) + " ms";
  $("m-pkts").textContent = st.packets;
  $("m-gain").textContent = f(st.gain, 2);

  const canArm = !st.read_only && !st.armed && !st.stale &&
                 phiNorm !== null && phiNorm < armGate;
  $("btn-arm").disabled = !canArm && !armPending;
  if (!armPending) {
    $("arm-hint").textContent = st.read_only ? "monitor mode — arming blocked (--read-only)"
      : st.armed ? "armed — DISARM is in the header"
      : st.stale ? "no link"
      : phiNorm === null ? "waiting for telemetry"
      : canArm ? `|φ| ${f(phiNorm, 3)} < ${f(armGate, 2)} deg — ready`
      : `|φ| ${f(phiNorm, 3)} ≥ ${f(armGate, 2)} deg — too far from equilibrium`;
  }
}

// ---------------------------------------------------------------------------
// Console
// ---------------------------------------------------------------------------

function pushConsole(line, level) {
  const el = $("console");
  if (el.dataset.virgin !== "1") { el.innerHTML = ""; el.dataset.virgin = "1"; }
  const div = document.createElement("div");
  div.className = level === "warn" ? "text-amber-300"
                : level === "tx" ? "text-cyan-300 font-bold"   // what we sent
                : "text-slate-400";
  div.textContent = line;
  el.appendChild(div);
  while (el.childElementCount > 400) el.removeChild(el.firstChild);
  el.scrollTop = el.scrollHeight;
}

// ---------------------------------------------------------------------------
// Socket
// ---------------------------------------------------------------------------

function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);

  ws.onmessage = (ev) => {
    const m = JSON.parse(ev.data);
    if (m.type === "telemetry") {
      if (!m.rows.length) return;
      for (const row of m.rows) pushRow(row);
      updateTiles(m.rows[m.rows.length - 1]);
      redraw();
    } else if (m.type === "console") {
      pushConsole(m.line, m.level);
    } else if (m.type === "status") {
      applyStatus(m);
    }
  };

  ws.onclose = () => {
    pill($("pill-link"), "○ SERVER DOWN", "bad");
    setTimeout(connect, 1200);
  };
  ws.onerror = () => ws.close();
}

const send = (msg) => { if (ws && ws.readyState === 1) ws.send(JSON.stringify(msg)); };
const cmd = (c) => send({ type: "cmd", cmd: c });

// ---------------------------------------------------------------------------
// Controls
// ---------------------------------------------------------------------------

// ARM is two-step. Everything that makes the cube safer is one step.
$("btn-arm").onclick = () => {
  const b = $("btn-arm");
  if (!armPending) {
    armPending = true;
    b.textContent = "CONFIRM ARM";
    b.className = b.className.replace("bg-emerald-600/90 hover:bg-emerald-500",
                                      "bg-amber-500 hover:bg-amber-400 text-black");
    $("arm-hint").textContent = "click again within 3 s to arm";
    armPendingTimer = setTimeout(resetArm, 3000);
  } else {
    cmd("a1");
    resetArm();
  }
};
function resetArm() {
  clearTimeout(armPendingTimer);
  armPending = false;
  const b = $("btn-arm");
  b.textContent = "ARM";
  b.className = "btn w-full px-3 py-3 rounded bg-emerald-600/90 hover:bg-emerald-500 " +
                "text-white font-bold text-sm tracking-[0.15em]";
}

$("btn-disarm").onclick = () => { cmd("a0"); resetArm(); };
// Halt is 'p' on the legacy build and 'h' on AutoTrim -- the server tells us
// which, because sending the wrong letter is silently accepted as something
// else entirely (a bare h is a keepalive on legacy, h0 is resume on AutoTrim).
$("btn-halt").onclick   = () => { if (haltCmd) cmd(haltCmd + "1"); resetArm(); };
$("btn-resume").onclick = () => { if (haltCmd) cmd(haltCmd + "0"); };
$("btn-resolve").onclick = () => cmd(family === "edge" ? "e" : "c");
$("btn-tare").onclick   = () => cmd("z1");
$("btn-untare").onclick = () => cmd("z0");

let gainTimer = null;
$("gain").oninput = (e) => {
  const v = parseFloat(e.target.value);
  $("gain-val").textContent = v.toFixed(3);
  clearTimeout(gainTimer);
  gainTimer = setTimeout(() => cmd("g" + v), 120);
};

$("btn-rec-start").onclick = () => send({ type: "record", action: "start" });
$("btn-rec-stop").onclick  = () => send({ type: "record", action: "stop" });
$("btn-clear").onclick = () => { $("console").innerHTML = ""; };

document.querySelectorAll(".mode-btn").forEach((b) => {
  b.onclick = () => {
    const m = b.dataset.mode;
    send({ type: "mode", schema: m === "auto" ? null : m });
    document.querySelectorAll(".mode-btn").forEach((o) => {
      const on = o === b;
      o.className = "mode-btn btn px-2 py-2 rounded text-[11px] font-semibold tracking-wide " +
        (on ? "bg-cyan-600/80 text-white glow-cyan" : "bg-slate-800 text-slate-400 hover:bg-slate-700");
    });
  };
});

// Space is the panic key: it disarms, wherever focus happens to be.
addEventListener("keydown", (e) => {
  if (e.code === "Space" && e.target.tagName !== "INPUT") {
    e.preventDefault();
    cmd("a0");
    resetArm();
  }
});

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------

document.querySelector('.mode-btn[data-mode="auto"]').click();

// A blank 3D panel and a broken 3D panel look identical, so say which it is.
try {
  cube = initCube($("cube"));
} catch (err) {
  const box = $("cube-error");
  box.classList.remove("hidden");
  box.textContent = "3D view unavailable: " + err.message +
    " — charts and controls still work.";
  console.error("[cubli] initCube failed", err);
}

buildCharts("corner");
setTileLabels("corner");
connect();
console.log("[cubli] dashboard booted — waiting for app.py on /ws");

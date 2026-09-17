/* app.js -- WebSocket client, live charts, metric tiles, command buttons.
 *
 * Talks to app.py over /ws. Three message kinds arrive: `telemetry` (a batch
 * of raw rows in the active schema's column order), `console` (one firmware
 * '#' line), and `status` (the server's view of the cube). Outbound is only
 * ever {type:"cmd"|"record"|"switch_mode"} -- the server validates every
 * command against the firmware's real grammar before anything reaches the wire.
 *
 * THE UI NEVER DECIDES WHAT THE CUBE IS DOING. Armed state comes from the
 * telemetry column (which also catches a self-disarm on a trip), and mode from
 * the firmware's own '# state=' line. Pressing a button changes what we SEND;
 * only the next status message changes what we SHOW.
 *
 * The previous Tailwind dashboard is preserved at /classic -- see
 * classic/app.js.
 *
 * NO 3D VIEW HERE (removed on request) -- see index.html's header comment.
 * The pivot readout box still shows the resolved corner/edge name, just not
 * animated.
 */

const $ = (id) => document.getElementById(id);

const WINDOW_S = 10;        // matches WINDOW_S in the matplotlib scripts
const CHART_HZ = 60;        // points/s kept per series; 250-500 Hz would melt Chart.js
const COL = { x: "#3b82f6", y: "#f5b544", z: "#9dbcff", k: "#ffffff", d: "#6f7787" };

let ws = null;
let schema = null;          // exact schema, e.g. 'corner_trim'
let family = null;          // 'corner' | 'edge' -- drives layout and 3D scene
let mode = null;            // what the FIRMWARE says its staged law is
let haltCmd = null;         // 'h' on the fused build; 'p' on the legacy corner
let armPolicy = "gate";     // 'gate' -> block above the gate; 'warn' -> warn only
let charts = [];            // [{chart, series:[{key,idx}], t0}]
let lastChartPush = -1e9;
let pivot = null;
let armGate = 0.5;
let envelope = 3.0;
let phiNorm = null;
let armed = false;
let stale = true;
let armPending = false;
let armPendingTimer = null;

// ---------------------------------------------------------------------------
// Chart plumbing
// ---------------------------------------------------------------------------

Chart.defaults.color = "#6f7787";
Chart.defaults.borderColor = "rgba(255,255,255,.07)";
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
           grid: { color: "rgba(255,255,255,.05)" } },
      y: { title: { display: true, text: unit },
           grid: { color: "rgba(255,255,255,.05)" } },
    },
    plugins: {
      legend: { display: true, position: "top", align: "end",
                labels: { boxWidth: 8, boxHeight: 8, padding: 6, usePointStyle: true } },
      tooltip: { enabled: false },
    },
  };
}

/** Column layouts, mirroring the panel split the matplotlib scripts use.
 *
 * TILT + WHEEL ONLY (on request) -- body rate and torque were dropped. Those
 * columns still exist in telemetry and in recorded CSVs; only these two live
 * charts were trimmed. This is also directly useful next to the LQR gain
 * sliders in the Control panel: tilt and wheel-speed are exactly what you
 * watch respond as you drag Tilt/Rate/Wheel gain. /classic still shows all
 * four if you want them. */
const LAYOUTS = {
  corner: [
    { title: "Tilt φ", unit: "deg", series: [
      { label: "φx", idx: 1, color: COL.x }, { label: "φy", idx: 2, color: COL.y },
      { label: "φz", idx: 3, color: COL.z }, { label: "|φ|", idx: -1, color: COL.k, width: 2 }] },
    { title: "Wheels ρ (dashed = 5 s low-pass)", unit: "rad/s", series: [
      { label: "ρx", idx: 7, color: COL.x }, { label: "ρy", idx: 8, color: COL.y },
      { label: "ρz", idx: 9, color: COL.z },
      { label: "ρx_lp", idx: 10, color: COL.x, dash: [4, 3] },
      { label: "ρy_lp", idx: 11, color: COL.y, dash: [4, 3] },
      { label: "ρz_lp", idx: 12, color: COL.z, dash: [4, 3] }] },
  ],
  edge: [
    { title: "Tilt θ", unit: "deg", series: [{ label: "θ", idx: 1, color: COL.x, width: 2 }] },
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
    wrap.className = "panel";
    wrap.innerHTML =
      `<h2>${spec.title}</h2><div class="chart-wrap"><canvas class="chart"></canvas></div>`;
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
  } else {
    phiNorm = Math.abs(row[1]);
    $("m-a").innerHTML = `<span style="color:${COL.x}">${f(row[1], 3)}</span>`;
    $("m-b").innerHTML = `<span style="color:${COL.y}">${f(row[2], 1)}</span>`;
    $("m-c").innerHTML = `<span style="color:${COL.x}">${f(RPM(row[7]), 0)}</span>`;
    $("m-d").innerHTML = `<span style="color:${COL.x}">${f(row[3], 4)}</span> / ` +
                         `<span style="color:${COL.y}">${f(row[4], 4)}</span>`;
  }

  // |phi| is shown against whichever threshold actually matters for this mode:
  // the arm gate where one exists, the validated recovery envelope where the
  // firmware has no gate at all. Green means "inside it".
  const ref = armPolicy === "warn" ? envelope : armGate;
  const el = $("m-phi");
  el.textContent = f(phiNorm, 3);
  el.classList.toggle("in", phiNorm < ref);

  const pct = Math.min(100, (phiNorm / Math.max(ref, 1e-6)) * 100);
  const bar = $("phi-bar");
  bar.style.width = pct + "%";
  bar.style.background = phiNorm < ref ? "var(--green)"
                       : pct > 300 ? "var(--red)" : "var(--amber)";
}

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------

function pill(el, text, tone) {
  el.className = "pill" + (tone ? " " + tone : "");
  el.textContent = text;
}

function applyStatus(st) {
  // Corner widths (21/26/29/33/36) share one layout, so switch on the family;
  // the exact schema is only for identification.
  if (st.family && st.family !== family) {
    family = st.family;
    buildCharts(family);
    setTileLabels(family);
    lastChartPush = -1e9;
  }
  schema = st.schema;
  haltCmd = st.halt_cmd;
  armPolicy = st.arm_policy || "gate";
  armGate = st.arm_gate_deg;
  envelope = st.corner_envelope_deg ?? 3.0;
  armed = !!st.armed;
  stale = !!st.stale;
  mode = st.mode;

  // ---- header pills ----
  pill($("pill-link"), st.stale ? "NO LINK" : `${f(st.rate_hz, 0)} Hz`,
       st.stale ? "" : "live");

  const stateText = armed
    ? `ARMED · ${(mode || "?").toUpperCase()}`
    : st.halted ? "HALTED"
    : `DISARMED · ${(mode || "?").toUpperCase()}`;
  pill($("pill-state"), stateText, armed ? "bad" : st.halted ? "warn" : "");
  $("pill-state").classList.toggle("blink", armed);

  // Session totals, not per-file counts -- recorded_rows resets to 0 at
  // every mode-switch file roll (a fresh CSV starts empty), which used to
  // read as "it stopped". recorded_rows_session/recorded_files never reset
  // mid-session, so THIS is what actually proves a Start...Stop bracket
  // survived a mode switch rather than restarting.
  const rec = $("pill-rec");
  rec.classList.toggle("hide", !st.recording);
  if (st.recording) pill(rec, `REC ${st.recorded_rows_session}`, "warn");
  $("rec-status").textContent = st.recording
    ? `${st.recording} — ${st.recorded_rows_session} rows` +
      (st.recorded_files > 1 ? ` across ${st.recorded_files} files` : "")
    : "idle";
  $("btn-rec-start").disabled = !!st.recording || st.read_only;
  $("btn-rec-stop").disabled  = !st.recording;

  // ---- status panel ----
  const big = $("st-state");
  // cube_state is the firmware's own word for it, which is more specific than
  // the armed flag (EDGE_BALANCE vs CORNER_BALANCE). Fall back to the flag for
  // a build old enough not to send a state line.
  big.textContent = armed ? (st.cube_state || "ARMED")
                  : st.halted ? "HALTED" : "DISARMED";
  big.className = "state-big" + (armed ? " armed" : st.halted ? " halted" : "");
  $("st-sub").textContent = st.stale
    ? "link stale — no packets"
    : `${st.schema || "?"}${st.schema_locked ? " (locked)" : ""}` +
      (st.read_only ? " · monitor mode" : "");

  $("m-gate").textContent = armPolicy === "warn"
    ? `no arm gate · envelope ${f(envelope, 1)} deg`
    : `gate ${f(armGate, 2)} deg (${st.arm_gate_source})`;

  $("m-rate").textContent = st.stale ? "—" : f(st.rate_hz, 0) + " Hz";
  $("m-age").textContent = st.link_age_s === null ? "—" : f(st.link_age_s * 1000, 0) + " ms";
  $("m-pkts").textContent = st.packets;
  $("m-gain").textContent = f(st.gain, 2);

  // The firmware is the authority on its own gate, and the copies in the tree
  // disagree. If it reports something the generated gains cannot justify, say
  // so rather than drawing a reassuring green bar.
  const gw = $("gate-warn");
  gw.classList.toggle("show", !!st.arm_gate_suspect);
  if (st.arm_gate_suspect) {
    gw.textContent =
      `⚠  firmware reports ARM_GATE = ${f(armGate, 2)} deg. cubli_gains.h ` +
      `specifies 0.50 deg. CubliBalance.ino carries 0.1672664619 rad ` +
      `(9.58 deg) forward verbatim from EdgeBalance_WiFi.ino — a suspected ` +
      `typo of 0.00872664619, kept for a faithful port. Verify before arming.`;
  }

  // ---- pivot ----
  if (st.pivot && (!pivot || pivot.name !== st.pivot.name)) {
    pivot = st.pivot;
    $("pivot-kind").textContent = pivot.kind === "edge" ? "Edge" : "Corner";
    $("pivot-val").textContent = pivot.gB
      ? `${pivot.name} · place offset ${f(pivot.place_offset_deg, 3)}°`
      : `${pivot.name} (unrecognised)`;
    $("pivot-readout").classList.remove("stale");
  }

  // ---- mode buttons ----
  // Disabled while armed: the firmware would force-disarm, and a control that
  // silently does something bigger than it says is worse than a disabled one.
  for (const which of ["edge", "corner"]) {
    const b = $("btn-mode-" + which);
    const isOn = mode === which;
    b.disabled = armed || stale || st.read_only;
    b.className = "btn" + (isOn ? " on" : "");
  }
  $("mode-hint").textContent =
      st.read_only ? "monitor mode — mode switch blocked (--read-only)"
    : armed        ? "disarm before switching mode"
    : stale        ? "no link"
    : mode         ? `staged: ${mode.toUpperCase()}`
    : "waiting for the firmware's state line";
  $("mode-hint").classList.toggle("warn", armed);

  // ---- mode-specific controls ----
  const isEdge = family === "edge";
  $("trim-panel").classList.toggle("hide", !isEdge);
  $("key-resolve").textContent = isEdge ? "e" : "c";
  $("btn-halt").disabled = !haltCmd || stale;
  $("btn-resume").disabled = !haltCmd || stale;
  $("btn-resolve").disabled = stale;

  // ---- arm ----
  updateArmButton(st);
}

/** ARM's enabled state and hint, which differ by arm_policy.
 *
 * 'gate' (edge): the firmware refuses above the gate, so we do too -- the
 * button greys out and says why.
 * 'warn' (corner): the firmware has NO gate. Blocking here would contradict it
 * and make ARM look broken during exactly the bring-up it was unblocked for,
 * so the button stays live and the hint turns amber past the envelope. The
 * two-step confirm is the interlock, not a threshold. */
function updateArmButton(st) {
  if (armPending) return;   // mid-confirm; leave the button alone

  const gated = armPolicy === "gate";
  const outside = phiNorm !== null && phiNorm >= (gated ? armGate : envelope);
  const canArm = !st.read_only && !armed && !stale && phiNorm !== null &&
                 !(gated && outside);

  const b = $("btn-arm");
  b.disabled = !canArm;
  b.className = "btn go big";
  b.innerHTML = 'Arm<span class="key">a1</span>';

  const hint = $("arm-hint");
  hint.textContent =
      st.read_only ? "monitor mode — arming blocked (--read-only)"
    : armed        ? "armed — DISARM or E-STOP to stop"
    : st.halted    ? "halted — press RESUME first"
    : stale        ? "no link"
    : phiNorm === null ? "waiting for telemetry"
    : gated && outside ? `|φ| ${f(phiNorm, 3)} ≥ ${f(armGate, 2)} deg gate — too far`
    : outside      ? `|φ| ${f(phiNorm, 3)} deg — OUTSIDE the ${f(envelope, 1)} deg `
                     + `validated envelope. No arm gate on this build.`
    : `|φ| ${f(phiNorm, 3)} deg — ready`;
  hint.classList.toggle("warn", outside);
}

// ---------------------------------------------------------------------------
// Console
// ---------------------------------------------------------------------------

function pushConsole(line, level) {
  const el = $("console");
  if (el.dataset.virgin !== "1") { el.innerHTML = ""; el.dataset.virgin = "1"; }
  const div = document.createElement("div");
  div.className = level === "warn" ? "w" : level === "tx" ? "t" : "i";
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

  ws.onopen = () => pill($("pill-ws"), "SERVER", "ok");

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
    pill($("pill-ws"), "SERVER DOWN", "bad");
    pill($("pill-link"), "NO LINK", "");
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
    b.className = "btn confirm big";
    b.innerHTML = 'Confirm arm<span class="key">a1</span>';
    $("arm-hint").textContent = "click again within 3 s to arm";
    $("arm-hint").classList.add("warn");
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
  b.className = "btn go big";
  b.innerHTML = 'Arm<span class="key">a1</span>';
}

// Disarm is never gated, never confirmed, and never disabled -- including when
// the link looks stale, because "stale" is a 1 s heuristic and the cube may
// well still be listening.
const disarm = () => { cmd("a0"); resetArm(); };
$("estop").onclick = disarm;

$("btn-halt").onclick   = () => { if (haltCmd) cmd(haltCmd + "1"); resetArm(); };
$("btn-resume").onclick = () => { if (haltCmd) cmd(haltCmd + "0"); };
$("btn-resolve").onclick = () => cmd(family === "edge" ? "e" : "c");

// Mode switch: one message, and the server puts a0 in front of it. The button
// does not optimistically repaint -- `mode` only moves when the firmware's
// state line says so.
document.querySelectorAll("[data-mode]").forEach((b) => {
  b.onclick = () => {
    send({ type: "switch_mode", mode: b.dataset.mode });
    resetArm();
  };
});

let gainTimer = null;
$("gain").oninput = (e) => {
  const v = parseFloat(e.target.value);
  $("gain-val").textContent = v.toFixed(2);
  clearTimeout(gainTimer);
  gainTimer = setTimeout(() => cmd("g" + v), 120);
};

// Edge trim. Sent only on Send, not on every keystroke -- 'o' moves the
// balance point of an armed cube.
const trimStep = (d) => {
  const el = $("trim-val");
  el.value = ((parseFloat(el.value) || 0) + d).toFixed(2);
};
$("btn-trim-dn").onclick = () => trimStep(-0.05);
$("btn-trim-up").onclick = () => trimStep(+0.05);
$("btn-trim-set").onclick = () => {
  const v = parseFloat($("trim-val").value);
  if (Number.isFinite(v)) cmd("o" + v.toFixed(2));
};

// Reset to the SAME hardcoded default the firmware boots with -- not a
// separate command, just "o<that number>". Keep this literal in sync with
// CubliBalance.ino's kEdgePhiOffsetDefaultDeg and schemas.py's
// EDGE_PHI_OFFSET_DEFAULT_DEG if either ever changes.
const EDGE_PHI_OFFSET_DEFAULT_DEG = -0.7;
$("key-trim-default").textContent = "o" + EDGE_PHI_OFFSET_DEFAULT_DEG.toFixed(2);
$("btn-trim-default").onclick = () => {
  $("trim-val").value = EDGE_PHI_OFFSET_DEFAULT_DEG.toFixed(2);
  cmd("o" + EDGE_PHI_OFFSET_DEFAULT_DEG.toFixed(2));
};

// LQR gain-term scales. Same debounce as the main gain slider; both modes'
// letters are always sent (the firmware refuses nothing here -- it just
// keeps two independent copies, see CubliBalance.ino SECTION 5), so these
// sliders stay live regardless of which mode is currently staged.
function wireGainScale(sliderId, valId, letter) {
  let timer = null;
  $(sliderId).oninput = (e) => {
    const v = parseFloat(e.target.value);
    $(valId).textContent = v.toFixed(2);
    clearTimeout(timer);
    timer = setTimeout(() => cmd(letter + v), 120);
  };
}
wireGainScale("p-scale", "p-val", "p");
wireGainScale("r-scale", "r-val", "r");
wireGainScale("w-scale", "w-val", "w");

$("btn-rec-start").onclick = () => send({ type: "record", action: "start" });
$("btn-rec-stop").onclick  = () => send({ type: "record", action: "stop" });
$("btn-clear").onclick = () => { $("console").innerHTML = ""; };

// Space is the panic key: it disarms, wherever focus happens to be.
addEventListener("keydown", (e) => {
  if (e.code === "Space" && e.target.tagName !== "INPUT") {
    e.preventDefault();
    disarm();
  }
});

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------

buildCharts("corner");
setTileLabels("corner");
connect();
console.log("[cubli] dashboard booted — waiting for app.py on /ws");

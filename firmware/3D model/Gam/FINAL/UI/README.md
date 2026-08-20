# UI — Cubli wireless console

Browser UI served **by the XIAO** (HTTP :80, WebSocket :81), plus tools to embed
the page into PROGMEM and (later) compile firmware into `../builds/`.

## Layout

| Path | Purpose |
|---|---|
| `web/index.html` | Single-page console (charts, commands, OTA client) |
| `web/fonts/montserrat.woff2` | Montserrat Variable, Latin subset (~20 KB), inlined at build time |
| `tools/embed_web.py` | Inline the fonts, gzip `web/index.html` → `../firmware/xiao/console_host/web_index.h`; also `--serve` for a browser preview |
| `tools/build.py` | Stub documenting compile → `../builds/teensy/<id>/latest.hex` |
| `docs/` | Layout and flashing notes |

## Look and feel

Black, white and dark blue, set in Montserrat. **Colour is an alarm, not
decoration**: the console stays monochrome while everything is nominal, and
amber/red appear only for a sagging pack, an armed cube or a failed flash.
White marks state (which mode the cube is in), blue marks the action you can
take. Keep new controls monochrome — spending red on a button that is merely
important is what makes real warnings invisible.

The font is served from the device, never from Google. `web/index.html`
references `fonts/montserrat.woff2` with a plain relative URL; `embed_web.py`
rewrites that into a `data:` URI so the XIAO serves the whole console as one
blob. A machine that already has Montserrat installed uses its own copy via
`local()`.

Only Latin is subsetted. The Greek symbols in the chart legends (φ, ρ, θ) and
the status glyphs fall through to the monospace stack, which is why every label
using them is `.mono`.

## Look at it

The same console and the same tool are mirrored under
`firmware/CubliUI/MODE_B_WIRELESS_CONSOLE/` (`web/`, `tools/`, `xiao/`); every
command below works from that `tools/` directory too.

### No hardware — just see the UI

```
cd "firmware/3D model/Gam/FINAL/UI/tools"
python embed_web.py --serve
```

Then open <http://127.0.0.1:8000/>. Use `--port 8080` if 8000 is taken, and
Ctrl-C to stop. This serves the **inlined** page — byte-for-byte what the XIAO
would send — so what you see is what you will flash. Nothing is written; the
committed header is untouched.

With no cube attached the header shows `WS DOWN` and every control is disabled,
which is correct: the console refuses to offer buttons it cannot deliver. To
see the panels populated, paste this into the browser console:

```js
(function () {                       // wrapped so you can paste it repeatedly
  if (ws) { ws.onclose = null; ws.close(); }
  ws = { readyState: 1, send: () => {}, close: () => {} };
  connect = () => {};                // stop the 1.2 s reconnect undoing this
  wsUp = true; pill($('p-ws'), 'WS', 'ok');
  onSys({ uart: 1, rate: 250, vbat: 23.6, armed: 0, mode: 2 });
  noteCornerFromLog('# corner resolved: [+1,+1,+1]  place_offset=0.714 deg');
  let t = 0;
  for (let i = 0; i < 700; i++) {
    t += 20; const s = t / 1000;
    onTelemetry([t, 0.9*Math.sin(s*1.7), 0.7*Math.sin(s*1.2+1), 0.5*Math.sin(s*2.3+2),
                 0,0,0, 14*Math.sin(s*0.8), 11*Math.sin(s*0.6+1), 8*Math.sin(s*1.1+2),
                 0,0,0,0,0,0,0,0,0, 0, 0]);
  }
  refreshButtons();
})();
```

Then `setArmed(true); setBattery(18.6)` to see the alarm palette: the ARMED
pill goes red and blinks, the pack reads `BELOW FLOOR — land it`, and ARM and
CALIBRATE lock out. Reload to get back to the honest disconnected state.

### Local page, real cube

Same command, but type the XIAO's IP into the host box in the header and press
CONNECT. The box appears automatically on `localhost` and on `file://`, because
in those cases the page cannot infer the board from its own URL. The IP is
remembered in `localStorage`. This is the fast loop for UI work: edit
`web/index.html`, reload, no reflash.

### The real thing

Flash the XIAO and browse to `http://<xiao-ip>/`. No host box — the page is
being served by the board, so it already knows.

> Opening `web/index.html` by double-clicking works, but Chrome gives every
> `file://` page an opaque origin and then blocks the webfont as cross-origin,
> so you get the fallback face instead of Montserrat. Use `--serve` when the
> typography is what you are judging.

## Commands the console can send

| Button | Wire | Effect |
|---|---|---|
| CALIBRATE CORNER | `c` | Re-runs corner identification: dots the gravity estimate against all 8 body corners, latches the closest, and that selects the gain matrix. The resolved corner is echoed on the log channel and latched into the panel readout. |
| RE-RESOLVE EDGE | `e` | Edge-family equivalent; the same button, relabelled when the telemetry width says `edge`. |
| ARM / CONFIRM | `a1` | Two-step. The firmware still applies its own tilt gate on top. |
| E-STOP | `a0` ×3 | Repeated because one dropped packet must not be the difference between stopped and not. |
| mode buttons | `{"t":"mode","m":0..2}` | Every change transits through IDLE. |

## PreFINAL vs this UI

- **FINAL/UI** is the lean console embedded on the XIAO.
- **PreFINAL/WIFIMODE/dashboard** remains the full Python FastAPI dashboard; it was not moved. Both can share the same UDP telemetry path.

## Regenerate `web_index.h`

Required before flashing, after editing `web/index.html` **or** anything under
`web/fonts/` — the digest covers the inlined page, so a font swap makes the
header stale too.

```
cd "firmware/3D model/Gam/FINAL/UI/tools"
python embed_web.py
python embed_web.py --check
```

`--check` writes nothing and exits 1 if the committed header does not match the
current page; it is the pre-flash guard against shipping a stale console.

## USB vs wireless

- Flash XIAO and non-OTA Teensy sketches over **USB**.
- **Wireless** Teensy updates go through the console OTA flow (requires an `ota_capable` Teensy image; see `../builds/manifest.json`).
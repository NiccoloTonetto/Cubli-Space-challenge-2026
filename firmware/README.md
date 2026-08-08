# Firmware

One folder per hardware target, staged the same way the physical project is
staged (2D panel first, 3D cube once the panel result validates the
pipeline — see [`../docs/dynamics/1D-Jig-to-3D-Cube-Strategy.md`](../docs/dynamics/1D-Jig-to-3D-Cube-Strategy.md)).

| Folder | Status | What it is |
|---|---|---|
| [`2D model/panel-bringup/`](2D%20model/panel-bringup/) | **Working** — balances on hardware | Teensy 4.1 + moteus-n1 + BMI270, Stage 1 planar panel, staged Stage 0→5 bring-up |
| `3D model/` | Not started | 3D cube firmware — will land here when that phase begins |

**Read [`Firmware Lessons — 2D Panel to 3D Cube.md`](Firmware%20Lessons%20—%202D%20Panel%20to%203D%20Cube.md)
before starting the cube firmware.** It's the condensed list of bugs found,
how they were fixed, and — just as importantly — which parts of the panel's
approach do and don't carry over to three wheels and a real attitude
estimate. Two of the bugs it documents cost real bring-up time on the panel
and are structurally guaranteed to reappear per-wheel on the cube if skipped.

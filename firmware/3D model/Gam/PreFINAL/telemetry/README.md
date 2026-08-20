# telemetry/ — every recording the FINAL tree produces

Two modes, two folders. They are different wire formats and they are read with
different tools, so they are not kept in one heap.

```
telemetry/
├── plot/     telemetry_edge_<stamp>.csv    PLOTMODE — comma-delimited
│             telemetry_corner_<stamp>.csv
└── serial/   session_<tag>_<stamp>.log     SERIALMONITORMODE — tab-delimited
```

`plot/` holds both builds' recordings, so the name says which one wrote it:
`_edge_` is 10 columns, `_corner_` is 21. The plotter does not rely on that —
it detects the format from the column count — but a directory listing should
answer "which run is this?" without opening the file.

| | `plot/` | `serial/` |
|---|---|---|
| Written by | [`WIFIMODE/telemetry_python_wifi.py`](../WIFIMODE/telemetry_python_wifi.py) (edge), [`_corner.py`](../WIFIMODE/telemetry_python_wifi_corner.py) (corner) | [`WIFIMODE/terminal_wifi.py`](../WIFIMODE/terminal_wifi.py) |
| When | on closing the live plot window | continuously, line by line, unless `--no-log` |
| Delimiter | comma | tab |
| Columns | 10 (edge) or 21 (corner) | 13 / 18 / 21, set by the bring-up stage |
| Header row | yes | yes, from the firmware |
| Plottable | always | **Stage 5 only** (21 columns) |

`serial/` also holds firmware console output — command echoes, `z1` tare
readbacks, arm refusals, trip reasons — interleaved with the data rows. That is
the point of the mode: it is what the Serial Monitor would have shown. Stage 1's
13 and Stages 2/3's 18 columns are outside `plot_session_csv.py`'s 10/21
auto-detection, so those logs are records to read, not to plot.

## Opening one

```bash
cd ..                          # FINAL/
python plot_session_csv.py     # press d for the newest, f to pick from a list
```

`d` takes the most recent recording of either mode — after a session, the one
you just made. `f` lists both folders separately, tags each line `EDGE` /
`CORNER` / `-` (not plottable) from the file's own first data row, and accepts
a list number, a filename, or just the timestamp: `014914` is enough to name
a file.

A filename works as an argument too, from anywhere in the tree:

```bash
python plot_session_csv.py 014914 --cols tilt
python plot_session_csv.py session_corner_20260817_222739.log --list
```

Full plotter usage is in the [FINAL README](../README.md#after-a-run-plot_session_csvpy).

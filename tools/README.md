# DSO-510 PC tools

Host-side tools for the custom firmware's USB streaming: a live plot
(`scope_live.py`), a reconnecting serial client (`scopeclient.py`) and the
stdlib-only protocol library (`dsoproto.py`).

## Install

```sh
python3 -m venv .venv
.venv/bin/pip install -r tools/requirements.txt
```

On Linux the serial port belongs to the `dialout` group (`uucp` on Arch).
Add yourself once, then log out and back in:

```sh
sudo usermod -aG dialout "$USER"
```

## Run the live viewer

```sh
.venv/bin/python3 tools/scope_live.py              # finds the scope by USB ID
.venv/bin/python3 tools/scope_live.py --port /dev/ttyACM0
```

The client reconnects by itself if the cable is pulled or the scope
soft-disconnects during a save. The status line shows `reconnecting…` while
it waits, and the last start command is re-sent once the scope returns.

## Modes

- **Frames** shows the latest acquisition frame, with time 0 at the trigger
  point, and the frame rate in the status line.
- **Logger** shows a rolling window (set by *Window*, up to 60 s) of the
  continuous sample stream at the chosen *Logger rate*. The status line shows
  the sample count, the measured S/s and the number of dropped samples. A drop
  appears as a gap in the trace. Long windows are decimated to min/max pairs
  so the display stays responsive at 500 kS/s.

*Mode* and *Logger rate* are locked while running; *Window* can change live.
Each Start (and each reconnect) begins a new logger session, and the plot is
cleared, because the scope restarts its sample index at 0.

## Recording

Tick *Record CSV* and pick a folder before or during a run.

- Frames: each frame is written as its own file in `frames-<time>/`,
  `frame_0001.csv`, `frame_0002.csv`, ... The format is the scope's own CSV.
- Logger: one file per session, `log-<time>-partN.csv`. Time 0 is the first
  sample of that session, so a new session (Start, or a reconnect) opens the
  next part. Dropped samples appear as `# gap=` comment lines, and a settings
  change within a session appears as a `# settings changed` line.

Recording only captures data from the moment it is ticked. Unticking the box
closes the file. Errors (for example an unwritable folder, or a scope error)
appear in red below the status line and clear after a few seconds.

## Reading a recording

`dsoproto.py` has no dependencies beyond the standard library, so you can
import it in your own scripts. A recorded CSV has `#` comment lines and a
`time_s,volts,adc_code` header:

```python
import csv, matplotlib.pyplot as plt
rows = [r for r in csv.reader(open("log-20261113-101500.csv")) if r and not r[0].startswith("#")][1:]
plt.plot([float(r[0]) for r in rows], [float(r[1]) for r in rows])
plt.xlabel("time (s)"); plt.ylabel("volts"); plt.show()
```

(`matplotlib` is not in `requirements.txt`; install it separately.)

## Tests

No hardware or pyserial needed:

```sh
python3 -m unittest discover -s tools/tests -v
python3 -m py_compile tools/scope_live.py tools/scopeclient.py tools/dsoproto.py
```

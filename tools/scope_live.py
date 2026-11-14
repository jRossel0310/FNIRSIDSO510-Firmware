#!/usr/bin/env python3
"""Live plot of the DSO-510 stream (frames or logger mode), with CSV recording.

pip install -r tools/requirements.txt ; python3 tools/scope_live.py [--port /dev/ttyACM0]
"""
import argparse
import collections
import os
import sys
import time

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dsoproto as dp  # noqa: E402
from scopeclient import ScopeClient  # noqa: E402

RATES = [1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000]


class Window(QtWidgets.QWidget):
    def __init__(self, client):
        super().__init__()
        self.c = client
        self.fw = "?"
        self.logmeta = None
        self.log_t = collections.deque()
        self.log_v = collections.deque()
        self.frame_csv_dir = None
        self.frame_count = 0
        self.log_csv = None
        self.log_writer = None
        self.stats = {"frames": 0, "samples": 0, "drops": 0, "t0": time.time()}

        self.setWindowTitle("DSO-510 live")
        self.plot = pg.PlotWidget()
        self.plot.setLabel("bottom", "time", units="s")
        self.plot.setLabel("left", "voltage", units="V")
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.curve = self.plot.plot(pen=pg.mkPen("y", width=1), connect="finite")

        self.mode = QtWidgets.QComboBox(); self.mode.addItems(["Frames", "Logger"])
        self.rate = QtWidgets.QComboBox(); self.rate.addItems([str(r) for r in RATES]); self.rate.setCurrentText("10000")
        self.window_s = QtWidgets.QDoubleSpinBox(); self.window_s.setRange(0.1, 600); self.window_s.setValue(5); self.window_s.setSuffix(" s")
        self.btn = QtWidgets.QPushButton("Start")
        self.rec = QtWidgets.QCheckBox("Record CSV")
        self.status = QtWidgets.QLabel("connecting…")

        bar = QtWidgets.QHBoxLayout()
        for w in (QtWidgets.QLabel("Mode"), self.mode, QtWidgets.QLabel("Logger rate"), self.rate,
                  QtWidgets.QLabel("Window"), self.window_s, self.btn, self.rec):
            bar.addWidget(w)
        bar.addStretch(1)
        lay = QtWidgets.QVBoxLayout(self)
        lay.addLayout(bar)
        lay.addWidget(self.plot, 1)
        lay.addWidget(self.status)

        self.btn.clicked.connect(self.toggle)
        self.rec.toggled.connect(self.toggle_record)
        self.running = False
        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.tick); self.timer.start(30)

    # --- controls -------------------------------------------------------------------------------
    def toggle(self):
        if self.running:
            self.c.stop(); self.running = False; self.btn.setText("Start")
            return
        self.log_t.clear(); self.log_v.clear(); self.logmeta = None
        if self.mode.currentText() == "Frames":
            self.c.start("frames")
        else:
            self.c.start("log", int(self.rate.currentText()))
        self.stats.update(frames=0, samples=0, drops=0, t0=time.time())
        self.running = True; self.btn.setText("Stop")

    def toggle_record(self, on):
        if not on:
            if self.log_csv:
                self.log_csv.close(); self.log_csv = None; self.log_writer = None
            self.frame_csv_dir = None
            return
        d = QtWidgets.QFileDialog.getExistingDirectory(self, "Record CSV files into")
        if not d:
            self.rec.setChecked(False); return
        stamp = time.strftime("%Y%m%d-%H%M%S")
        if self.mode.currentText() == "Frames":
            self.frame_csv_dir = os.path.join(d, "frames-" + stamp); os.makedirs(self.frame_csv_dir, exist_ok=True)
            self.frame_count = 0
        else:
            self.log_csv = open(os.path.join(d, "log-%s.csv" % stamp), "w", newline="")
            self.log_writer = dp.LogCsvWriter(self.log_csv, self.fw)
            if self.logmeta:
                self.log_writer.meta(self.logmeta)

    # --- data -------------------------------------------------------------------------------------
    def on_packet(self, p):
        if isinstance(p, dp.Info):
            self.fw = p.fields.get("fw", self.fw)
        elif isinstance(p, dp.Frame):
            self.stats["frames"] += 1
            m = p.meta
            n = len(p.samples)
            t = (np.arange(n) - m.trig_index) / m.rate_hz
            v = (np.frombuffer(p.samples, dtype=np.uint8).astype(np.int32) - m.zero_code) * (m.uv_per_code() / 1e6)
            self.curve.setData(t, v)
            if self.frame_csv_dir:
                self.frame_count += 1
                with open(os.path.join(self.frame_csv_dir, "frame_%04d.csv" % self.frame_count), "w", newline="") as f:
                    f.write(dp.frame_csv_text(m, p.samples, self.fw))
        elif isinstance(p, dp.LogMeta):
            self.logmeta = p.meta
            if self.log_writer:
                self.log_writer.meta(p.meta)
        elif isinstance(p, dp.Log) and self.logmeta:
            m = self.logmeta
            n = len(p.samples)
            self.stats["samples"] += n
            t = (p.first_index + np.arange(n)) / m.rate_hz
            v = (np.frombuffer(p.samples, dtype=np.uint8).astype(np.int32) - m.zero_code) * (m.uv_per_code() / 1e6)
            self.log_t.extend(t.tolist()); self.log_v.extend(v.tolist())
            if self.log_writer:
                self.log_writer.log(p)
        elif isinstance(p, dp.Drop):
            self.stats["drops"] += p.count
            if self.logmeta:                          # break the line at the gap
                self.log_t.append(p.first_index / self.logmeta.rate_hz); self.log_v.append(float("nan"))
            if self.log_writer:
                self.log_writer.drop(p)
        elif isinstance(p, dp.Err):
            self.status.setText("scope: " + p.text)

    def tick(self):
        for p in self.c.packets(timeout=0):
            self.on_packet(p)
        if self.mode.currentText() == "Logger" and self.log_t:
            horizon = self.log_t[-1] - self.window_s.value()
            while self.log_t and self.log_t[0] < horizon:
                self.log_t.popleft(); self.log_v.popleft()
            step = max(1, len(self.log_t) // 20000)     # keep redraws fast
            t = np.fromiter(self.log_t, float)[::step]; v = np.fromiter(self.log_v, float)[::step]
            self.curve.setData(t, v)
        dt = max(1e-3, time.time() - self.stats["t0"])
        conn = "connected" if self.c.connected else "reconnecting…"
        if self.mode.currentText() == "Frames":
            self.status.setText("%s · %.1f frames/s · fw %s" % (conn, self.stats["frames"] / dt, self.fw))
        else:
            self.status.setText("%s · %d samples (%.0f S/s) · dropped %d · fw %s" % (
                conn, self.stats["samples"], self.stats["samples"] / dt, self.stats["drops"], self.fw))

    def closeEvent(self, e):
        if self.log_csv:
            self.log_csv.close()
        self.c.stop(); self.c.close()
        e.accept()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", help="serial port (default: find the scope by USB ID)")
    a = ap.parse_args()
    app = pg.mkQApp("DSO-510 live")
    w = Window(ScopeClient(port=a.port))
    w.resize(1100, 600); w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

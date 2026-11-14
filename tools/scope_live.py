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
MAX_POINTS = 20000        # logger curve points per redraw (min/max decimation above this)
MAX_WINDOW_S = 60
MAX_SAMPLES = 5_000_000   # rate x effective window is capped at this
ERR_SHOW_S = 5
FLUSH_S = 1.0


class Window(QtWidgets.QWidget):
    def __init__(self, client):
        super().__init__()
        self.c = client
        self.fw = "?"
        self.running = False
        self.run_mode = None          # "Frames" or "Logger", fixed at Start
        self.run_rate = None
        self.logmeta = None           # None = no logger session yet (or a new one is starting)
        self.chunks = collections.deque()   # logger: (time_s, volts) arrays; NaN volts mark a gap
        self.dirty = False
        self.conn_prev = client.connected
        self.seen_client_err = None
        self.err_until = 0.0
        self.rec_dir = None           # set while "Record CSV" is ticked
        self.rec_stamp = None
        self.frame_dir = None
        self.frame_count = 0
        self.log_csv = None
        self.log_writer = None
        self.log_part = 0
        self.last_flush = 0.0
        self.stats = {"frames": 0, "samples": 0, "drops": 0, "t0": time.time()}

        self.setWindowTitle("DSO-510 live")
        self.plot = pg.PlotWidget()
        self.plot.setLabel("bottom", "time", units="s")
        self.plot.setLabel("left", "voltage", units="V")
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.curve = self.plot.plot(pen=pg.mkPen("y", width=1), connect="finite")

        self.mode = QtWidgets.QComboBox(); self.mode.addItems(["Frames", "Logger"])
        self.rate = QtWidgets.QComboBox(); self.rate.addItems([str(r) for r in RATES]); self.rate.setCurrentText("10000")
        self.window_s = QtWidgets.QDoubleSpinBox(); self.window_s.setRange(0.1, MAX_WINDOW_S); self.window_s.setValue(5); self.window_s.setSuffix(" s")
        self.btn = QtWidgets.QPushButton("Start")
        self.rec = QtWidgets.QCheckBox("Record CSV")
        self.status = QtWidgets.QLabel("connecting…")
        self.err = QtWidgets.QLabel("")
        self.err.setStyleSheet("color: #c00;")

        bar = QtWidgets.QHBoxLayout()
        for w in (QtWidgets.QLabel("Mode"), self.mode, QtWidgets.QLabel("Logger rate"), self.rate,
                  QtWidgets.QLabel("Window"), self.window_s, self.btn, self.rec):
            bar.addWidget(w)
        bar.addStretch(1)
        lay = QtWidgets.QVBoxLayout(self)
        lay.addLayout(bar)
        lay.addWidget(self.plot, 1)
        lay.addWidget(self.status)
        lay.addWidget(self.err)

        self.btn.clicked.connect(self.toggle)
        self.rec.toggled.connect(self.toggle_record)
        self.window_s.valueChanged.connect(lambda _v: setattr(self, "dirty", True))
        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.tick); self.timer.start(30)

    # --- errors ----------------------------------------------------------------------------------
    def show_error(self, msg):
        self.err.setText(msg)
        self.err_until = time.time() + ERR_SHOW_S

    def record_failed(self, e):
        self.show_error("recording failed: %s" % e)
        self.rec.setChecked(False)    # toggle_record(False) closes any open log file

    # --- controls --------------------------------------------------------------------------------
    def set_run_controls(self, enabled):
        self.mode.setEnabled(enabled)
        self.rate.setEnabled(enabled)

    def clear_plot(self):
        self.chunks.clear()
        self.dirty = False
        self.curve.setData([], [])

    def new_session(self):
        """Scope logger index restarts at 0 on every LOG command: drop the old data."""
        self.logmeta = None
        self.clear_plot()

    def toggle(self):
        if self.running:
            self.c.stop(); self.running = False
            self.btn.setText("Start"); self.set_run_controls(True)
            return
        self.run_mode = self.mode.currentText()
        self.run_rate = int(self.rate.currentText())
        self.new_session()
        self.stats.update(frames=0, samples=0, drops=0, t0=time.time())
        if self.run_mode == "Frames":
            self.c.start("frames")
        else:
            self.c.start("log", self.run_rate)
        self.running = True
        self.btn.setText("Stop"); self.set_run_controls(False)

    def toggle_record(self, on):
        self.close_log()
        if not on:
            self.rec_dir = None
            return
        d = QtWidgets.QFileDialog.getExistingDirectory(self, "Record CSV files into")
        if not d:
            self.rec.setChecked(False); return
        self.rec_dir = d
        self.rec_stamp = time.strftime("%Y%m%d-%H%M%S")
        self.frame_dir = None; self.frame_count = 0; self.log_part = 0

    def close_log(self):
        if self.log_csv:
            try:
                self.log_csv.close()
            except OSError:
                pass
        self.log_csv = None; self.log_writer = None

    def roll_log(self, meta):
        """One log file per logger session: close the old part and open the next."""
        self.close_log()
        self.log_part += 1
        path = os.path.join(self.rec_dir, "log-%s-part%d.csv" % (self.rec_stamp, self.log_part))
        self.log_csv = open(path, "w", newline="")
        self.log_writer = dp.LogCsvWriter(self.log_csv, self.fw)
        self.log_writer.meta(meta)
        self.last_flush = time.time()

    def try_roll(self, meta):
        try:
            self.roll_log(meta)
        except OSError as e:
            self.record_failed(e)

    # --- data ------------------------------------------------------------------------------------
    @staticmethod
    def volts(m, samples):
        codes = np.frombuffer(samples, dtype=np.uint8).astype(np.int32)
        return (codes - m.zero_code) * (m.uv_per_code() / 1e6)

    def on_packet(self, p):
        if isinstance(p, dp.Info):
            self.fw = p.fields.get("fw", self.fw)
        elif isinstance(p, dp.Frame):
            self.on_frame(p)
        elif isinstance(p, dp.LogMeta):
            self.on_logmeta(p.meta)
        elif isinstance(p, dp.Log):
            self.on_log(p)
        elif isinstance(p, dp.Drop):
            self.on_drop(p)
        elif isinstance(p, dp.Err):
            self.show_error("scope: " + p.text)

    def on_frame(self, p):
        m = p.meta
        self.stats["frames"] += 1
        t = (np.arange(len(p.samples)) - m.trig_index) / m.rate_hz
        self.curve.setData(t, self.volts(m, p.samples))
        if self.rec_dir:
            try:
                if self.frame_dir is None:
                    self.frame_dir = os.path.join(self.rec_dir, "frames-" + self.rec_stamp)
                    os.makedirs(self.frame_dir, exist_ok=True)
                self.frame_count += 1
                with open(os.path.join(self.frame_dir, "frame_%04d.csv" % self.frame_count), "w", newline="") as f:
                    f.write(dp.frame_csv_text(m, p.samples, self.fw))
            except OSError as e:
                self.record_failed(e)

    def on_logmeta(self, meta):
        new_segment = self.logmeta is None or meta.rate_hz != self.logmeta.rate_hz
        self.logmeta = meta
        if new_segment:
            self.clear_plot()
        if new_segment or (self.rec_dir and self.log_writer is None):
            if self.rec_dir:
                self.try_roll(meta)
        elif self.log_writer:
            try:
                self.log_writer.meta(meta)
            except OSError as e:
                self.record_failed(e)

    def on_log(self, p):
        m = self.logmeta
        if m is None or not len(p.samples):
            return
        self.stats["samples"] += len(p.samples)
        t = (p.first_index + np.arange(len(p.samples))) / m.rate_hz
        self.chunks.append((t, self.volts(m, p.samples)))
        self.dirty = True
        if self.rec_dir and self.log_writer is None:     # Record ticked mid-session: open its file now
            self.try_roll(m)
        if self.log_writer:
            try:
                self.log_writer.log(p)
            except OSError as e:
                self.record_failed(e)

    def on_drop(self, p):
        self.stats["drops"] += p.count
        m = self.logmeta
        if m is not None:            # NaN point at the gap start breaks the trace there
            self.chunks.append((np.array([(p.first_index - p.count) / m.rate_hz]), np.array([np.nan])))
            self.dirty = True
        if self.log_writer:
            try:
                self.log_writer.drop(p)
            except OSError as e:
                self.record_failed(e)

    def effective_window(self):
        """Requested window, capped so that rate x window <= MAX_SAMPLES."""
        w = self.window_s.value()
        rate = self.logmeta.rate_hz if self.logmeta else self.run_rate
        return min(w, MAX_SAMPLES / rate) if rate else w

    def trim(self):
        """Keep only the last effective_window() seconds of logger data."""
        if not self.chunks:
            return
        horizon = self.chunks[-1][0][-1] - self.effective_window()
        while self.chunks and self.chunks[0][0][-1] < horizon:
            self.chunks.popleft()
        if self.chunks:
            t, v = self.chunks[0]
            i = int(np.searchsorted(t, horizon))
            if i:
                self.chunks[0] = (t[i:], v[i:])

    def draw_logger(self):
        if not self.chunks:
            self.curve.setData([], [])
            return
        t = np.concatenate([c[0] for c in self.chunks])
        v = np.concatenate([c[1] for c in self.chunks])
        n = len(t)
        if n > MAX_POINTS:           # per-bucket min/max keeps peaks; a NaN bucket stays NaN (gap)
            b = MAX_POINTS // 2
            idx = np.linspace(0, n, b + 1).astype(np.int64)[:-1]
            lo = np.minimum.reduceat(v, idx)
            hi = np.maximum.reduceat(v, idx)
            t0 = t[idx]
            t = np.column_stack((t0, t0)).ravel()
            v = np.column_stack((lo, hi)).ravel()
        self.curve.setData(t, v)

    def tick(self):
        now = time.time()
        conn = self.c.connected
        if self.running and conn and not self.conn_prev:
            self.new_session()        # reconnect: the client re-sent the start command
        self.conn_prev = conn
        for p in self.c.packets(timeout=0):
            self.on_packet(p)
        if self.c.last_error and self.c.last_error != self.seen_client_err:
            self.seen_client_err = self.c.last_error
            self.show_error(self.c.last_error)
        if self.run_mode == "Logger":
            self.trim()
            if self.dirty:
                self.draw_logger()
                self.dirty = False
        if self.log_csv and now - self.last_flush >= FLUSH_S:
            try:
                self.log_csv.flush()
            except OSError as e:
                self.record_failed(e)
            self.last_flush = now
        if self.err.text() and now > self.err_until:
            self.err.setText("")
        dt = max(1e-3, now - self.stats["t0"])
        conn_txt = "connected" if conn else "reconnecting…"
        if self.c.last_error:                       # reader thread has stopped for good
            self.status.setText("client stopped: %s" % self.c.last_error)
        elif not self.running:
            self.status.setText("stopped · bad crc %d · fw %s" % (self.c.bad_crc, self.fw))
        elif self.run_mode == "Frames":
            self.status.setText("%s · %.1f frames/s · bad crc %d · fw %s" % (
                conn_txt, self.stats["frames"] / dt, self.c.bad_crc, self.fw))
        else:
            eff = self.effective_window()
            cap = (" · window %.1f s (capped)" % eff) if eff < self.window_s.value() - 1e-9 else ""
            self.status.setText("%s · %d samples (%.0f S/s) · dropped %d · bad crc %d · fw %s%s" % (
                conn_txt, self.stats["samples"], self.stats["samples"] / dt, self.stats["drops"],
                self.c.bad_crc, self.fw, cap))

    def closeEvent(self, e):
        self.close_log()
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

"""Serial connection to the DSO-510: reader thread, decoding, auto-reconnect, commands."""
import queue
import threading
import time

import dsoproto

VID, PID = 0x2E3C, 0x5720


def find_port():
    """First CDC-ACM port of the scope (pyserial's list_ports), or None."""
    from serial.tools import list_ports
    for p in list_ports.comports():
        if p.vid == VID and p.pid == PID:
            return p.device
    return None


def _serial_opener(port):
    import serial
    return serial.Serial(port, 115200, timeout=0.05)


class ScopeClient:
    def __init__(self, port=None, opener=None, retry_s=0.5):
        self.port_name = port
        self.opener = opener or _serial_opener
        self.retry_s = retry_s
        self.q = queue.Queue()
        self.connected = False
        self.mode = None            # None, "frames" or "log"
        self.rate = None
        self._port = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _command(self):
        if self.mode == "frames":
            return b"FRAMES\n"
        if self.mode == "log":
            return b"LOG %d\n" % self.rate
        return None

    def _send(self, data):
        with self._lock:
            if self._port is not None:
                try:
                    self._port.write(data)
                except OSError:
                    pass

    def start(self, mode, rate=None):
        self.mode, self.rate = mode, rate
        cmd = self._command()
        if cmd:
            self._send(cmd)

    def stop(self):
        self.mode = None
        self._send(b"STOP\n")

    def packets(self, timeout=0.05):
        out = []
        try:
            out.append(self.q.get(timeout=timeout))
            while True:
                out.append(self.q.get_nowait())
        except queue.Empty:
            pass
        return out

    def close(self):
        self._stop.set()
        self._t.join(timeout=2)
        with self._lock:
            if self._port is not None:
                try:
                    self._port.close()
                except OSError:
                    pass
                self._port = None

    def _run(self):
        while not self._stop.is_set():
            try:
                name = self.port_name or find_port()
                if not name:
                    raise OSError("scope not found")
                p = self.opener(name)
            except Exception:                       # noqa: BLE001 - any open failure means retry
                time.sleep(self.retry_s)
                continue
            dec = dsoproto.Decoder()
            with self._lock:
                self._port = p
            self.connected = True
            cmd = self._command()
            if cmd:
                self._send(cmd)                     # resume the previous mode
            try:
                while not self._stop.is_set():
                    data = p.read(4096)
                    if data:
                        for pk in dec.feed(data):
                            self.q.put(pk)
            except Exception:                       # noqa: BLE001 - unplug / soft-disconnect
                pass
            self.connected = False
            with self._lock:
                self._port = None
            try:
                p.close()
            except Exception:                       # noqa: BLE001
                pass
            if not self._stop.is_set():
                time.sleep(self.retry_s)

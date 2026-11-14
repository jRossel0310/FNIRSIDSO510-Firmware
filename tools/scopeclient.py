"""Serial connection to the DSO-510: reader thread, decoding, auto-reconnect, commands."""
import queue
import threading
import time

import dsoproto

VID, PID = 0x2E3C, 0x5720
PYSERIAL_MISSING = "pyserial not installed: pip install -r tools/requirements.txt"

try:
    import serial as _serial
    DISCONNECT_ERRORS = (OSError, _serial.SerialException)
except ImportError:                     # the client still imports; opening a port reports it
    DISCONNECT_ERRORS = (OSError,)


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
        self.last_error = None          # fatal: the reader thread has stopped
        self.mode = None                # None, "frames" or "log"; set with rate under _lock
        self.rate = None
        self._port = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _command(self):
        with self._lock:
            mode, rate = self.mode, self.rate
        if mode == "frames":
            return b"FRAMES\n"
        if mode == "log":
            return b"LOG %d\n" % rate
        return None

    def _send(self, data):
        with self._lock:
            if self._port is not None:
                try:
                    self._port.write(data)
                except OSError:
                    pass

    def start(self, mode, rate=None):
        with self._lock:
            self.mode, self.rate = mode, rate
        cmd = self._command()
        if cmd:
            self._send(cmd)

    def stop(self):
        with self._lock:
            self.mode, self.rate = None, None
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
                except DISCONNECT_ERRORS:
                    pass
                self._port = None

    def _run(self):
        while not self._stop.is_set():
            try:
                name = self.port_name or find_port()
                if not name:
                    raise OSError("scope not found")
                p = self.opener(name)
            except DISCONNECT_ERRORS:           # not there (yet): retry
                time.sleep(self.retry_s)
                continue
            except ImportError:                 # pyserial missing: retrying cannot help
                self.last_error = PYSERIAL_MISSING
                return
            except Exception as e:              # noqa: BLE001 - report, do not spin
                self.last_error = "open failed: %s: %s" % (type(e).__name__, e)
                return
            dec = dsoproto.Decoder()
            with self._lock:
                self._port = p
            self.connected = True
            cmd = self._command()
            if cmd:
                self._send(cmd)                 # resume the previous mode
            fatal = None
            try:
                while not self._stop.is_set():
                    data = p.read(4096)
                    if data:
                        for pk in dec.feed(data):
                            self.q.put(pk)
            except DISCONNECT_ERRORS:           # unplug / soft-disconnect
                pass
            except Exception as e:              # noqa: BLE001 - a bug, not a cable: stop and report
                fatal = "reader stopped: %s: %s" % (type(e).__name__, e)
            self.connected = False
            with self._lock:
                self._port = None
            try:
                p.close()
            except DISCONNECT_ERRORS:
                pass
            if fatal:
                self.last_error = fatal
                return
            if not self._stop.is_set():
                time.sleep(self.retry_s)

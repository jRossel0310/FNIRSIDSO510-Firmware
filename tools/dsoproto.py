"""DSO-510 live streaming protocol (spec: sub-project 4).

Packets: A5 5A | type u8 | len u16 LE | payload | crc16 LE (CRC-16/CCITT-FALSE over type, len, payload).
Stdlib only, so it can be reused in any script (e.g. plotting a recorded CSV with matplotlib).
"""
import struct
from dataclasses import dataclass, field

INFO, FRAME, LOGMETA, LOG, DROP, OK, ERR = range(1, 8)
SYNC = b"\xa5\x5a"


def crc16(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass
class Meta:
    rate_hz: int
    vdiv_mv: int
    coupling_dc: int
    probe_x10: int
    zero_code: int
    offset_pwm: int
    trig_mode: int = 0
    edge: int = 0
    triggered: int = 0
    trig_level_code: int = 0
    trig_index: int = 0

    def uv_per_code(self):
        return self.vdiv_mv * 40 * (10 if self.probe_x10 else 1)

    def volts(self, code):
        return (code - self.zero_code) * self.uv_per_code() / 1e6


@dataclass
class Info:
    fields: dict = field(default_factory=dict)


@dataclass
class Frame:
    meta: Meta
    samples: bytes


@dataclass
class LogMeta:
    meta: Meta


@dataclass
class Log:
    first_index: int
    samples: bytes


@dataclass
class Drop:
    first_index: int
    count: int


@dataclass
class Ok:
    text: str


@dataclass
class Err:
    text: str


def parse(ptype, p):
    if ptype == INFO:
        kv = {}
        for line in p.decode("ascii", "replace").splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                kv[k] = v
        return Info(kv)
    if ptype == FRAME:
        (rate, vdiv, dc, x10, mode, edge, trig, zero, level, tidx, pwm) = struct.unpack_from("<IHBBBBBhhiH", p, 0)
        (n,) = struct.unpack_from("<H", p, 21)
        m = Meta(rate, vdiv, dc, x10, zero, pwm, mode, edge, trig, level, tidx)
        return Frame(m, bytes(p[23:23 + n]))
    if ptype == LOGMETA:
        (rate, vdiv, x10, dc, zero, pwm) = struct.unpack_from("<IHBBhH", p, 0)
        return LogMeta(Meta(rate, vdiv, dc, x10, zero, pwm))
    if ptype == LOG:
        return Log(struct.unpack_from("<I", p, 0)[0], bytes(p[4:]))
    if ptype == DROP:
        a, b = struct.unpack_from("<II", p, 0)
        return Drop(a, b)
    if ptype == OK:
        return Ok(p.decode("ascii", "replace"))
    if ptype == ERR:
        return Err(p.decode("ascii", "replace"))
    return None


class Decoder:
    """Feed raw serial bytes; returns complete, CRC-checked packets. Resynchronises after junk."""

    def __init__(self):
        self.buf = bytearray()
        self.bad_crc = 0

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                keep = 1 if self.buf[-1:] == b"\xa5" else 0
                del self.buf[:len(self.buf) - keep]
                return out
            if i:
                del self.buf[:i]
            if len(self.buf) < 5:
                return out
            ptype = self.buf[2]
            n = self.buf[3] | self.buf[4] << 8
            if len(self.buf) < 5 + n + 2:
                return out
            body = bytes(self.buf[2:5 + n])
            crc = self.buf[5 + n] | self.buf[6 + n] << 8
            if crc16(body) != crc:
                self.bad_crc += 1
                del self.buf[:1]                    # skip this sync, look for the next one
                continue
            pk = parse(ptype, body[3:])
            del self.buf[:7 + n]
            if pk is not None:
                out.append(pk)


def _rdiv(n, d):
    return (n + d // 2) // d if n >= 0 else -((-n + d // 2) // d)


def _fixed(v, dec):
    sign = "-" if v < 0 else ""
    digits = str(abs(v)).rjust(dec + 1, "0")
    return sign + (digits[:-dec] + "." + digits[-dec:] if dec else digits)


def _row(t_ns, code, m):
    return "%s,%s,%d\n" % (_fixed(t_ns, 9), _fixed(_rdiv((code - m.zero_code) * m.uv_per_code(), 100), 4), code)


def frame_csv_text(m, samples, fw):
    """Exactly the scope's own CSV (fw/core/csv.c) for one frame."""
    modes = ["Auto", "Normal", "Single"]
    uv = m.uv_per_code()
    lines = [
        "# DSO-510 custom firmware %s\n" % fw,
        "# sample_rate_hz=%d\n" % m.rate_hz,
        "# volts_per_div=%s\n" % _fixed(m.vdiv_mv * 10 * (10 if m.probe_x10 else 1), 4),
        "# coupling=%s\n" % ("DC" if m.coupling_dc else "AC"),
        "# probe=%s\n" % ("10" if m.probe_x10 else "1"),
        "# trigger=%s,%s,%s\n" % (modes[m.trig_mode] if m.trig_mode < 3 else "Auto",
                                  "falling" if m.edge else "rising",
                                  _fixed(_rdiv((m.trig_level_code - m.zero_code) * uv, 100), 4)),
        "# trigger_index=%d\n" % m.trig_index,
        "# triggered=%d\n" % (1 if m.triggered else 0),
        "# cal=offset_pwm=%d,volts_per_code=%s\n" % (m.offset_pwm, _fixed(uv, 6)),
        "time_s,volts,adc_code\n",
    ]
    for i, c in enumerate(samples):
        lines.append(_row(_rdiv((i - m.trig_index) * 10**9, m.rate_hz), c, m))
    return "".join(lines)


class LogCsvWriter:
    """Streams a logger session to CSV: time 0 = first sample, '# gap=' lines at drops."""

    def __init__(self, f, fw):
        self.f = f
        self.fw = fw
        self.m = None
        self.next_index = 0

    def meta(self, m):
        uv = m.uv_per_code()
        if self.m is None:
            self.f.write("# DSO-510 custom firmware %s (PC log)\n" % self.fw)
            self.f.write("# sample_rate_hz=%d\n" % m.rate_hz)
            self.f.write("# volts_per_div=%s\n" % _fixed(m.vdiv_mv * 10 * (10 if m.probe_x10 else 1), 4))
            self.f.write("# coupling=%s\n" % ("DC" if m.coupling_dc else "AC"))
            self.f.write("# probe=%s\n" % ("10" if m.probe_x10 else "1"))
            self.f.write("# trigger=none\n")
            self.f.write("# cal=offset_pwm=%d,volts_per_code=%s\n" % (m.offset_pwm, _fixed(uv, 6)))
            self.f.write("time_s,volts,adc_code\n")
        else:
            self.f.write("# settings changed at %d: volts_per_div=%s,offset_pwm=%d,volts_per_code=%s\n" % (
                self.next_index, _fixed(m.vdiv_mv * 10 * (10 if m.probe_x10 else 1), 4), m.offset_pwm, _fixed(uv, 6)))
        self.m = m

    def drop(self, d):
        self.f.write("# gap=%d samples at %d\n" % (d.count, d.first_index - d.count))

    def log(self, lg):
        if self.m is None:
            return
        rate = self.m.rate_hz
        parts = []
        for k, c in enumerate(lg.samples):
            idx = lg.first_index + k
            parts.append(_row(_rdiv(idx * 10**9, rate), c, self.m))
        self.f.write("".join(parts))
        self.next_index = lg.first_index + len(lg.samples)

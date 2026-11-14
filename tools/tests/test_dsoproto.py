import io
import os
import struct
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import dsoproto as dp  # noqa: E402

VEC = os.path.join(HERE, "..", "..", "tests", "vectors")


def vec(name):
    with open(os.path.join(VEC, name), "rb") as f:
        return f.read()


class TestCrc(unittest.TestCase):
    def test_check_value(self):
        self.assertEqual(dp.crc16(b"123456789"), 0x29B1)


class TestVectors(unittest.TestCase):
    def one(self, name):
        d = dp.Decoder()
        pk = d.feed(vec(name))
        self.assertEqual(len(pk), 1)
        self.assertEqual(d.bad_crc, 0)
        return pk[0]

    def test_info(self):
        p = self.one("info.bin")
        self.assertIsInstance(p, dp.Info)
        self.assertEqual(p.fields["fw"], "1.0.0")
        self.assertEqual(p.fields["mode"], "idle")
        self.assertEqual(p.fields["rates"], "1000,2000,5000,10000,20000,50000,100000,200000,500000")

    def test_frame(self):
        p = self.one("frame.bin")
        self.assertIsInstance(p, dp.Frame)
        m = p.meta
        self.assertEqual((m.rate_hz, m.vdiv_mv, m.coupling_dc, m.probe_x10), (600000, 500, 1, 0))
        self.assertEqual((m.trig_mode, m.edge, m.triggered), (0, 0, 1))
        self.assertEqual((m.zero_code, m.trig_level_code, m.trig_index, m.offset_pwm), (46, 75, 3, 1630))
        self.assertEqual(list(p.samples), [46, 71, 96, 121, 146, 121, 96, 71])
        self.assertEqual(m.uv_per_code(), 20000)
        self.assertAlmostEqual(m.volts(146), 2.0)

    def test_logmeta(self):
        p = self.one("logmeta.bin")
        self.assertIsInstance(p, dp.LogMeta)
        m = p.meta
        self.assertEqual((m.rate_hz, m.vdiv_mv, m.probe_x10, m.coupling_dc, m.zero_code, m.offset_pwm),
                         (100000, 1000, 0, 1, 128, 2033))

    def test_log_drop_ok_err(self):
        p = self.one("log.bin")
        self.assertIsInstance(p, dp.Log)
        self.assertEqual(p.first_index, 1000)
        self.assertEqual(list(p.samples), list(range(120, 136)))
        p = self.one("drop.bin")
        self.assertEqual((p.first_index, p.count), (2048, 512))
        self.assertEqual(self.one("ok.bin").text, "LOG 100000")
        self.assertEqual(self.one("err.bin").text, "bad rate")


class TestStream(unittest.TestCase):
    def expect(self, pk):
        kinds = [type(p).__name__ for p in pk]
        self.assertEqual(kinds, ["Info", "Frame", "Log", "Drop", "Ok", "Err"])

    def test_resync_and_bad_crc(self):
        d = dp.Decoder()
        pk = d.feed(vec("stream.bin"))
        self.expect(pk)
        self.assertEqual(d.bad_crc, 1)              # the corrupted LOG was dropped

    def test_byte_by_byte(self):
        d = dp.Decoder()
        pk = []
        for b in vec("stream.bin"):
            pk += d.feed(bytes([b]))
        self.expect(pk)


def pkt(ptype, payload):
    """Build a CRC-valid packet on the wire."""
    body = bytes([ptype]) + struct.pack("<H", len(payload)) + payload
    return dp.SYNC + body + struct.pack("<H", dp.crc16(body))


FRAME_HDR = struct.pack("<IHBBBBBhhiH", 600000, 500, 1, 0, 0, 0, 0, 46, 75, 3, 1630)


class TestFaults(unittest.TestCase):
    def test_short_frame_payload_is_skipped(self):
        d = dp.Decoder()
        pk = d.feed(pkt(dp.FRAME, bytes(10)) + pkt(dp.OK, b"OK"))
        self.assertEqual([type(p).__name__ for p in pk], ["Ok"])
        self.assertEqual(d.bad_payload, 1)

    def test_frame_n_larger_than_samples_is_skipped(self):
        d = dp.Decoder()
        bad = pkt(dp.FRAME, FRAME_HDR + struct.pack("<H", 10) + bytes(4))   # n=10, only 4 sample bytes
        pk = d.feed(bad + pkt(dp.OK, b"OK"))
        self.assertEqual([type(p).__name__ for p in pk], ["Ok"])
        self.assertEqual(d.bad_payload, 1)

    def test_false_sync_is_skipped_without_bad_crc(self):
        d = dp.Decoder()
        junk = b"\x11\xa5\x5a\x09\x03\x00"                  # sync followed by unknown type 9
        pk = d.feed(junk + pkt(dp.OK, b"OK"))
        self.assertEqual([type(p).__name__ for p in pk], ["Ok"])
        self.assertEqual(d.bad_crc, 0)

    def test_over_long_length_is_false_sync(self):
        # a valid type with a length beyond that type's maximum must not stall the decoder
        for ptype, n in ((dp.LOG, 4 + 1025), (dp.LOGMETA, 13), (dp.DROP, 9), (dp.OK, 65), (dp.ERR, 65),
                         (dp.INFO, 513), (dp.FRAME, 23 + 8193), (dp.LOG, 0xFFFF)):
            d = dp.Decoder()
            junk = dp.SYNC + bytes([ptype]) + struct.pack("<H", n)
            pk = d.feed(junk + pkt(dp.OK, b"OK"))
            self.assertEqual([type(p).__name__ for p in pk], ["Ok"], (ptype, n))
            self.assertEqual(d.bad_crc, 0)

    def test_longest_valid_lengths_pass(self):
        d = dp.Decoder()
        log = pkt(dp.LOG, struct.pack("<I", 7) + bytes(1024))
        meta = pkt(dp.LOGMETA, struct.pack("<IHBBhH", 1000, 500, 0, 1, 128, 1630))
        drop = pkt(dp.DROP, struct.pack("<II", 10, 5))
        pk = d.feed(log + meta + drop + pkt(dp.OK, b"x" * 64))
        self.assertEqual([type(p).__name__ for p in pk], ["Log", "LogMeta", "Drop", "Ok"])


class TestCsv(unittest.TestCase):
    def test_drop_before_meta_writes_nothing(self):
        out = io.StringIO()
        w = dp.LogCsvWriter(out, "1.0.0")
        w.drop(dp.Drop(5, 3))
        self.assertEqual(out.getvalue(), "")
        w.meta(dp.Meta(rate_hz=1000, vdiv_mv=1000, coupling_dc=1, probe_x10=0, zero_code=128, offset_pwm=2033))
        self.assertTrue(out.getvalue().startswith("# DSO-510 custom firmware 1.0.0 (PC log)\n"))

    def test_frame_csv_matches_firmware(self):
        p = dp.Decoder().feed(vec("frame.bin"))[0]
        text = dp.frame_csv_text(p.meta, p.samples, "1.0.0")
        self.assertEqual(text, vec("frame.csv").decode("ascii"))

    def test_log_csv(self):
        m = dp.Meta(rate_hz=1000, vdiv_mv=1000, coupling_dc=1, probe_x10=0, zero_code=128, offset_pwm=2033)
        out = io.StringIO()
        w = dp.LogCsvWriter(out, "1.0.0")
        w.meta(m)
        w.log(dp.Log(0, bytes([128, 153])))
        w.drop(dp.Drop(5, 3))
        w.log(dp.Log(5, bytes([103])))
        self.assertEqual(out.getvalue(),
                         "# DSO-510 custom firmware 1.0.0 (PC log)\n"
                         "# sample_rate_hz=1000\n"
                         "# volts_per_div=1.0000\n"
                         "# coupling=DC\n"
                         "# probe=1\n"
                         "# trigger=none\n"
                         "# cal=offset_pwm=2033,volts_per_code=0.040000\n"
                         "time_s,volts,adc_code\n"
                         "0.000000000,0.0000,128\n"
                         "0.001000000,1.0000,153\n"
                         "# gap=3 samples at 2\n"
                         "0.005000000,-1.0000,103\n")


if __name__ == "__main__":
    unittest.main()

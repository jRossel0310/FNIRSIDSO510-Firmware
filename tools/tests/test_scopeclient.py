import os
import queue
import sys
import threading
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import scopeclient  # noqa: E402

VEC = os.path.join(HERE, "..", "..", "tests", "vectors")


class FakePort:
    """Serves bytes from a script; records writes; can 'unplug'."""

    def __init__(self, chunks, fail_after=None):
        self.chunks = list(chunks)
        self.written = b""
        self.fail_after = fail_after
        self.reads = 0
        self.closed = False

    def read(self, n):
        self.reads += 1
        if self.fail_after is not None and self.reads > self.fail_after:
            raise OSError("unplugged")
        if self.chunks:
            return self.chunks.pop(0)
        time.sleep(0.01)
        return b""

    def write(self, b):
        self.written += b

    def close(self):
        self.closed = True


class TestClient(unittest.TestCase):
    def test_decodes_and_sends_commands(self):
        with open(os.path.join(VEC, "stream.bin"), "rb") as f:
            data = f.read()
        port = FakePort([data[:50], data[50:]])
        c = scopeclient.ScopeClient(port="fake", opener=lambda p: port)
        c.start("frames")
        got = []
        deadline = time.time() + 2
        while len(got) < 6 and time.time() < deadline:
            got += c.packets(timeout=0.1)
        c.close()
        self.assertEqual([type(p).__name__ for p in got], ["Info", "Frame", "Log", "Drop", "Ok", "Err"])
        self.assertIn(b"FRAMES\n", port.written)

    def test_reconnects_and_resumes(self):
        ports = [FakePort([], fail_after=2), FakePort([])]
        opened = []

        def opener(p):
            if not ports:
                raise OSError("no device")
            opened.append(ports[0])
            return ports.pop(0)

        c = scopeclient.ScopeClient(port="fake", opener=opener, retry_s=0.05)
        c.start("log", 100000)
        deadline = time.time() + 3
        while len(opened) < 2 and time.time() < deadline:
            time.sleep(0.05)
        time.sleep(0.2)
        c.close()
        self.assertEqual(len(opened), 2)
        self.assertIn(b"LOG 100000\n", opened[1].written)   # resumed after the reconnect


if __name__ == "__main__":
    unittest.main()

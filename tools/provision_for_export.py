#!/usr/bin/env python3
"""Prepare a Core2 running UIFlow 2 so its flash can be exported as a
publishable M5Burner firmware image.

M5Burner has no way to compile a MicroPython project into a .bin. The supported
route is to set a device up exactly the way the end user should receive it, then
use M5Burner's "Firmware Exporter" to dump that device's whole flash to a .bin
and publish that. This script does the "set the device up" half:

  1. uploads factory_test.py to the device as /flash/main.py
  2. optionally deletes leftover files so they don't ship in the image
  3. sets the UIFlow2 boot option so the device runs main.py straight away
  4. verifies by hashing the file back off the device

Usage:
    python tools/provision_for_export.py --port COM5
    python tools/provision_for_export.py --port COM5 --clean
    python tools/provision_for_export.py --port COM5 --list

Requires pyserial (`pip install pyserial`).

Why not mpremote: UIFlow2 2.4.x does not complete mpremote 2.x's raw-paste
handshake ("could not enter raw repl"), so this talks the classic raw REPL
protocol instead.
"""

import argparse
import ast
import binascii
import hashlib
import os
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is required: python -m pip install pyserial")


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_SOURCE = os.path.join(REPO_ROOT, "factory_test.py")
REMOTE_MAIN = "/flash/main.py"

# Files UIFlow2 itself puts on /flash. Anything else is ours or left over.
STOCK_FLASH_ENTRIES = {
    "apps",
    "boot.py",
    "certificate",
    "libs",
    "main.py",
    "res",
    "README.md",
}


class RawRepl:
    """Classic raw REPL (Ctrl-A / Ctrl-D), no raw-paste."""

    def __init__(self, port, baud=115200, timeout=1):
        # Open without asserting DTR/RTS: on the Core2 those lines drive EN and
        # GPIO0, so a default open would reset the board on every connect.
        self.s = serial.Serial(baudrate=baud, timeout=timeout, dsrdtr=False, rtscts=False)
        self.s.port = port
        self.s.dtr = False
        self.s.rts = False
        self.s.open()
        self._pending = b""

    def close(self):
        self.s.close()

    def _read_until(self, ending, timeout=10):
        # One read can return the delimiter plus whatever follows it, so search
        # the whole buffer and keep the remainder for the next read.
        deadline = time.time() + timeout
        buf = self._pending
        while True:
            idx = buf.find(ending)
            if idx >= 0:
                self._pending = buf[idx + len(ending):]
                return buf[: idx + len(ending)]
            if time.time() >= deadline:
                break
            chunk = self.s.read(self.s.in_waiting or 1)
            if chunk:
                buf += chunk
        self._pending = b""
        raise TimeoutError("waiting for %r, got %r" % (ending, buf[-200:]))

    def drain(self, quiet_for=2.0, limit=30.0):
        """Swallow boot chatter until the device has been quiet for a moment."""
        deadline = time.time() + limit
        last = time.time()
        while time.time() < deadline and time.time() - last < quiet_for:
            if self.s.read(self.s.in_waiting or 1):
                last = time.time()
        self._pending = b""

    def enter(self, attempts=8):
        # UIFlow2 ignores Ctrl-C while its startup/network phase runs, so let the
        # boot finish, then keep asking for the raw REPL.
        self.drain()
        for _ in range(attempts):
            self.s.write(b"\r\x03\x03")
            time.sleep(0.3)
            self.s.reset_input_buffer()
            self._pending = b""
            self.s.write(b"\r\x01")
            try:
                self._read_until(b"raw REPL; CTRL-B to exit\r\n>", timeout=4)
                return
            except TimeoutError:
                time.sleep(1.0)
        raise TimeoutError(
            "device never entered the raw REPL. Close Thonny/UIFlow/M5Burner if "
            "one of them holds %s, then try again." % self.s.port
        )

    def exit(self):
        self.s.write(b"\r\x02")

    def exec(self, code, timeout=30):
        self.s.write(code.encode() + b"\x04")
        if self._read_until(b"OK", timeout=timeout)[-2:] != b"OK":
            raise IOError("device rejected the command")
        out = self._read_until(b"\x04", timeout=timeout)[:-1]
        err = self._read_until(b"\x04", timeout=timeout)[:-1]
        self._read_until(b">", timeout=timeout)
        if err:
            raise RuntimeError(err.decode(errors="replace").strip())
        return out.decode(errors="replace")


def wait_for_filesystem(repl, tries=10):
    """/flash mounts a moment after boot; don't race it."""
    for _ in range(tries):
        try:
            if "flash" in repl.exec("import os; print(os.listdir('/'))"):
                return
        except RuntimeError:
            pass
        time.sleep(1.0)
    raise RuntimeError("/flash never appeared")


def device_listing(repl):
    raw = repl.exec("import os; print(repr(sorted(os.listdir('/flash'))))").strip()
    return ast.literal_eval(raw)


def upload(repl, data, remote, chunk=512):
    repl.exec("import binascii")
    repl.exec("f = open(%r, 'wb')" % remote)
    for i in range(0, len(data), chunk):
        blob = binascii.hexlify(data[i : i + chunk]).decode()
        repl.exec("f.write(binascii.unhexlify('%s'))" % blob, timeout=20)
        done = min(i + chunk, len(data))
        sys.stdout.write("\r    uploading %6d / %d bytes" % (done, len(data)))
        sys.stdout.flush()
    repl.exec("f.close()")
    sys.stdout.write("\n")


def remote_sha256(repl, remote):
    code = (
        "import hashlib, binascii\n"
        "h = hashlib.sha256()\n"
        "n = 0\n"
        "f = open(%r, 'rb')\n"
        "while True:\n"
        "    b = f.read(512)\n"
        "    if not b: break\n"
        "    h.update(b); n += len(b)\n"
        "f.close()\n"
        "print(n, binascii.hexlify(h.digest()).decode())\n" % remote
    )
    size, digest = repl.exec(code, timeout=60).split()
    return int(size), digest


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="serial port of the Core2, e.g. COM5 or /dev/tty.wchusbserial*")
    ap.add_argument("--source", default=DEFAULT_SOURCE, help="script to install as main.py (default: factory_test.py)")
    ap.add_argument("--boot-option", type=int, default=0, choices=(0, 1, 2),
                    help="0 run main.py directly (default), 1 startup menu + network, 2 network only")
    ap.add_argument("--clean", action="store_true",
                    help="delete files on /flash that UIFlow2 did not ship, so they don't end up in the exported image")
    ap.add_argument("--list", action="store_true", help="just show what is on /flash and exit")
    args = ap.parse_args()

    with open(args.source, "rb") as fh:
        # Normalise to LF so the on-device file matches the repo byte for byte.
        data = fh.read().replace(b"\r\n", b"\n")
    local_digest = hashlib.sha256(data).hexdigest()

    print("Source : %s (%d bytes, sha256 %s)" % (args.source, len(data), local_digest[:16]))
    print("Port   : %s" % args.port)

    repl = RawRepl(args.port)
    try:
        print("\nConnecting (waiting out the UIFlow2 boot)...")
        repl.enter()
        wait_for_filesystem(repl)
        print("  connected:", repl.exec("import sys; print(sys.version)").strip())

        before = device_listing(repl)
        print("\n/flash before: %s" % ", ".join(before))
        if args.list:
            return 0

        if args.clean:
            strays = [f for f in before if f not in STOCK_FLASH_ENTRIES]
            for name in strays:
                # os.remove only handles files; fall back to rmdir for directories.
                repl.exec(
                    "import os\n"
                    "try:\n"
                    "    os.remove('/flash/%s')\n"
                    "except OSError:\n"
                    "    os.rmdir('/flash/%s')\n" % (name, name)
                )
                print("  removed /flash/%s" % name)
            if not strays:
                print("  nothing to clean")

        print("\nInstalling %s as %s" % (os.path.basename(args.source), REMOTE_MAIN))
        upload(repl, data, REMOTE_MAIN)

        size, digest = remote_sha256(repl, REMOTE_MAIN)
        if size != len(data) or digest != local_digest:
            raise SystemExit("VERIFY FAILED: device has %d bytes / %s" % (size, digest[:16]))
        print("  verified: %d bytes, sha256 %s" % (size, digest[:16]))

        repl.exec(
            "import esp32\n"
            "nvs = esp32.NVS('uiflow')\n"
            "nvs.set_u8('boot_option', %d)\n"
            "nvs.commit()\n"
            "print('boot_option =', nvs.get_u8('boot_option'))\n" % args.boot_option
        )
        print("  boot_option = %d (%s)" % (
            args.boot_option,
            {0: "runs main.py directly", 1: "startup menu + network", 2: "network only"}[args.boot_option],
        ))

        print("\n/flash after : %s" % ", ".join(device_listing(repl)))
        print("\nRebooting the device...")
        try:
            repl.exec("import machine; machine.reset()", timeout=2)
        except Exception:
            pass  # the reset kills the REPL before it can answer
    finally:
        try:
            repl.exit()
        finally:
            repl.close()

    print(
        "\nDevice is ready to export.\n"
        "Next: M5Burner -> USER CUSTOM -> Firmware Exporter -> pick %s -> Start,\n"
        "then USER CUSTOM -> Publish to upload the resulting .bin.\n"
        "See the 'Publishing to M5Burner' section of README.md." % args.port
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())

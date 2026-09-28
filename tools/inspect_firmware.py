#!/usr/bin/env python3
"""Audit an exported M5Burner firmware image before publishing it.

A firmware export is a dump of the device's entire flash, so it carries more
than the program: the NVS partition (where UIFlow 2 stores Wi-Fi credentials)
and any data still sitting in freed filesystem blocks come along for the ride.
Run this on the .bin M5Burner produced and read the warnings before uploading.

    python tools/inspect_firmware.py exported.bin

Reports:
  * the partition table
  * which NVS namespaces/keys exist -- names only, values are never read
  * whether the image contains the current factory_test.py
  * leftovers from previously deleted files

Exit status is 1 if anything worth reviewing before a public upload turned up.
"""

import argparse
import os
import struct
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_SOURCE = os.path.join(REPO_ROOT, "factory_test.py")

PART_TYPES = {0: "app", 1: "data"}
PART_SUBTYPES = {
    (1, 0): "ota", (1, 1): "phy", (1, 2): "nvs", (1, 0x81): "fat",
    (1, 0x82): "spiffs", (0, 0): "factory", (0, 0x10): "ota_0", (0, 0x11): "ota_1",
}

# NVS entry type codes (esp-idf nvs_types.hpp)
T_U8, T_STR, T_BLOB_DATA = 0x01, 0x21, 0x42
TYPE_NAMES = {
    T_U8: "u8", 0x11: "i8", 0x02: "u16", 0x12: "i16", 0x04: "u32", 0x14: "i32",
    0x08: "u64", 0x18: "i64", T_STR: "str", 0x41: "blob",
    T_BLOB_DATA: "blob_data", 0x48: "blob_idx",
}

# NVS keys that hold something you would not want to hand out publicly.
SENSITIVE_KEYS = {
    "ssid0", "ssid1", "ssid2",
    "pswd0", "pswd1", "pswd2",
    "server", "token", "apikey", "user_id", "mac",
}


def partitions(img):
    out = []
    off = 0x8000
    while True:
        entry = img[off : off + 32]
        if len(entry) < 32 or entry[:2] != b"\xaa\x50":
            return out
        _, ptype, subtype, addr, size, label, _flags = struct.unpack("<HBBII16sI", entry)
        out.append((label.rstrip(b"\x00").decode(errors="replace"), ptype, subtype, addr, size))
        off += 32


def parse_nvs(blob):
    """Yield (namespace, key, type). Stored values are never returned."""
    namespaces, found = {}, []
    for page_start in range(0, len(blob), 4096):
        page = blob[page_start : page_start + 4096]
        if len(page) < 4096 or page[0:4] == b"\xff\xff\xff\xff":
            continue
        bitmap = page[32:64]
        i = 0
        while i < 126:
            state = (bitmap[i // 4] >> ((i % 4) * 2)) & 0x3
            if state != 2:  # 2 == written; 0 == erased, 3 == never used
                i += 1
                continue
            e = page[64 + i * 32 : 96 + i * 32]
            ns, typ, span = e[0], e[1], e[2]
            key = e[8:24].split(b"\x00")[0]
            # A str/blob entry is followed by span-1 payload entries, also
            # marked written. Stepping over them keeps payload bytes from
            # being misread as key names.
            i += max(span, 1)
            if not key or any(c < 0x20 or c > 0x7E for c in key):
                continue
            key = key.decode()
            if ns == 0 and typ == T_U8:  # namespace declaration
                namespaces[e[24]] = key
                continue
            if typ == T_BLOB_DATA:
                continue
            found.append((ns, key, typ))
    return namespaces, sorted(set(found))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("image", help="the .bin produced by M5Burner's Firmware Exporter")
    ap.add_argument("--source", default=DEFAULT_SOURCE, help="script that should be inside it (default: factory_test.py)")
    args = ap.parse_args()

    with open(args.image, "rb") as fh:
        img = fh.read()
    print("image: %s (%d bytes, %.1f MB)\n" % (args.image, len(img), len(img) / 1048576))

    parts = partitions(img)
    if not parts:
        sys.exit("no partition table at 0x8000 - is this a full-flash export?")
    print("partition table")
    for label, ptype, subtype, addr, size in parts:
        print("  %-10s %-5s %-9s 0x%08x  %9d bytes" % (
            label, PART_TYPES.get(ptype, ptype),
            PART_SUBTYPES.get((ptype, subtype), hex(subtype)), addr, size))

    warnings = []

    for label, ptype, subtype, addr, size in parts:
        if PART_SUBTYPES.get((ptype, subtype)) != "nvs":
            continue
        namespaces, found = parse_nvs(img[addr : addr + size])
        print("\nNVS '%s' - key names only, values are never read" % label)
        for ns, key, typ in found:
            nsname = namespaces.get(ns, "ns#%d" % ns)
            flag = "  <-- sensitive" if key in SENSITIVE_KEYS else ""
            print("  %-14s %-18s %-10s%s" % (nsname, key, TYPE_NAMES.get(typ, hex(typ)), flag))
            if key in SENSITIVE_KEYS:
                warnings.append("NVS %s/%s is set" % (nsname, key))
        boot = [k for _, k, _ in found if k == "boot_option"]
        if not boot:
            warnings.append("boot_option is not set - the image will show the UIFlow 2 startup menu")

    print("\ncontent checks")
    with open(args.source, "rb") as fh:
        source = fh.read().replace(b"\r\n", b"\n")
    # main.py is stored uncompressed, so every distinctive line of the source
    # should appear verbatim in the image. Checking one or two lines is not
    # enough: two revisions of this script share most of their lines, so a
    # stale image still matches a handful of probes. Check them all and report
    # the ratio instead.
    probes = [ln.strip() for ln in source.split(b"\n")]
    probes = sorted({p for p in probes if len(p) > 24 and not p.startswith(b"#")})
    missing = [p for p in probes if p not in img]
    found = len(probes) - len(missing)
    name = os.path.basename(args.source)

    if not missing:
        print("  %-26s exact match (%d/%d lines)" % (name, found, len(probes)))
    else:
        # A line that straddles a FAT cluster boundary can be split in the
        # image, so a couple of misses is normal; a stale build misses many.
        pct = 100.0 * found / len(probes)
        verdict = "STALE BUILD" if pct < 97 else "probably current"
        print("  %-26s %s (%d/%d lines, %.1f%%)" % (name, verdict, found, len(probes), pct))
        print("  lines from the working copy not found in the image:")
        for p in missing[:8]:
            print("      %s" % p.decode(errors="replace")[:68])
        if len(missing) > 8:
            print("      ... and %d more" % (len(missing) - 8))
        if pct < 97:
            warnings.append(
                "the image was built from a different revision of %s "
                "(%d of %d lines missing)" % (name, len(missing), len(probes)))

    print("\n" + ("-" * 60))
    if warnings:
        print("review before publishing publicly:")
        for w in warnings:
            print("  * %s" % w)
        print(
            "\nDeleting a file or clearing a setting does not wipe the flash blocks\n"
            "behind it, so a dump can still carry old data. For a clean public\n"
            "image: esptool erase-flash, re-burn UIFlow 2 skipping the Wi-Fi setup,\n"
            "re-run tools/provision_for_export.py, then export."
        )
        return 1
    print("nothing flagged - image looks safe to publish")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Put a (modified) Model:Cycles OS 1.13 MAIN OS into the official Model:Samples OS 1.13 container.

Rule (same as Modded-Cycles crossflash.py and ms-multi-output --target cycles-crossflash): keep the
container of the machine that RECEIVES the file. Everything the Samples bootstrap and running OS check
stays Samples: product 0x0F, start-marker device byte 0x0A (also the packet checksum seed), ELE3 header
byte 0x07 = 0x14, bootstrap (section 2), updater (4), meta (5), HMAC-SHA256 key ("DELAY TIME").
Only section 3 (MAIN OS) is taken from the guest file, as its stored aPLib stream, not recompressed.

    python3 tools/pack_for_samples.py --samples stock/model-samples_OS1.13.syx \
        --cycles stock/model-cycles_OS1.13.syx --guest out/Model-TG-v1.0.0.syx \
        --expect-main fb985a1660f946cc0fd0133300c7e6248b7b23a7f3ffede339fc2e368563fae6 \
        -o out/Model-TG-v1.0.0_for-model-samples.syx

The guest must be a valid Model:Cycles image whose sections 2/4/5 are the official Cycles ones (proves
only the MAIN OS was changed). The output is re-read and every check is replayed before it is written.
"""
import argparse
import hashlib
import hmac
import os
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
# mtlib (MIT, drumkilla/elektron-model-tweaks) from a Modded-Cycles checkout: MODDED_CYCLES=path/to/Modded-Cycles
sys.path.insert(0, str(pathlib.Path(os.environ.get("MODDED_CYCLES", HERE.parent.parent / "Modded-Cycles")) / "tools"))

from mtlib import aplib, container                     # noqa: E402
from mtlib.syx import unwrap, wrap, BYTES_PER_MSG       # noqa: E402

OFFICIAL = {
    "cycles": dict(product=0x11, dev=0x0C, hdr7=0x15,
                   syx="44fe586269631a0ca7da25a3383fc6733c314809505fc3cc52f1e0ed9800640c",
                   main="cc99d4f0175d34d1e91d046e6ec85a5e8ab58ab9edbb3c24406acd48cb99ee98"),
    "samples": dict(product=0x0F, dev=0x0A, hdr7=0x14,
                    syx="e11859b68deb7e5e3fe86ab32581212093849c4be5d3950add011eac398a2ce8",
                    main="a351392c62ec1c6c3324a807baf46934690d54edfc76029a4b4882541cad1ab2"),
}
# The bootstrap decompresses the MAIN OS to 0x40000400 and keeps its work area from 0x40200000
# (Modded-Cycles notes/31). Model-TG's own build caps its blob at 0x401c3750.
MAIN_BASE, MAIN_LIMIT = 0x40000400, 0x40200000


def sha(b):
    return hashlib.sha256(b).hexdigest()


def die(msg):
    raise SystemExit("!! " + msg)


def load(path, product, official_sha=None):
    raw = pathlib.Path(path).read_bytes()
    if official_sha and sha(raw) != official_sha:
        die(f"{path}: not the official OS 1.13 file (SHA-256 {sha(raw)[:16]}...)")
    stream, info = unwrap(raw)                         # validates every packet checksum and the count
    if info["product"] != product:
        die(f"{path}: product 0x{info['product']:02x}, expected 0x{product:02x}")
    c = container.parse(stream)
    blob = c["blob"]
    if int.from_bytes(stream[4:8], "big") != container.content_checksum(blob):
        die(f"{path}: content checksum mismatch")
    stored, plain = {}, {}
    for s in c["sections"]:
        stored[s["id"]] = blob[s["off"]:s["off"] + s["size"]]
        try:
            plain[s["id"]] = aplib.depack(stored[s["id"]])[0]
        except Exception:
            plain[s["id"]] = stored[s["id"]]
    body, dig = blob[:-container.DIGEST_LEN], blob[-container.DIGEST_LEN:]
    key = container.find_key(list(plain.values()), body, dig)
    if not key:
        die(f"{path}: HMAC trailer does not verify with the key derived from the image")
    return dict(raw=raw, info=info, c=c, stored=stored, plain=plain, key=key, head=raw[:raw.find(0xF7) + 1])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--samples", required=True, help="official model-samples_OS1.13.syx (host container)")
    ap.add_argument("--cycles", required=True, help="official model-cycles_OS1.13.syx (to check the guest)")
    ap.add_argument("--guest", required=True, help="Model:Cycles image whose MAIN OS goes in (e.g. Model-TG.syx)")
    ap.add_argument("--expect-main", help="required SHA-256 of the guest's decompressed MAIN OS")
    ap.add_argument("-o", "--output", required=True)
    a = ap.parse_args()

    smp = load(a.samples, 0x0F, OFFICIAL["samples"]["syx"])
    cyc = load(a.cycles, 0x11, OFFICIAL["cycles"]["syx"])
    gst = load(a.guest, 0x11)
    if sha(smp["plain"][3]) != OFFICIAL["samples"]["main"] or sha(cyc["plain"][3]) != OFFICIAL["cycles"]["main"]:
        die("official MAIN OS hashes differ")
    if gst["key"] != cyc["key"]:
        die("guest is not signed with the Model:Cycles key")
    for sid in (2, 4, 5):
        if gst["stored"].get(sid) != cyc["stored"][sid]:
            die(f"guest section {sid} differs from the official Cycles one: only the MAIN OS may change")
    if set(gst["stored"]) != set(cyc["stored"]):
        die(f"guest sections {sorted(gst['stored'])} != official {sorted(cyc['stored'])}")
    g_main = gst["plain"][3]
    if a.expect_main and sha(g_main) != a.expect_main:
        die(f"guest MAIN OS {sha(g_main)} != expected {a.expect_main}")
    if MAIN_BASE + len(g_main) > MAIN_LIMIT:
        die(f"guest MAIN OS ends at 0x{MAIN_BASE + len(g_main):08x}, past the bootstrap work area 0x{MAIN_LIMIT:08x}")

    # build: Samples container, guest's stored section 3, Samples key, Samples transport
    blob = container.rebuild(smp["c"], {3: gst["stored"][3]}, smp["key"])
    out = wrap(container.build_stream(blob, BYTES_PER_MSG), 0x0F, smp["info"]["start_seq"])

    # re-read and replay every check the Samples OS (gate 1) and Samples bootstrap (gate 2) make
    o = load_bytes(out)
    head = out[:out.find(0xF7) + 1]
    if head[4] != 0x0F or head[6:8] != b"\x7f\x01" or head[8] != 0x0A:
        die(f"start marker {head.hex()}: need product 0x0F and device byte 0x0A")
    if head[:9] != smp["head"][:9]:
        die("start marker prefix differs from the official Samples file")
    ob = o["c"]["blob"]
    if ob[7] != OFFICIAL["samples"]["hdr7"] or ob[:0x1C] != smp["c"]["blob"][:0x1C]:
        die("ELE3 header differs from the official Samples one")
    for sid in (2, 4, 5):
        if o["stored"][sid] != smp["stored"][sid]:
            die(f"section {sid} is not the official Samples one")
    if o["stored"][3] != gst["stored"][3] or o["plain"][3] != g_main:
        die("section 3 is not the guest MAIN OS")
    body, dig = ob[:-container.DIGEST_LEN], ob[-container.DIGEST_LEN:]
    if not hmac.compare_digest(hmac.new(smp["key"], body, hashlib.sha256).digest(), dig):
        die("HMAC does not verify with the Samples key")
    if hmac.compare_digest(hmac.new(cyc["key"], body, hashlib.sha256).digest(), dig):
        die("HMAC also verifies with the Cycles key: inconsistent")
    if o["key"] != smp["key"]:
        die("key re-derived from the output is not the Samples key")
    # the Samples OS also derives its key from its own MAIN OS ("DELAY TIME" at 0x4012A35F): same key
    s3 = smp["plain"][3]
    t0 = 0x4012A35F - 0x40000400
    text = s3[t0:s3.index(b"\0", t0)]
    const = s3[0x4012A37E - 0x40000400:0x4012A37E - 0x40000400 + 32]
    k2 = bytes(x ^ y ^ z for x, y, z in zip(hashlib.sha256(text).digest(), hashlib.sha256(text[::-1]).digest(), const))
    if text != b"DELAY TIME" or k2 != smp["key"]:
        die("Samples MAIN OS key derivation does not match the bootstrap key")

    pathlib.Path(a.output).write_bytes(out)
    print(f"wrote {a.output}  ({len(out)} B, {o['info']['count']} packets)")
    print(f"  sha256 .syx      {sha(out)}")
    print(f"  sha256 MAIN OS   {sha(g_main)}  ({len(g_main)} B, ends 0x{MAIN_BASE + len(g_main):08x})")
    print("  transport 0x0F / dev 0x0A, ELE3 hdr 0x14, sections 2/4/5 = official Samples,")
    print("  section 3 = guest stream verbatim, HMAC valid with Samples key (\"DELAY TIME\"), invalid with Cycles key")


def load_bytes(raw):
    p = pathlib.Path(sys.argv[0]).with_name(".pack_for_samples.tmp")
    try:
        p.write_bytes(raw)
        return load(p, 0x0F)
    finally:
        p.unlink(missing_ok=True)


if __name__ == "__main__":
    main()

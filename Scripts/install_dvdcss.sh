#!/bin/bash
# install_dvdcss.sh — one-step setup for encrypted-DVD playback on the DVD core.
#
# It fetches a prebuilt libdvdcss (the DVD-decryption library) and drops it where
# the core looks for it. libdvdcss is NOT part of MiSTer and is NOT included here —
# this script only downloads it for you from a third party, so you decide to
# install it. Run it once from the MiSTer Scripts menu; then encrypted discs play.
#
# It needs only tools a stock MiSTer already has: wget and python3. (The library
# ships as a .deb compressed with xz; python3's standard library unpacks it, so no
# extra tools are required.)
#
# Advanced: set DVDCSS_URL to a direct link to a glibc/armhf "libdvdcss.so.2" to
# use your own copy instead of the default source, e.g.
#   DVDCSS_URL=http://host/libdvdcss.so.2 ./install_dvdcss.sh

set -u

DEST_DIR="${DVDCSS_DEST_DIR:-/media/fat/dvdcss}"
DEST="$DEST_DIR/libdvdcss.so.2"
TMP="/tmp/dvdcss_install.$$"

# Third-party source: prebuilt armhf libdvdcss2 from deb-multimedia. MiSTer runs
# an older glibc (~2.32), so we prefer OLDER-suite builds (bullseye needs only
# GLIBC_2.7); newer builds (bookworm) require GLIBC_2.33+ and would not load. The
# on-device load check below skips any candidate that is nonetheless too new.
DEB_URLS="
http://www.deb-multimedia.org/pool/main/libd/libdvdcss-dmo/libdvdcss2_1.4.3-dmo1_armhf.deb
http://www.deb-multimedia.org/pool/main/libd/libdvdcss-dmo/libdvdcss2_1.4.3-dmo2_armhf.deb
http://www.deb-multimedia.org/pool/main/libd/libdvdcss-dmo/libdvdcss2_1.4.3-dmo2+b1_armhf.deb
"

say()  { echo "dvdcss: $*"; }
fail() { echo "dvdcss: ERROR: $*" >&2; }

fetch() { # url dest
	if command -v wget >/dev/null 2>&1; then wget -q -O "$2" "$1";
	elif command -v curl >/dev/null 2>&1; then curl -fsSL -o "$2" "$1";
	else fail "no wget or curl available"; return 1; fi
}

# Confirm the result is a 32-bit ARM ELF shared object (guards against an HTML
# error page or a wrong-arch/musl library that would fail to load).
verify_so() { # path
	local hdr
	hdr=$(head -c 20 "$1" 2>/dev/null | od -An -tx1 | tr -d ' \n')
	case "$hdr" in
		7f454c4601*2800) return 0 ;;   # ELF32 little-endian, e_machine=0x28 (ARM)
		7f454c46*) fail "downloaded library is an ELF but not 32-bit ARM"; return 1 ;;
		*) fail "downloaded file is not a library (bad download?)"; return 1 ;;
	esac
}

# Actually dlopen the library on THIS system, but only REJECT it when the failure
# is specifically a glibc-version mismatch (a too-new build). Any other outcome —
# it loads, ctypes is missing, some unrelated error — is treated as "ok/can't
# tell" so a genuinely fine library is never blocked.
verify_loadable() { # path -> nonzero only on a confirmed glibc mismatch
	command -v python3 >/dev/null 2>&1 || return 0
	python3 - "$1" <<'PY'
import sys
try:
    import ctypes, os
except Exception:
    sys.exit(0)
try:
    ctypes.CDLL(sys.argv[1], mode=os.RTLD_NOW)
    sys.exit(0)
except OSError as e:
    m = str(e)
    sys.exit(1 if ("GLIBC_" in m or "version `" in m) else 0)
except Exception:
    sys.exit(0)
PY
}

# Extract usr/lib/.../libdvdcss.so.2* out of a Debian .deb using only python3's
# standard library (ar container by hand; xz via the lzma module, or an xz binary
# if this python lacks lzma; tar via tarfile).
extract_so() { # deb dest
	python3 - "$1" "$2" <<'PY'
import io, sys, tarfile, subprocess
deb_path, dest = sys.argv[1], sys.argv[2]
data = open(deb_path, "rb").read()
if data[:8] != b"!<arch>\n":
    sys.exit("not a .deb (ar) archive")
off, members = 8, {}
while off + 60 <= len(data):
    hdr = data[off:off+60]; off += 60
    name = hdr[0:16].decode("ascii","replace").strip().rstrip("/")
    size = int(hdr[48:58].decode("ascii").strip())
    members[name] = data[off:off+size]
    off += size + (size & 1)
dn = next((n for n in members if n.startswith("data.tar")), None)
if not dn:
    sys.exit("no data.tar in package")
raw = members[dn]
if dn.endswith(".xz"):
    try:
        import lzma; tar_bytes = lzma.decompress(raw)
    except Exception:
        for xz in ("xz","unxz","busybox"):
            try:
                args = [xz,"unxz"] if xz=="busybox" else [xz,"-dc"]
                tar_bytes = subprocess.run(args, input=raw, stdout=subprocess.PIPE, check=True).stdout
                break
            except Exception:
                tar_bytes = None
        if tar_bytes is None:
            sys.exit("cannot decompress xz (python lzma module missing and no xz binary)")
elif dn.endswith(".gz"):
    import gzip; tar_bytes = gzip.decompress(raw)
elif dn.endswith(".zst"):
    tar_bytes = subprocess.run(["zstd","-dc"], input=raw, stdout=subprocess.PIPE, check=True).stdout
else:
    tar_bytes = raw
tf = tarfile.open(fileobj=io.BytesIO(tar_bytes), mode="r:")
so = next((m for m in tf.getmembers() if m.isreg() and "libdvdcss.so.2" in m.name), None)
if not so:
    sys.exit("libdvdcss.so.2 not found in package")
open(dest, "wb").write(tf.extractfile(so).read())
PY
}

mkdir -p "$DEST_DIR" "$TMP" || { fail "cannot create $DEST_DIR"; exit 1; }

# 1) A direct .so link (user-provided) is the simplest, most reliable path.
if [ "${DVDCSS_URL:-}" != "" ] && printf '%s' "$DVDCSS_URL" | grep -qiE '\.so(\.[0-9]+)*$'; then
	say "downloading library from DVDCSS_URL..."
	if fetch "$DVDCSS_URL" "$DEST" && verify_so "$DEST" && verify_loadable "$DEST"; then
		chmod 0644 "$DEST"; rm -rf "$TMP"
		say "installed $DEST"; say "done — insert a disc and open the DVD core."; exit 0
	fi
	rm -f "$DEST"
	fail "could not install from DVDCSS_URL (download failed, or the library needs a newer glibc than this system)"
	rm -rf "$TMP"; exit 1
fi

if ! command -v python3 >/dev/null 2>&1; then
	fail "python3 not found (needed to unpack the library)."
	rm -rf "$TMP"
	echo "Manual step: put a glibc/armhf libdvdcss.so.2 at $DEST"
	exit 1
fi

# 2) Download the .deb (first candidate that works) and extract with python3.
say "downloading DVD decryption library..."
urls="${DVDCSS_URL:+$DVDCSS_URL} $DEB_URLS"
for url in $urls; do
	[ -n "$url" ] || continue
	if fetch "$url" "$TMP/pkg.deb" && [ -s "$TMP/pkg.deb" ]; then
		if extract_so "$TMP/pkg.deb" "$DEST" && verify_so "$DEST"; then
			if verify_loadable "$DEST"; then
				chmod 0644 "$DEST"; rm -rf "$TMP"
				say "installed $DEST"
				say "done — insert a disc and open the DVD core."
				exit 0
			fi
			say "that build needs a newer glibc than this system — trying an older one..."
			rm -f "$DEST"
		fi
	fi
done

rm -rf "$TMP"
fail "automatic install failed (could not download or unpack the library)."
echo
echo "Manual step (one file):"
echo "  1. On a PC, get a glibc/armhf 'libdvdcss.so.2' (e.g. from the deb-multimedia"
echo "     libdvdcss2 armhf package)."
echo "  2. Copy it to the SD card as:  $DEST"
echo "  Or re-run with a direct link:  DVDCSS_URL=<url-to-libdvdcss.so.2> $0"
exit 1

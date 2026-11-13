#!/usr/bin/env bash
# Wait for the DSO-510 IAP drive (hold update button + power on), then copy a firmware .bin.
set -euo pipefail
BIN=${1:?usage: tools/flash.sh path/to/APP_DSO-510_x.bin}
[ -f "$BIN" ] || { echo "no such file: $BIN" >&2; exit 1; }
echo "Waiting for the IAP drive (hold update button and press power)..."
until DEV=$(lsblk -dnpo NAME,MODEL | awk '$2=="Msc" && $3=="iap" {print $1}' | head -1) && [ -n "$DEV" ]; do sleep 1; done
sleep 2
MNT=$(lsblk -no MOUNTPOINTS "$DEV" | head -1)
if [ -z "$MNT" ]; then udisksctl mount -b "$DEV" >/dev/null; MNT=$(lsblk -no MOUNTPOINTS "$DEV" | head -1); fi
echo "IAP drive at $MNT ($(ls "$MNT" | tr '\n' ' '))"
cp "$BIN" "$MNT/" && sync
echo "Copied $(basename "$BIN") ($(stat -c %s "$BIN") bytes). The scope flashes and starts the app."

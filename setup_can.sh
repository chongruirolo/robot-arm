#!/usr/bin/env bash
# Run once before any robot scripts: brings up can0 at 1 Mbps (Piper default).
# Re-run if the USB-CAN adapter is unplugged/replugged.

set -e

IFACE="can0"
BITRATE=1000000

if ! ip link show "$IFACE" &>/dev/null; then
    echo "ERROR: $IFACE not found. Is the USB-CAN adapter plugged in?"
    exit 1
fi

STATE=$(cat /sys/class/net/$IFACE/operstate 2>/dev/null || echo "unknown")

if [ "$STATE" = "up" ]; then
    echo "$IFACE is already up."
else
    echo "Bringing up $IFACE at $BITRATE bps..."
    sudo ip link set "$IFACE" down 2>/dev/null || true
    sudo ip link set "$IFACE" up type can bitrate "$BITRATE"
    echo "$IFACE is up."
fi

ip link show "$IFACE"

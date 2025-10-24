#!/bin/bash

DRIVER_DIR="../kernel"
DRIVER_NAME="nxp_simtemp.ko"
DRIVER_PATH="${DRIVER_DIR}/${DRIVER_NAME}"
OVERLAY_DIR="${DRIVER_DIR}/dts"
OVERLAY_NAME="nxp_simtemp-overlay.dtbo"
OVERLAY_PATH="${OVERLAY_DIR}/${OVERLAY_NAME}"

echo "Warning: This script must be run with sudo"

if [ -f "$DRIVER_PATH" ]; then
	echo "Installing driver ${DRIVER_NAME}"
	insmod "$DRIVER_PATH"
fi

if [ -f "$OVERLAY_PATH" ]; then
	echo "Loading overlay..."
	dtoverlay "$OVERLAY_PATH"
fi

echo "Printing last kernel logs:"
dmesg | tail

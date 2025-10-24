#!/bin/bash

echo "Building the driver..."
pushd ../kernel > /dev/null
make
popd > /dev/null

echo "Building the device tree..."
pushd ../kernel/dts > /dev/null
dtc -@ -I dts -O dtb -o nxp_simtemp-overlay.dtbo nxp_simtemp-overlay.dts
popd > /dev/null

echo "Building the test application..."
pushd ../user/cli > /dev/null
make
popd > /dev/null

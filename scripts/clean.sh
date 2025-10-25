#!/bin/bash

echo "Cleaning project..."
pushd ../kernel > /dev/null
make clean
popd > /dev/null

pushd ../user/cli > /dev/null
make clean
popd > /dev/null

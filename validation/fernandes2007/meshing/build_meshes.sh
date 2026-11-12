#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Chinmay S. Patil
set -euo pipefail
cd "${0%/*}"

if [ -z "${WM_PROJECT_DIR:-}" ]; then
    . /usr/lib/openfoam/openfoam2412/etc/bashrc
fi

echo "=== 1. Building rotatingMesh ==="
cd rotatingMesh
mkdir -p constant/triSurface
cp ../triSurface/* constant/triSurface/
surfaceFeatureExtract > log.surfaceFeatureExtract 2>&1
blockMesh > log.blockMesh 2>&1
snappyHexMesh -overwrite > log.snappyHexMesh 2>&1
checkMesh > log.checkMesh 2>&1
echo "rotatingMesh completed. Result:"
tail -n 4 log.checkMesh

cd ../staticMesh
echo "=== 2. Building staticMesh ==="
mkdir -p constant/triSurface
cp ../triSurface/* constant/triSurface/
surfaceFeatureExtract > log.surfaceFeatureExtract 2>&1
blockMesh > log.blockMesh 2>&1
snappyHexMesh -overwrite > log.snappyHexMesh 2>&1
checkMesh > log.checkMesh 2>&1
echo "staticMesh completed. Result:"
tail -n 4 log.checkMesh

echo "=== Mesh generation complete ==="

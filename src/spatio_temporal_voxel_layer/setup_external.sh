#!/bin/bash
# spatio_temporal_voxel_layer 외부 라이브러리 준비 스크립트

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
EXTERNAL_DIR="$SCRIPT_DIR/external"
TAR_FILE="$EXTERNAL_DIR/openvdb_vendor.tar.gz"
TARGET_DIR="$EXTERNAL_DIR/openvdb_vendor"

echo "Checking for compressed external binaries..."

if [ ! -f "$TAR_FILE" ]; then
    echo "Error: $TAR_FILE not found!"
    exit 1
fi

if [ -d "$TARGET_DIR" ]; then
    echo "External directory already exists. Skipping extraction."
    exit 0
fi

echo "Extracting openvdb_vendor.tar.gz..."
tar -xzf "$TAR_FILE" -C "$EXTERNAL_DIR"

if [ $? -eq 0 ]; then
    echo "Extraction successful!"
else
    echo "Error: Extraction failed!"
    exit 1
fi

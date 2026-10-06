#!/bin/bash
# HumanCallRobot Build Script for Linux/macOS

set -e

echo "=== Building HumanCallRobot ==="
mkdir -p build
cmake -B build
cmake --build build

echo "=== Build Complete ==="

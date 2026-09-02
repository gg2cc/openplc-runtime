#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# SRC="${SCRIPT_DIR}/drivers/plugins/native/canopen"
# TARGET="root@192.168.1.173:/opt/openplc-runtime/core/src/drivers/plugins/native/"

SRC="${SCRIPT_DIR}/drivers/plugins/native/canopen/libcanopen_plugin.so"
TARGET="root@192.168.1.173:/opt/openplc-runtime/build/plugins"

# SRC="${SCRIPT_DIR}/drivers/plugins/native/can/libcan_plugin.so"
# TARGET="root@192.168.1.173:/opt/openplc-runtime/build/plugins"

echo "Deploying ${SRC} to ${TARGET}..."
scp "${SRC}" "${TARGET}"

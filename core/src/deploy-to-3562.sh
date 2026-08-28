#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="${SCRIPT_DIR}/drivers/plugins/native/canopen"
TARGET="root@192.168.1.173:/opt/openplc-runtime/core/src/drivers/plugins/native/"

echo "Deploying ${SRC} to ${TARGET}..."
scp -r "${SRC}" "${TARGET}"

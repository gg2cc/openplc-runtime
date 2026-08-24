#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="${SCRIPT_DIR}/drivers/plugins/native/can"
TARGET="root@192.168.100.100:/opt/openplc-runtime/core/src/drivers/plugins/native/"

echo "Deploying ${SRC} to ${TARGET}..."
scp -r "${SRC}" "${TARGET}"

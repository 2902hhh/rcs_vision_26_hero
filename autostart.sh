#!/usr/bin/env bash

set -euo pipefail

# Resolve project root from this script location.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

# Wait for desktop/session and USB devices to settle on boot.
sleep 5

mkdir -p logs

screen \
    -L \
    -Logfile "logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog" \
    -d \
    -m \
    bash -lc "./build/standard configs/hero.yaml"

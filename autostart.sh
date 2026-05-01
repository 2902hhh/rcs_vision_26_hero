#!/usr/bin/env bash

# Wait for hardware, drivers, and desktop session to become ready.
sleep 10

# Absolute paths.
USER_NAME="rm"
PROJECT_DIR="/home/$USER_NAME/Desktop/rcs_vision_26_hero_classic"
SCREEN_BIN="/usr/bin/screen"
SESSION_NAME="vision"
RESTART_DELAY_SEC=10

# Enter project directory.
cd "$PROJECT_DIR" || { echo "Directory not found: $PROJECT_DIR"; exit 1; }

# Prepare logs.
mkdir -p logs

# Clean up previous instances before creating a new screen session.
killall -9 standard 2>/dev/null || true
"$SCREEN_BIN" -S "$SESSION_NAME" -X quit 2>/dev/null || true

# Start standard in a detached screen session with auto-restart.
"$SCREEN_BIN" \
  -L \
  -Logfile "$PROJECT_DIR/logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog" \
  -S "$SESSION_NAME" \
  -d \
  -m \
  bash -lc "while true; do ./build/standard configs/hero.yaml; echo 'standard exited, restart after ${RESTART_DELAY_SEC}s'; sleep ${RESTART_DELAY_SEC}; done"

# Give systemd a deterministic result.
echo "Vision program started in screen session '$SESSION_NAME' with auto-restart enabled"

#!/usr/bin/env bash

sleep 5

PROJECT_DIR="$HOME/Desktop/rcs_vision_26_hero_classic"
cd "$PROJECT_DIR" || exit 1

mkdir -p logs

screen \
    -L \
    -Logfile "logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog" \
    -d \
    -m \
    bash -lc "./build/standard configs/hero.yaml"

#!/usr/bin/env bash
# run_brain.sh — Launches the full autonomous hexapod brain stack
# Usage: ./run_brain.sh [extra ros2 launch args]
# Example: ./run_brain.sh model:=nvidia/llama-3.1-nemotron-70b-instruct

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SECRETS_FILE="$SCRIPT_DIR/.brain_secrets"

# ── Load secrets ──────────────────────────────────────────────────────────────
if [[ -f "$SECRETS_FILE" ]]; then
    # Source only KEY=VALUE lines, skip comments
    export $(grep -v '^\s*#' "$SECRETS_FILE" | xargs)
    echo "[brain] Loaded API key from .brain_secrets"
else
    echo "[brain] WARNING: .brain_secrets not found — NVIDIA_API_KEY must be set manually"
fi

if [[ -z "$NVIDIA_API_KEY" ]]; then
    echo "[brain] ERROR: NVIDIA_API_KEY is not set!"
    echo "  Create /home/vishal/gz_hex/.brain_secrets with:"
    echo "  NVIDIA_API_KEY=nvapi-xxxx..."
    exit 1
fi

# ── ROS2 environment ──────────────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# openai package location
export PYTHONPATH=/home/vishal/.local/lib/python3.12/site-packages:${PYTHONPATH:-}

echo "[brain] Launching autonomous brain stack..."
echo "[brain] Model: ${MODEL:-meta/llama-3.1-70b-instruct}"
echo "[brain] Send queries to: ros2 topic pub --once /brain/query std_msgs/msg/String \"{data: 'your task here'}\""
echo ""

ros2 launch rover autonomous.launch.py "$@"

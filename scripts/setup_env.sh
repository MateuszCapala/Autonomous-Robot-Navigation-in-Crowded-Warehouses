#!/bin/bash
# Source this file before building: source scripts/setup_env.sh

source /opt/ros/jazzy/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

if [ -f "$WS_ROOT/install/setup.bash" ]; then
    source "$WS_ROOT/install/setup.bash"
fi

export ACADOS_ROOT="${ACADOS_ROOT:-$HOME/acados}"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$ACADOS_ROOT/lib"
export PYTHONPATH="$PYTHONPATH:$ACADOS_ROOT/interfaces/acados_template"

echo "ROS2 Jazzy + ACADOS environment loaded."
echo "  ACADOS_ROOT: $ACADOS_ROOT"

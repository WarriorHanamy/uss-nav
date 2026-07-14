#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RVIZ_WS_DIR="${RVIZ_WS_DIR:-$HOME/rviz_ws}"
ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"

# ── check prerequisites ────────────────────────────────────────────
if [[ -z "${DISPLAY:-}" ]]; then
    echo "DISPLAY is not set; cannot launch RViz." >&2
    exit 1
fi

if ! command -v docker >/dev/null 2>&1; then
    echo "docker not found." >&2
    exit 1
fi

# ── X11 authorization ──────────────────────────────────────────────
command -v xhost >/dev/null 2>&1 && xhost +SI:localuser:root >/dev/null

# ── cleanup trap ────────────────────────────────────────────────────
cleanup() {
    echo ""
    echo "Stopping uss-nav devel container..."
    docker compose --project-directory "${SCRIPT_DIR}" stop devel 2>/dev/null || true
    echo "Done."
}
trap cleanup EXIT

# ── start simulation container ──────────────────────────────────────
echo "Starting uss-nav simulation (devel)..."
docker compose --project-directory "${SCRIPT_DIR}" up -d devel

# ── wait for ROS master ─────────────────────────────────────────────
echo "Waiting for roscore on localhost:11311..."
for i in $(seq 1 60); do
    if timeout 1 bash -c "echo >/dev/tcp/127.0.0.1/11311" 2>/dev/null; then
        echo "roscore ready (${i}s)"
        break
    fi
    if [[ $i -eq 60 ]]; then
        echo "Timeout waiting for roscore." >&2
        exit 1
    fi
    sleep 1
done

# ── build rviz image if needed ──────────────────────────────────────
if ! docker image inspect rviz_ws:latest >/dev/null 2>&1; then
    echo "Building rviz_ws image..."
    docker compose --project-directory "${RVIZ_WS_DIR}" build rviz
fi

# ── launch RViz (blocking, Ctrl+C to quit) ─────────────────────────
echo ""
echo "=== USS-NAV Simulation + RViz ==="
echo "  devel:  localhost:11311"
echo "  rviz:   ${RVIZ_WS_DIR}/bringup/config/ego_planner/uss_nav_sim.rviz"
echo "  Ctrl+C to stop all"
echo ""

docker compose --project-directory "${RVIZ_WS_DIR}" run --rm \
    -e ROS_MASTER_URI="${ROS_MASTER_URI}" \
    -e DISPLAY="${DISPLAY}" \
    rviz \
    bash -lc 'rviz -d /root/rviz_ws/bringup/config/ego_planner/uss_nav_sim.rviz'

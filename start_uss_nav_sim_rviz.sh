#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RVIZ_WS_DIR="${RVIZ_WS_DIR:-$HOME/rviz_ws}"
ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
DEVEL_CONTAINER="uss-nav-devel-${GIT_SHA:-local}"
DEFAULT_MAP_PCD="${SCRIPT_DIR}/.data/pcd/J30V2_latest.pcd"
FALLBACK_MAP_PCD="${SCRIPT_DIR}/2026-06-28-all-120.pcd"

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

# ── scenegraph data preflight ─────────────────────────────────────
if [[ "${LAUNCH_MODE:-scenegraph}" != "random" && ! -f "${DEFAULT_MAP_PCD}" ]]; then
    if [[ -f "${FALLBACK_MAP_PCD}" ]]; then
        if ! mkdir -p "$(dirname "${DEFAULT_MAP_PCD}")" || ! ln -f "${FALLBACK_MAP_PCD}" "${DEFAULT_MAP_PCD}"; then
            echo "Failed to link default scene map into .data/pcd." >&2
            echo "Check ownership of ${SCRIPT_DIR}/.data or create ${DEFAULT_MAP_PCD} manually." >&2
            exit 1
        fi
        echo "Linked default scene map: ${DEFAULT_MAP_PCD}"
    else
        echo "Missing scene map: ${DEFAULT_MAP_PCD}" >&2
        echo "Provide it or run with LAUNCH_MODE=random for the procedural map." >&2
        exit 1
    fi
fi

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
docker compose --project-directory "${SCRIPT_DIR}" up --build --force-recreate -d devel

# ── wait for ROS master ─────────────────────────────────────────────
echo "Waiting for roscore on localhost:11311..."
for i in $(seq 1 60); do
    if [[ "$(docker inspect -f '{{.State.Running}}' "${DEVEL_CONTAINER}" 2>/dev/null || true)" != "true" ]]; then
        echo "devel container exited before roscore became ready. Recent logs:" >&2
        docker logs --tail 120 "${DEVEL_CONTAINER}" >&2 || true
        exit 1
    fi
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

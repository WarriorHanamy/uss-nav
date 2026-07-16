#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RVIZ_WS_DIR="${RVIZ_WS_DIR:-$HOME/rviz_ws}"
ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
COMPOSE_PROJECT_NAME="${COMPOSE_PROJECT_NAME:-uss-nav-super}"
DEVEL_CONTAINER="uss-nav-devel-${GIT_SHA:-local}"
FLUENT_BIT_CONTAINER="uss-nav-fluent-bit-${GIT_SHA:-local}"
DEFAULT_MAP_PCD="${SCRIPT_DIR}/.data/pcd/J30V2_latest.pcd"
FALLBACK_MAP_PCD="${SCRIPT_DIR}/2026-06-28-all-120.pcd"
DEFAULT_SCENE_GRAPH="${SCRIPT_DIR}/.data/scene_graph/J30V2_snapshot"
FALLBACK_SCENE_GRAPH="${SCRIPT_DIR}/scene_graph_saved/J30V2_whole-20260626-9"
RVIZ_DEVEL_SETUP="${RVIZ_WS_DIR}/.artifacts/devel/setup.bash"
export LAUNCH_MODE="${LAUNCH_MODE:-super}"
export TRACE_ENABLE="${TRACE_ENABLE:-1}"
export TRACE_ID="${TRACE_ID:-super-$(date +%Y%m%d-%H%M%S)}"

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

if [[ "${LAUNCH_MODE:-scenegraph}" != "random" && ! -f "${DEFAULT_SCENE_GRAPH}/manifest.json" ]]; then
    if [[ -f "${FALLBACK_SCENE_GRAPH}/manifest.json" ]]; then
        echo "Missing scene graph snapshot: ${DEFAULT_SCENE_GRAPH}" >&2
        echo "Create it from ${FALLBACK_SCENE_GRAPH} before launching." >&2
        echo "Example: mkdir -p ${SCRIPT_DIR}/.data/scene_graph && cp -a ${FALLBACK_SCENE_GRAPH} ${DEFAULT_SCENE_GRAPH}" >&2
        exit 1
    else
        echo "Missing scene graph snapshot: ${DEFAULT_SCENE_GRAPH}" >&2
        echo "Provide it or run with LAUNCH_MODE=random for the procedural map." >&2
        exit 1
    fi
fi

# ── cleanup trap ────────────────────────────────────────────────────
cleanup() {
    echo ""
    echo "Stopping uss-nav devel container..."
    docker compose --project-name "${COMPOSE_PROJECT_NAME}" --project-directory "${SCRIPT_DIR}" down --remove-orphans 2>/dev/null || true
    docker rm -f "${DEVEL_CONTAINER}" >/dev/null 2>&1 || true
    docker rm -f "${FLUENT_BIT_CONTAINER}" >/dev/null 2>&1 || true
    echo "Done."
}
trap cleanup EXIT

# ── start simulation container ──────────────────────────────────────
echo "Starting uss-nav simulation (devel)..."
docker compose --project-name "${COMPOSE_PROJECT_NAME}" --project-directory "${SCRIPT_DIR}" down --remove-orphans >/dev/null 2>&1 || true
docker rm -f "${DEVEL_CONTAINER}" >/dev/null 2>&1 || true
docker rm -f "${FLUENT_BIT_CONTAINER}" >/dev/null 2>&1 || true
docker compose --project-name "${COMPOSE_PROJECT_NAME}" --project-directory "${SCRIPT_DIR}" up --build --force-recreate -d devel
if [[ "${TRACE_ENABLE}" == "1" || "${TRACE_ENABLE}" == "true" || "${TRACE_ENABLE}" == "TRUE" ]]; then
    if ! docker compose --project-name "${COMPOSE_PROJECT_NAME}" --project-directory "${SCRIPT_DIR}" up --force-recreate -d fluent-bit; then
        echo "Warning: Fluent Bit sidecar did not start; ROS logs and run.bag will still be written." >&2
    fi
fi

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

if [[ ! -f "${RVIZ_DEVEL_SETUP}" ]]; then
    echo "Building rviz_ws workspace artifacts..."
    mkdir -p "${RVIZ_WS_DIR}/.artifacts/build" "${RVIZ_WS_DIR}/.artifacts/devel"
    docker compose --project-directory "${RVIZ_WS_DIR}" run --rm build
fi

# ── launch RViz (blocking, Ctrl+C to quit) ─────────────────────────
echo ""
echo "=== USS-NAV Simulation + RViz ==="
echo "  devel:  localhost:11311"
echo "  mode:   super planner
  rviz:   ${RVIZ_WS_DIR}/bringup/config/super_planner/scenegraph_super.rviz"
if [[ "${TRACE_ENABLE}" == "1" || "${TRACE_ENABLE}" == "true" || "${TRACE_ENABLE}" == "TRUE" ]]; then
    echo "  trace:  ${SCRIPT_DIR}/.artifacts/traces/${TRACE_ID}"
fi
echo "  Ctrl+C to stop all"
echo ""

docker compose --project-directory "${RVIZ_WS_DIR}" run --rm \
    -e ROS_MASTER_URI="${ROS_MASTER_URI}" \
    -e DISPLAY="${DISPLAY}" \
    rviz \
    bash -lc 'rviz -d /workspace/src/bringup/config/super_planner/scenegraph_super.rviz'

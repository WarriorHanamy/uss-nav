#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash
source /workspace/devel/setup.bash

echo "=== EGO Planner Simulation ==="

# ── display setup ──────────────────────────────────────────────────
if [ -z "$DISPLAY" ] || [ ! -d "/tmp/.X11-unix" ]; then
  echo "No DISPLAY, starting Xvfb..."
  export DISPLAY=:99
  Xvfb :99 -screen 0 1280x1024x24 &
  sleep 1
fi
echo "X11 display: ${DISPLAY}"

# ── launch ────────────────────────────────────────────────────────
echo "Starting simulation..."
echo "RViz managed by ~/rviz_ws (external), not started here."

LAUNCH_MODE="${LAUNCH_MODE:-scenegraph}"
MAP_PCD="${MAP_PCD:-/workspace/.data/pcd/J30V2_latest.pcd}"
TRACE_ENABLE="${TRACE_ENABLE:-0}"
TRACE_ID="${TRACE_ID:-uss-nav-$(date +%Y%m%d-%H%M%S)}"
TRACE_DIR="${TRACE_DIR:-/workspace/.artifacts/traces/${TRACE_ID}}"
DEFAULT_TRACE_BAG_TOPICS="/tf /tf_static /bridge/Instruct /Instruct_res /planner/fsm_state /tracking_finish /drone_0_visual_slam/odom /drone_0_ego_planner_node/local_goal /drone_0_planning/pos_cmd /planning/fsm_vis /planning/fsm_path /map_generator/global_cloud /drone_0_ego_planner_node/grid_map/occupancy_inflate /drone_0_ego_planner_node/grid_map/occupancy_inflateBig"
TRACE_BAG_TOPICS="${TRACE_BAG_TOPICS:-$DEFAULT_TRACE_BAG_TOPICS}"
ROSLAUNCH_LOG="/tmp/roslaunch.log"
ROSBAG_PID=""

if [ "$TRACE_ENABLE" = "1" ] || [ "$TRACE_ENABLE" = "true" ] || [ "$TRACE_ENABLE" = "TRUE" ]; then
  mkdir -p "$TRACE_DIR"
  export TRACE_ID TRACE_DIR DECISION_TRACE_FILE="${TRACE_DIR}/decision.jsonl"
  ROSLAUNCH_LOG="${TRACE_DIR}/roslaunch.log"
  cat > "${TRACE_DIR}/manifest.json" <<EOF
{"trace_id":"${TRACE_ID}","mode":"devel","launch_mode":"${LAUNCH_MODE}","started_at":"$(date -Is)","bag":"${TRACE_DIR}/run.bag","decision_log":"${TRACE_DIR}/decision.jsonl","bag_topics":"${TRACE_BAG_TOPICS}","status":"starting"}
EOF
  echo "Trace enabled: ${TRACE_DIR}"
fi

cleanup_trace() {
  if [ -n "${ROSBAG_PID}" ]; then
    kill "${ROSBAG_PID}" 2>/dev/null || true
    wait "${ROSBAG_PID}" 2>/dev/null || true
  fi
  if [ "$TRACE_ENABLE" = "1" ] || [ "$TRACE_ENABLE" = "true" ] || [ "$TRACE_ENABLE" = "TRUE" ]; then
    cat > "${TRACE_DIR}/manifest.json" <<EOF
{"trace_id":"${TRACE_ID}","mode":"devel","launch_mode":"${LAUNCH_MODE}","finished_at":"$(date -Is)","bag":"${TRACE_DIR}/run.bag","decision_log":"${TRACE_DIR}/decision.jsonl","bag_topics":"${TRACE_BAG_TOPICS}","status":"finished"}
EOF
  fi
}
trap cleanup_trace EXIT

if [ "$LAUNCH_MODE" = "random" ]; then
    echo "Mode: random (procedural map)"
    roslaunch bringup_test sim_random_main.launch \
      flight_type:=2 max_vel:=0.6 max_acc:=1.0 \
      &>"${ROSLAUNCH_LOG}" &
else
    echo "Mode: scenegraph (PCD + offline scene graph)"
    if [ ! -f "$MAP_PCD" ]; then
      echo "Missing scene map: $MAP_PCD" >&2
      echo "Mount or create .data/pcd/J30V2_latest.pcd, or set LAUNCH_MODE=random." >&2
      exit 1
    fi
    roslaunch bringup_test sim_scenegraph_main.launch \
      flight_type:=2 max_vel:=0.6 max_acc:=1.0 \
      map_pcd:="$MAP_PCD" \
      &>"${ROSLAUNCH_LOG}" &
fi
LAUNCH_PID=$!

# ── health check ──────────────────────────────────────────────────
TOPICS_OK=0
for i in $(seq 1 30); do
  sleep 2
  if ! kill -0 $LAUNCH_PID 2>/dev/null; then
    echo "❌ roslaunch died at t${i}. Log:"
    tail -40 "${ROSLAUNCH_LOG}"
    exit 1
  fi
  if [ $TOPICS_OK -eq 0 ] && \
     rostopic info /map_generator/global_cloud 2>/dev/null | grep -q "Publishers:" && \
     rostopic info /drone_0_visual_slam/odom 2>/dev/null | grep -q "Publishers:"; then
    TOPICS_OK=1
    echo "✅ map + odom ready (t=${i})"
    if [ "$TRACE_ENABLE" = "1" ] || [ "$TRACE_ENABLE" = "true" ] || [ "$TRACE_ENABLE" = "TRUE" ]; then
      if command -v rosbag >/dev/null 2>&1; then
        rosbag record -O "${TRACE_DIR}/run.bag" ${TRACE_BAG_TOPICS} >"${TRACE_DIR}/rosbag.log" 2>&1 &
        ROSBAG_PID=$!
        echo "Trace bag recording: ${TRACE_DIR}/run.bag"
      else
        echo "rosbag not found; decision trace will still be collected." | tee "${TRACE_DIR}/rosbag.log"
      fi
    fi
  fi
  if rostopic info /drone_0_planning/pos_cmd 2>/dev/null | grep -q "Publishers:"; then
    echo "✅ pos_cmd ready (t=${i})"
    echo ""
    echo "=== EGO planner running ==="
    echo ""
    echo "=== startup log (errors) ==="
    grep -i 'error\|fatal' "${ROSLAUNCH_LOG}" | tail -10 || echo "(no errors)"
    echo "--------------------------------------------------------------"
    break
  fi
done

wait $LAUNCH_PID

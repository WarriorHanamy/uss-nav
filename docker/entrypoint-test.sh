#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash
source /workspace/devel/setup.bash

# Bind mount preserves source tree (ws_main/src/planner/...), but the
# catkin devel space was built with flattened paths (e.g. /bringup_test).
# Override ROS_PACKAGE_PATH to point to the actual source locations.
export ROS_PACKAGE_PATH="/workspace/src/mission_executive:\
/workspace/src/perception/global_belief:\
/workspace/src/planners/ego_planner/plan_env:\
/workspace/src/planners/ego_planner/path_searching:\
/workspace/src/planners/ego_planner/traj_opt:\
/workspace/src/planners/ego_planner/map_interface:\
/workspace/src/planners/ego_planner/plan_manage:\
/workspace/src/uav_simulator/so3_quadrotor_simulator:\
/workspace/src/uav_simulator/so3_control:\
/workspace/src/uav_simulator/local_sensing:\
/workspace/src/uav_simulator/map_generator:\
/workspace/src/perception/scene_graph:\
/workspace/src/perception/camera_fov:\
/workspace/src/utils/quadrotor_msgs:\
/workspace/src/utils/traj_utils:\
/workspace/src/utils/uav_utils:\
/opt/ros/noetic/share"

TEST_ID="${TEST_ID:-default}"
MQTT_HOST="${MQTT_HOST:-host.docker.internal}"
DURATION="${DURATION:-300}"
FLIGHT_TYPE="${FLIGHT_TYPE:-2}"
MAX_VEL="${MAX_VEL:-0.6}"
MAX_ACC="${MAX_ACC:-1.0}"
OBS_NUM="${OBS_NUM:-30}"
X_SIZE="${X_SIZE:-50}"
Y_SIZE="${Y_SIZE:-30}"
TRACE_ENABLE="${TRACE_ENABLE:-0}"
TRACE_ID="${TRACE_ID:-${TEST_ID}}"
TRACE_DIR="${TRACE_DIR:-/workspace/.artifacts/traces/${TRACE_ID}}"
DEFAULT_TRACE_BAG_TOPICS="/tf /tf_static /bridge/Instruct /Instruct_res /planner/fsm_state /tracking_finish /drone_0_visual_slam/odom /drone_0_ego_planner_node/local_goal /drone_0_planning/pos_cmd /planning/fsm_vis /planning/fsm_path /map_generator/global_cloud /drone_0_ego_planner_node/grid_map/occupancy_inflate /drone_0_ego_planner_node/grid_map/occupancy_inflateBig"
TRACE_BAG_TOPICS="${TRACE_BAG_TOPICS:-$DEFAULT_TRACE_BAG_TOPICS}"
ROSLAUNCH_LOG="/tmp/roslaunch.log"
BRIDGE_LOG="/tmp/bridge.log"
ROSBAG_PID=""

echo "=== EGO Planner Test [$TEST_ID] ==="
echo "  duration=${DURATION}s  flight_type=${FLIGHT_TYPE}  max_vel=${MAX_VEL}  max_acc=${MAX_ACC}"
echo "  obs_num=${OBS_NUM}  x_size=${X_SIZE}  y_size=${Y_SIZE}"

if [ "$TRACE_ENABLE" = "1" ] || [ "$TRACE_ENABLE" = "true" ] || [ "$TRACE_ENABLE" = "TRUE" ]; then
  mkdir -p "$TRACE_DIR"
  export TRACE_ID TRACE_DIR DECISION_TRACE_FILE="${TRACE_DIR}/decision.jsonl"
  ROSLAUNCH_LOG="${TRACE_DIR}/roslaunch.log"
  BRIDGE_LOG="${TRACE_DIR}/bridge.log"
  cat > "${TRACE_DIR}/manifest.json" <<EOF
{"trace_id":"${TRACE_ID}","mode":"test","test_id":"${TEST_ID}","started_at":"$(date -Is)","bag":"${TRACE_DIR}/run.bag","decision_log":"${TRACE_DIR}/decision.jsonl","bag_topics":"${TRACE_BAG_TOPICS}","status":"starting"}
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
{"trace_id":"${TRACE_ID}","mode":"test","test_id":"${TEST_ID}","finished_at":"$(date -Is)","bag":"${TRACE_DIR}/run.bag","decision_log":"${TRACE_DIR}/decision.jsonl","bag_topics":"${TRACE_BAG_TOPICS}","status":"finished"}
EOF
  fi
}
trap cleanup_trace EXIT

# Headless display
export DISPLAY=:99
Xvfb :99 -screen 0 1280x1024x24 &
sleep 1

# Generate a custom map YAML
sed -e "s/obs_num:.*/obs_num: ${OBS_NUM}/" \
    -e "s/x_size:.*/x_size: ${X_SIZE}/" \
    -e "s/y_size:.*/y_size: ${Y_SIZE}/" \
    /workspace/src/bringup_test/params/sim_ego_map.yaml \
    > /tmp/sim_random_map_${TEST_ID}.yaml

# Patch the launch to use custom map (copy to /tmp since source is read-only)
cp /workspace/src/bringup_test/launch/sim_random_map.launch \
   /tmp/sim_random_map_${TEST_ID}.launch
cp /workspace/src/bringup_test/launch/sim_random_main.launch \
   /tmp/sim_random_main_${TEST_ID}.launch
sed -i 's|params/sim_ego_map.yaml|/tmp/sim_random_map_'"${TEST_ID}"'.yaml|' \
   /tmp/sim_random_map_${TEST_ID}.launch
sed -i 's|\$(find bringup_test)/launch/sim_random_map.launch|/tmp/sim_random_map_'"${TEST_ID}"'.launch|' \
   /tmp/sim_random_main_${TEST_ID}.launch

# Start ego planner
echo "Starting ego planner (headless)..."
roslaunch /tmp/sim_random_main_${TEST_ID}.launch \
  flight_type:=$FLIGHT_TYPE max_vel:=$MAX_VEL max_acc:=$MAX_ACC \
  &>"${ROSLAUNCH_LOG}" &
LAUNCH_PID=$!

# Wait for planner to be ready
for i in $(seq 1 30); do
  sleep 2
  if ! kill -0 $LAUNCH_PID 2>/dev/null; then
    echo "❌ roslaunch died. Log:"
    tail -20 "${ROSLAUNCH_LOG}"
    exit 1
  fi
  if rostopic info /drone_0_planning/pos_cmd 2>/dev/null | grep -q "Publishers:"; then
    echo "✅ Planner ready (t=${i}s)"
    if [ "$TRACE_ENABLE" = "1" ] || [ "$TRACE_ENABLE" = "true" ] || [ "$TRACE_ENABLE" = "TRUE" ]; then
      if command -v rosbag >/dev/null 2>&1; then
        rosbag record -O "${TRACE_DIR}/run.bag" ${TRACE_BAG_TOPICS} >"${TRACE_DIR}/rosbag.log" 2>&1 &
        ROSBAG_PID=$!
        echo "Trace bag recording: ${TRACE_DIR}/run.bag"
      else
        echo "rosbag not found; decision trace will still be collected." | tee "${TRACE_DIR}/rosbag.log"
      fi
    fi
    break
  fi
done

# Start MQTT bridge
echo "Starting MQTT bridge → ${MQTT_HOST}:1883 ..."
python3 /bridge/ego_mqtt_bridge.py \
  --mqtt-host "$MQTT_HOST" \
  --mqtt-port 1883 \
  --test-id "$TEST_ID" \
  --topic-prefix test \
  &>"${BRIDGE_LOG}" &
BRIDGE_PID=$!

echo "✅ Test running for ${DURATION}s"
sleep "$DURATION"

# Cleanup
echo "Test complete, stopping..."
kill $BRIDGE_PID 2>/dev/null || true
kill $LAUNCH_PID 2>/dev/null || true
if [ -n "${ROSBAG_PID}" ]; then
  kill "${ROSBAG_PID}" 2>/dev/null || true
  wait "${ROSBAG_PID}" 2>/dev/null || true
  ROSBAG_PID=""
fi

# Clean up temp files
rm -f /tmp/sim_random_map_${TEST_ID}.launch /tmp/sim_random_main_${TEST_ID}.launch
echo "=== Test [$TEST_ID] done ==="

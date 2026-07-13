#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# Bind mount preserves source tree (ws_main/src/planner/...), but the
# catkin devel space was built with flattened paths (e.g. /sim_bringup).
# Override ROS_PACKAGE_PATH to point to the actual source locations.
export ROS_PACKAGE_PATH="/catkin_ws/src/planner/sim_bringup:\
/catkin_ws/src/planner/ego_plannerv3/plan_env:\
/catkin_ws/src/planner/ego_plannerv3/path_searching:\
/catkin_ws/src/planner/ego_plannerv3/traj_opt:\
/catkin_ws/src/planner/ego_plannerv3/map_interface:\
/catkin_ws/src/planner/ego_plannerv3/plan_manage:\
/catkin_ws/src/planner/uav_simulator/so3_quadrotor_simulator:\
/catkin_ws/src/planner/uav_simulator/so3_control:\
/catkin_ws/src/planner/uav_simulator/local_sensing:\
/catkin_ws/src/planner/uav_simulator/map_generator:\
/catkin_ws/src/planner/exploration/exploration_manager:\
/catkin_ws/src/planner/exploration/perception_utils:\
/catkin_ws/src/planner/exploration/lkh_tsp_solver:\
/catkin_ws/src/planner/scene_graph:\
/catkin_ws/src/utils/quadrotor_msgs:\
/catkin_ws/src/utils/traj_utils:\
/catkin_ws/src/utils/uav_utils:\
/catkin_ws/src/utils/catkin_simple:\
/catkin_ws/src/utils/pose_utils:\
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

echo "=== EGO Planner Test [$TEST_ID] ==="
echo "  duration=${DURATION}s  flight_type=${FLIGHT_TYPE}  max_vel=${MAX_VEL}  max_acc=${MAX_ACC}"
echo "  obs_num=${OBS_NUM}  x_size=${X_SIZE}  y_size=${Y_SIZE}"

# Headless display
export DISPLAY=:99
Xvfb :99 -screen 0 1280x1024x24 &
sleep 1

# Generate a custom map YAML
sed -e "s/obs_num:.*/obs_num: ${OBS_NUM}/" \
    -e "s/x_size:.*/x_size: ${X_SIZE}/" \
    -e "s/y_size:.*/y_size: ${Y_SIZE}/" \
    /catkin_ws/src/planner/sim_bringup/params/sim_ego_map.yaml \
    > /tmp/sim_ego_map_${TEST_ID}.yaml

# Patch the launch to use custom map (copy to /tmp since source is read-only)
cp /catkin_ws/src/planner/sim_bringup/launch/sim_ego_map.launch \
   /tmp/sim_ego_map_${TEST_ID}.launch
cp /catkin_ws/src/planner/sim_bringup/launch/sim_ego_main.launch \
   /tmp/sim_ego_main_${TEST_ID}.launch
sed -i 's|params/sim_ego_map.yaml|/tmp/sim_ego_map_'"${TEST_ID}"'.yaml|' \
   /tmp/sim_ego_map_${TEST_ID}.launch
sed -i 's|\$(find sim_bringup)/launch/sim_ego_map.launch|/tmp/sim_ego_map_'"${TEST_ID}"'.launch|' \
   /tmp/sim_ego_main_${TEST_ID}.launch

# Start ego planner
echo "Starting ego planner (headless)..."
roslaunch /tmp/sim_ego_main_${TEST_ID}.launch \
  flight_type:=$FLIGHT_TYPE max_vel:=$MAX_VEL max_acc:=$MAX_ACC \
  use_rviz:=false \
  &>/tmp/roslaunch.log &
LAUNCH_PID=$!

# Wait for planner to be ready
for i in $(seq 1 30); do
  sleep 2
  if ! kill -0 $LAUNCH_PID 2>/dev/null; then
    echo "❌ roslaunch died. Log:"
    tail -20 /tmp/roslaunch.log
    exit 1
  fi
  if rostopic info /drone_0_planning/pos_cmd 2>/dev/null | grep -q "Publishers:"; then
    echo "✅ Planner ready (t=${i}s)"
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
  &>/tmp/bridge.log &
BRIDGE_PID=$!

echo "✅ Test running for ${DURATION}s"
sleep "$DURATION"

# Cleanup
echo "Test complete, stopping..."
kill $BRIDGE_PID 2>/dev/null || true
kill $LAUNCH_PID 2>/dev/null || true

# Clean up temp files
rm -f /tmp/sim_ego_map_${TEST_ID}.launch /tmp/sim_ego_main_${TEST_ID}.launch
echo "=== Test [$TEST_ID] done ==="

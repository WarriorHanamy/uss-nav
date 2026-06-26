#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

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
    /catkin_ws/src/sim_bringup/params/sim_ego_map.yaml \
    > /tmp/sim_ego_map_${TEST_ID}.yaml

# Patch the launch to use custom map
MAP_LAUNCH_FILE=/catkin_ws/src/sim_bringup/launch/sim_ego_map.launch
cp "$MAP_LAUNCH_FILE" "${MAP_LAUNCH_FILE}.bak"
sed -i 's|params/sim_ego_map.yaml|/tmp/sim_ego_map_'"${TEST_ID}"'.yaml|' "$MAP_LAUNCH_FILE"

# Start ego planner
echo "Starting ego planner (headless)..."
roslaunch sim_bringup sim_ego_main.launch \
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

# Restore launch file
mv "${MAP_LAUNCH_FILE}.bak" "$MAP_LAUNCH_FILE" 2>/dev/null || true
echo "=== Test [$TEST_ID] done ==="

#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash
source /workspace/devel/setup.bash

echo "=== USS-NAV Release: SceneGraph Online Construction ==="

# ── display setup ──────────────────────────────────────────────────
if [ -n "$DISPLAY" ] && [ -d "/tmp/.X11-unix" ]; then
  echo "X11 display: ${DISPLAY}"
else
  echo "No DISPLAY, starting Xvfb..."
  export DISPLAY=:99
  Xvfb :99 -screen 0 1280x1024x24 &
  sleep 1
fi

# ── launch simulation ─────────────────────────────────────────────
echo "Starting map_generator + quadrotor sim + EGO planner..."
echo "RViz managed by ~/rviz_ws (external), not started here."

roslaunch bringup_test sim_random_main.launch \
  flight_type:=2 max_vel:=0.6 max_acc:=1.0 \
  &>/tmp/roslaunch.log &
LAUNCH_PID=$!

# ── health check: wait for odom ──────────────────────────────────
TOPICS_OK=0
for i in $(seq 1 30); do
  sleep 2
  if ! kill -0 $LAUNCH_PID 2>/dev/null; then
    echo "roslaunch died at t${i}. Log:"
    tail -40 /tmp/roslaunch.log
    exit 1
  fi
  if [ $TOPICS_OK -eq 0 ] && \
     rostopic info /map_generator/global_cloud 2>/dev/null | grep -q "Publishers:" && \
     rostopic info /drone_0_visual_slam/odom 2>/dev/null | grep -q "Publishers:"; then
    TOPICS_OK=1
    echo "map + odom ready (t=${i})"
  fi
  if rostopic info /drone_0_planning/pos_cmd 2>/dev/null | grep -q "Publishers:"; then
    echo "pos_cmd ready (t=${i})"
    break
  fi
done

# ── launch YOLOE for SceneGraph object pipeline ───────────────────
REAL_YOLOE="${REAL_YOLOE:-0}"
if [ "$REAL_YOLOE" = "1" ] && [ -f /workspace/.pretrained/yoloe-11m-seg-pf.pt ]; then
  echo "Starting real YOLOE (pretrained weights found)..."
  rosparam load /workspace/src/bringup_test/params/yoloe_pretrained.yaml
  python3 /workspace/src/perception/yoloe/predict_realtime_cam_sim.py \
    _model_path:=/workspace/.pretrained/yoloe-11m-seg-pf.pt \
    _prompt_model_path:=/workspace/.pretrained/yoloe-11m-seg.pt \
    _clip_model_path:=/workspace/.pretrained/mobileclip_blt.pt \
    _prompt_file_path:=/workspace/src/perception/yoloe/prompt/prompt.txt \
    _odom_topic:=/drone_0_visual_slam/odom \
    &>/tmp/yoloe_server.log &
  YOLOE_PID=$!
  echo "Real YOLOE started (PID=$YOLOE_PID)"
else
  echo "Starting fake YOLOE (object detection simulation)..."
  python3 /workspace/src/perception/yoloe/fake_realtime_cam_sim.py \
    _odom_topic:=/drone_0_visual_slam/odom \
    _publish_duration:=999999 \
    _label:=tree \
    &>/tmp/fake_yoloe.log &
  YOLOE_PID=$!
  echo "Fake YOLOE started (PID=$YOLOE_PID)"
fi

# ── print status ──────────────────────────────────────────────────
echo ""
if [ "$REAL_YOLOE" = "1" ]; then
  YOLOE_MODE="real YOLOE"
else
  YOLOE_MODE="fake YOLOE"
fi
echo "=== SceneGraph Online Construction Running ==="
echo "  - Skeleton generation from LiDAR"
echo "  - Area clustering (Leiden community detection)"
echo "  - Object pipeline (${YOLOE_MODE} -> EncodeMask -> ObjectMap)"
echo "  - Planning (EGO Planner)"
echo ""
echo "Topics:"
echo "  /drone_0_visual_slam/odom     - quadrotor odometry"
echo "  /map_generator/global_cloud   - global point cloud"
echo "  /livox/lidar_world            - simulated LiDAR"
  echo "  /yoloe/encodemask             - ${YOLOE_MODE} object detections"
echo "  /scene_graph/vis              - SceneGraph visualization"
echo "  /drone_0_planning/pos_cmd     - planning commands"
echo ""
echo "Press Ctrl+C to stop."
echo "--------------------------------------------------------------"

wait $LAUNCH_PID

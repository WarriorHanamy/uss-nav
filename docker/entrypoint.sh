#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

echo "=== EGO Planner Simulation ==="

# ── display setup ──────────────────────────────────────────────────
USE_RVIZ=false
if [ -n "$DISPLAY" ] && [ -d "/tmp/.X11-unix" ]; then
  echo "✅ X11 display: ${DISPLAY}"
  USE_RVIZ=true
else
  echo "No DISPLAY, starting Xvfb..."
  export DISPLAY=:99
  Xvfb :99 -screen 0 1280x1024x24 &
  sleep 1
fi

# ── launch ────────────────────────────────────────────────────────
echo "Starting map_generator + quadrotor sim + EGO planner..."
echo "use_rviz=$USE_RVIZ"

roslaunch sim_bringup sim_ego_main.launch \
  flight_type:=2 max_vel:=0.6 max_acc:=1.0 \
  use_rviz:="$USE_RVIZ" \
  &>/tmp/roslaunch.log &
LAUNCH_PID=$!

# ── health check ──────────────────────────────────────────────────
TOPICS_OK=0
for i in $(seq 1 30); do
  sleep 2
  if ! kill -0 $LAUNCH_PID 2>/dev/null; then
    echo "❌ roslaunch died at t${i}. Log:"
    tail -40 /tmp/roslaunch.log
    exit 1
  fi
  if [ $TOPICS_OK -eq 0 ] && \
     rostopic info /map_generator/global_cloud 2>/dev/null | grep -q "Publishers:" && \
     rostopic info /drone_0_visual_slam/odom 2>/dev/null | grep -q "Publishers:"; then
    TOPICS_OK=1
    echo "✅ map + odom ready (t=${i})"
  fi
  if rostopic info /drone_0_planning/pos_cmd 2>/dev/null | grep -q "Publishers:"; then
    echo "✅ pos_cmd ready (t=${i})"
    echo ""
    echo "=== EGO planner running ==="
    echo ""
    echo "=== rviz log ==="
    grep -i 'rviz\|error\|warn\|fatal\|xcb\|display' /tmp/roslaunch.log | tail -10 || echo "(no relevant lines)"
    echo "--------------------------------------------------------------"
    break
  fi
done

wait $LAUNCH_PID

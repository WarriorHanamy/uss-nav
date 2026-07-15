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
echo "use_rviz=false (RViz launched externally from ~/rviz_ws)"

LAUNCH_MODE="${LAUNCH_MODE:-scenegraph}"
MAP_PCD="${MAP_PCD:-/workspace/.data/pcd/J30V2_latest.pcd}"

if [ "$LAUNCH_MODE" = "random" ]; then
    echo "Mode: random (procedural map)"
    roslaunch bringup_test sim_random_main.launch \
      flight_type:=2 max_vel:=0.6 max_acc:=1.0 \
      use_rviz:=false \
      &>/tmp/roslaunch.log &
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
      use_rviz:=false \
      &>/tmp/roslaunch.log &
fi
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
    echo "=== startup log (errors) ==="
    grep -i 'error\|fatal' /tmp/roslaunch.log | tail -10 || echo "(no errors)"
    echo "--------------------------------------------------------------"
    break
  fi
done

wait $LAUNCH_PID

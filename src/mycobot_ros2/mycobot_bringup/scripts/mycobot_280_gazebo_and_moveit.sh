#!/bin/bash

echo "=============================="
echo " MyCobot Gazebo + MoveIt Bringup"
echo "=============================="

# ----------- CLEAN EXIT HANDLER -----------
cleanup() {
    echo ""
    echo "Shutting down simulation (clean)..."

    # Kill only processes we launched
    kill ${GAZEBO_PID} 2>/dev/null || true
    kill ${MOVEIT_PID} 2>/dev/null || true

    wait
    echo "Shutdown complete."
    exit 0
}

trap cleanup SIGINT SIGTERM

# ----------- LAUNCH GAZEBO -----------
echo "[1/2] Launching Gazebo simulation..."

ros2 launch mycobot_gazebo mycobot.gazebo.launch.py \
    load_controllers:=true \
    world_file:=empty.world \
    use_camera:=true \
    use_rviz:=false \
    use_robot_state_pub:=true \
    use_sim_time:=true \
    x:=0.0 \
    y:=0.0 \
    z:=0.03 \
    roll:=0.0 \
    pitch:=0.0 \
    yaw:=0.0 &

GAZEBO_PID=$!

echo "Waiting for Gazebo + controllers..."
sleep 20

# ----------- LAUNCH MOVEIT -----------
echo "[2/2] Launching MoveIt (RViz)..."

ros2 launch arm_moveit_config demo.launch.py \
    use_sim_time:=true \
    use_robot_state_pub:=false \
    use_gazebo:=true &

MOVEIT_PID=$!

# ----------- WAIT -----------
wait

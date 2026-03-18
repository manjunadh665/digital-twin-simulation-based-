#!/bin/bash
sleep 5
ros2 bag record \
  --topics /joint_states /tf /clock \
  --qos-profile-overrides-path /home/hyndavi/ros2_ws/src/digital_twin/config/qos_override.yaml \
  -o /home/hyndavi/arm_bags/session_$(date +%Y%m%d_%H%M%S)
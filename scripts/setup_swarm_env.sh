#!/bin/bash

# CycloneDDS — обязательно для 20 роботов
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/ros2_ws/src/scripts/cyclonedds_swarm.xml
export ROS_DOMAIN_ID=42

# Увеличиваем UDP буферы ядра (от discovery storms)
sudo sysctl -w net.core.rmem_max=8388608 2>/dev/null
sudo sysctl -w net.core.rmem_default=8388608 2>/dev/null

echo "Fleet environment ready: CycloneDDS, DOMAIN_ID=$ROS_DOMAIN_ID"
#!/bin/bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /usr/share/gazebo/setup.bash
export GAZEBO_MODEL_PATH=/workspaces/gazebo_tutorial/models:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=/workspaces/gazebo_tutorial/worlds:/workspaces/gazebo_tutorial/resource:${GAZEBO_RESOURCE_PATH}
export GAZEBO_PLUGIN_PATH=/workspaces/gazebo_tutorial/bin:${GAZEBO_PLUGIN_PATH}
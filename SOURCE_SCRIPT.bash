#!/usr/bin/env bash
set -e

GREEN="\e[32m"
RESET="\e[0m"

# ros variables
ROS_DISTRO="$(cat .ros_distro)"
ROS_PATH="$(cat .ros_path)$ROS_DISTRO"

echo -n "You should be running this script using \`" && echo -n "$GREEN. ./SOURCE_SCRIPT.bash$RESET" && echo "\`."
echo "If you didn't do so, please restart the script."
echo

echo Sourcing virtual environment...
. .venv/bin/activate
echo Done!
echo

echo Sourcing ROS 2 environment files...
. install/local_setup.bash || true
. $ROS_PATH/setup.bash || true
echo Done!

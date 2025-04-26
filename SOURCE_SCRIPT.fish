#!/usr/bin/env fish

set ROS2_WORKSPACE_LOCATION (dirname (readlink -m (status --current-filename)))

# grab ros 2 properties
set ROS_DISTRO (cat .ros_distro)
set ROS_PATH (cat .ros_path)$ROS_DISTRO

echo -n "You should be running this script using `" && set_color green && echo -n ". ./SOURCE_SCRIPT.fish" && set_color normal && echo "`."
echo "If you didn't do so, please restart the script."
echo

echo Sourcing virtual environment...
. .venv/bin/activate.fish
echo Done!

echo Sourcing ROS 2 environment files...
bass source $ROS2_WORKSPACE_LOCATION/install/local_setup.bash || true
bass source $ROS_PATH/setup.bash || true
echo Done!

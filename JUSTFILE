# use fish for scripting
set shell := ["bash", "-c"]

ros2_workspace_dir := justfile_directory()

build:
    @ echo "Building the ROS 2 workspace...";
    colcon build --symlink-install
    @ echo "Build complete!";

sim: build
    #!/usr/bin/env bash

    # so... we've just completed the build.
    #
    # let's source any new packages it made
    . ./SOURCE_SCRIPT.bash

    # we'll also kill the simulator if it's still alive (happens pretty often
    # from previous runs)
    killall -9 gazebo & killall -9 gzserver & killall -9 gzclient & killall -9 ign & killall -9 ruby & killall -9 simulator

    # start the simulator by running its launch file!
    echo "The simulation is about to begin...";
    ros2 launch simulator sim.launch.py

clean:
    @ echo "Attempting to clean build files...";
    rm -rfd install/ install/ log/ .cargo/
    rm -rfd build/
    - rm -rfd .venv/
    @ echo "Build files have been removed!";

test: build
    @ echo "Running tests..."
    colcon test --event-handlers console_direct+
    @ echo "Test run complete!"

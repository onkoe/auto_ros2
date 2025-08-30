# use fish for scripting
set shell := ["bash", "-c"]

ros2_workspace_dir := justfile_directory()

# aliases let users provide multiple commands for one
alias fetch := get
alias update := get

# Prints this help message.
help:
    @ echo "Welcome to the SoRo Justfile! Here are the commands you can run:"
    @ just --list

# Builds the project using ROS 2 tooling.
build:
    @ echo "Building the ROS 2 workspace...";

    @ # first, perform an "initial build" for only the custom interfaces package
    colcon build --symlink-install \
        --packages-select custom_interfaces \
        --cmake-args \
        -DPython_ROOT_DIR="$CONDA_PREFIX" \
        -DPython_EXECUTABLE="$(python -c 'import sys; print(sys.executable)')"

    @ # then, we can do a "full build" of the entire workspace
    colcon build --symlink-install \
        --cmake-args \
        -DPython_ROOT_DIR="$CONDA_PREFIX" \
        -DPython_EXECUTABLE="$(python -c 'import sys; print(sys.executable)')"
    @ echo "Build complete!";

# Runs the Gazebo simulation.
sim: build
    #!/usr/bin/env bash

    # so... we've just completed the build.
    #
    # let's source any new packages it made
    . ./SOURCE_SCRIPT.bash

    # set gazebo plugin search path
    #
    # note: name is IGN_GAZEBO_SYSTEM_PLUGIN_PATH for our old-ass Gazebo
    # Fortress (6), but for newer versions, you'll want to use
    # `GAZEBO_PLUGIN_PATH` instead.
    #
    # anyway. let's gonna grab the ros installation path and add it onto
    # there...
    ROS_INSTALLATION_PATH="$(cat {{ros2_workspace_dir}}/.ros_path)$(cat {{ros2_workspace_dir}}/.ros_distro)/lib"
    export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$ROS_INSTALLATION_PATH
    echo "Adding these paths to Gazebo's plugin path: $IGN_GAZEBO_SYSTEM_PLUGIN_PATH"

    # we'll also kill the simulator if it's still alive (happens pretty often
    # from previous runs)
    (killall -9 gazebo & killall -9 gzserver & killall -9 gzclient & killall -9 ign & killall -9 ruby & killall -9 simulator) || true

    # start the simulator by running its launch file!
    echo "The simulation is about to begin...";
    ros2 launch simulator sim.launch.py

# Removes extra build files.
clean:
    @ echo "Attempting to clean build files...";
    rm -rfd install/ install/ log/ .cargo/
    rm -rfd build/
    - rm -rfd .venv/
    @ echo "Build files have been removed!";

# Runs our test suite through `colcon`.
test: build
    #!/usr/bin/env bash
    set -e
    echo "Running tests..."
    colcon test --event-handlers console_direct+
    echo "Test run complete!"

# Fetches the environment's dependencies.
get:
    #!/usr/bin/env bash
    set -e
    echo "Fetching newest dependencies..."

    # ensure pixi is installed
    if ! command -v pixi &> /dev/null; then
        echo "Pixi is not installed! Please see its installation instructions below:"
        echo "https://github.com/prefix-dev/pixi/#installation"
        exit 255
    fi

    # great, pixi is installed!
    #
    # install dependencies and guide the user through continuing...
    echo "Pixi is installed properly. Updating Pixi environment..."
    pixi i

    echo
    echo -e "\e[32mDependencies should be installed successfully! \e[39m"
    echo
    echo -e "\e[32mTo continue, run \`\e[34mpixi shell\e[32m\`. It'll start a shell with access to ROS 2, the simulator, and more! \e[39m"

# Autonomous

This repo is home to the Sooner Rover Team's [ROS 2](https://ros.org/) codebase. For more about this project and our plans, see [the Autonomous kanban board](https://github.com/orgs/Sooner-Rover-Team/projects/4/views/1).

## Join Us

Want to join SoRo? Hop into [our Discord server](https://discord.gg/qBHR26S6N5) and introduce yourself! We'll help you find the right place. :D

To know about events and see our progress, you can also [follow our Instagram](https://www.instagram.com/soonerrover/)!

## Contributing

To find something to work on, you can take a look at our [task board](https://github.com/orgs/Sooner-Rover-Team/projects/4/views/1)! This "kanban board" contains info about what we need to do this year.

For information about how we write code and put it on GitHub, please see [our documentation on contributions, `CONTRIBUTING.md`](./CONTRIBUTING.md).

## Start Coding

To start coding, you need a few things, depending on your operating system:

- Windows: Install Ubuntu on WSL using this tutorial. Then, restart your computer and open click "Ubuntu" in your start menu.
  - (1) Install Ubuntu on WSL [using this tutorial](https://mattzaskeonline.info/blog/2024-04/getting-started-wsl-quick-installation-guide)!
  - (2) Restart your computer.
  - (3) Click Ubuntu in your Start Menu.
- macOS: (Probably) no actions necessary.
  - An Ubuntu VM might help you stay on track, but that's kinda difficult and usually requires paid software.
  - Type `git` into your terminal.
- Linux: No actions necessary!
  - Our setup code below assumes you're running Ubuntu.
  - Other distros may or may not work - they're untested.
    - (though Fedora works for me)

### Step 1: Install `git`

If you're on Ubuntu (natively or on Windows through WSL), `git` won't be installed for you.

To get it, open a terminal, then type this command: `sudo apt-get install git -y`:

```bash
# for reference, these gray hashtag things are comments - you DON'T need to
# type them!
#
# anyway, we're starting off in your user's home folder.
#
# let's start by grabbing `git` and `curl`:
barrett@farts ~> sudo apt-get install git curl -y

Reading package lists... Done
Building dependency tree... Done
Reading state information... Done

(...)

Setting up curl (8.5.0-2ubuntu10.6)
Setting up git (1:2.43.0-1ubuntu7.3)
Processing triggers for libc-bin (2.39-0ubuntu8.5)
Processing triggers for man-db (2.12.0-4build2)
Processing triggers for mailcap (3.70+nmu1ubuntu1)

All done!
barrett@farts ~>
```

### Step 2: Get `pixi`

Now, we need the Pixi project manager tool.

1. Get it by pasting this command: `curl -fsSL https://pixi.sh/install.sh | sh`
2. Then, restart your terminal.

```bash
# now, we need to install something called `pixi` on the computer.
#
# `pixi` manages ROS 2, Python, Rust, and other tooling, so it's required to
# continue...
barrett@farts ~> curl -fsSL https://pixi.sh/install.sh | sh
This script will automatically download and install Pixi (latest) for you.
Getting it from this url: https://github.com/prefix-dev/pixi/releases/latest/download/pixi-x86_64-unknown-linux-musl.tar.gz
  % Total    % Received % Xferd  Average Speed   Time    Time     Time  Current
                                 Dload  Upload   Total   Spent    Left  Speed
  0     0    0     0    0     0      0      0 --:--:-- --:--:-- --:--:--     0
  0     0    0     0    0     0      0      0 --:--:-- --:--:-- --:--:--     0
100 23.2M  100 23.2M    0     0  27.6M      0 --:--:-- --:--:-- --:--:-- 53.7M
The 'pixi' binary is installed into '/home/barrett/.pixi/bin'
Updating '/home/barrett/.bashrc'
Please restart or source your shell.

# as it said, we need to source the shell!
#
# you can either:
#
# 1. open a new terminal, or
# 2. type this:
barrett@farts ~> source ~/.bashrc

# let's ensure Pixi is installed by running it:
barrett@farts ~> pixi --version
pixi 0.53.0

# yup, it's installed!
```

### Step 3: Get this repo (folder) on your computer

Now that we have `git` and `pixi`, we can put this GitHub repo on your computer!

1. Ensure you're in the right folder by typing `pwd`
   - The "right folder" is just your home folder.
2. Use `git clone` to download it on your computer.

```bash
# let's check that I'm in my home folder:
barrett@farts ~> pwd
/home/barrett

# why, yes I am!
#
# if you're not in the `/home/YOUR_USERNAME` folder, just type `cd` like so:
# barrett@farts ~/OTHER_FOLDER> cd

# great! now, let's use `git` to grab the `auto_ros2` repo:
barrett@farts ~> git clone https://github.com/Sooner-Rover-Team/auto_ros2
Cloning into 'auto_ros2'...
remote: Enumerating objects: 3889, done.
remote: Counting objects: 100% (886/886), done.
remote: Compressing objects: 100% (278/278), done.
remote: Total 3889 (delta 668), reused 616 (delta 604), pack-reused 3003 (from 1)
Receiving objects: 100% (3889/3889), 2.12 MiB | 5.83 MiB/s, done.
Resolving deltas: 100% (2499/2499), done.

# now, move your terminal to that folder by typing `cd auto_ros2`:
barrett@farts ~> cd auto_ros2/
barrett@farts ~/auto_ros2 (main)> # we're in!

# and check what's in the folder with `ls`:
barrett@farts ~/auto_ros2 (main)> ls
cam.yml     Cargo.toml       docker  JUSTFILE  pixi.lock  pyproject.toml  rviz                SOURCE_SCRIPT.fish  uv.lock
Cargo.lock  CONTRIBUTING.md  docs    lib       pixi.toml  README.md       SOURCE_SCRIPT.bash  src
```

### Step 4: Install dependencies with `pixi`

We can use `pixi` to install everything needed to run + change the code:

```bash
# note: before running this command, make sure you're in the
# `/home/YOUR_USER/auto_ros2` folder!
#
# like so:
barrett@farts ~/auto_ros2 (main)> pwd
/home/barrett/auto_ros2

# great, that's perfect! now, install the dependencies with `pixi`:
barrett@farts ~/auto_ros2 (main)> pixi i
 WARN Skipped running the post-link scripts because `run-post-link-scripts` = `false`
	- bin/.librsvg-pre-unlink.sh

To enable them, run:
	pixi config set --local run-post-link-scripts insecure

More info:
	https://pixi.sh/latest/reference/pixi_configuration/#run-post-link-scripts

 WARN These conda-packages will be overridden by pypi:
	libopencv
▪ preparing packages   [━━━━━━━━━━━━━━━━━━━━] 774/774
▪ installing           [━━━━━━━━━━━━━━━━━━━━] 774/774
barrett@farts ~/auto_ros2 (main)>
```

### Step 5: Build and run the code!

Now, build and run the code:

1. Enter the `pixi` environment by typing: `pixi shell`
2. Build the ROS 2 packages: `just build`
3. Register the new packages in ROS 2: `source ./SOURCE_SCRIPT.bash`
   - note: You MUST do this every time you rebuild, and every time you open a new terminal.

```bash
# enter the `pixi` environment:
barrett@farts ~/auto_ros2 (main)> pixi shell

# good! you should see a little `(auto_ros2)` marker to mark the environment,
# like this:
barrett@farts ~/auto_ros2 (main)>                                   # (auto_ros2)

# now, use the `just build` command (installed by pixi) to build the code!
#
# you DON'T need to run any of these huge commands! only run `just build`,
# like so:
barrett@farts ~/auto_ros2 (main)> just build                      # (auto_ros2)
Building the ROS 2 workspace...

colcon build --symlink-install --packages-select custom_interfaces --cmake-args -DPython_ROOT_DIR="$CONDA_PREFIX" -DPython_EXECUTABLE="$(python -c 'import sys; print(sys.executable)')"
Starting >>> custom_interfaces
Finished <<< custom_interfaces [9.07s]
Summary: 1 package finished [9.44s]

colcon build --symlink-install --cmake-args -DPython_ROOT_DIR="$CONDA_PREFIX" -DPython_EXECUTABLE="$(python -c 'import sys; print(sys.executable)')"
Starting >>> custom_interfaces_shim
Starting >>> aruco_node
Starting >>> custom_interfaces
Starting >>> drive_launcher
Starting >>> log_node
Starting >>> manual_control
Starting >>> navigator
Starting >>> see3cam
Starting >>> simulator
Starting >>> zed
Finished <<< drive_launcher [1.45s]
Finished <<< custom_interfaces [1.52s]
Finished <<< see3cam [2.02s]
Finished <<< log_node [2.22s]
Finished <<< manual_control [2.23s]
Finished <<< aruco_node [2.25s]
Finished <<< navigator [2.25s]
Finished <<< zed [2.29s]
Finished <<< simulator [2.82s]
Finished <<< custom_interfaces_shim [5.56s]
Starting >>> drive_tui
Starting >>> lights
Starting >>> maps_backend
Starting >>> sensors
Starting >>> wheels
Finished <<< drive_tui [2.82s]
Finished <<< lights [23.0s]
Finished <<< wheels [23.0s]
Finished <<< maps_backend [23.1s]
Finished <<< sensors [23.4s]

Summary: 15 packages finished [29.1s]
Build complete!

# great, the build's all done!
#
# source the new packages so ROS 2 can see them:
barrett@farts ~/auto_ros2 (main)> source SOURCE_SCRIPT.bash       # (auto_ros2)
You should be running this script using `. ./SOURCE_SCRIPT.bash`.
If you did not do so, please restart the script.

Sourcing ROS 2 environment files...
Done!

Setting Nav2-compatible ROS middleware implementation...
Done!

Turning on colorized output from ROS 2...
Done!

All source tasks are now complete!

# our code is now built, and ROS 2 now knows about it!
#
# let's finish up by running a simple node, like the Lights node:
barrett@farts ~/auto_ros2 (main)> ros2 run lights lights_node     # (auto_ros2)
[INFO] [1756748042.419725160] [lights_node]: The `lights_node` has been created.
[INFO] [1756748042.420550145] [lights_node]: Server created! Starting request watcher...
```

To close the `lights_node`, press `Ctrl` + `\`at the same time on your keyboard. (on macOS, you might hit `Cmd` + `\` instead).

### Conclusion

Just like that, you now have the code running on your computer!

If you have any questions, difficulties, or errors, ask in the Discord! You may also find solutions to common problems/errors in the [`docs/faq.md` file](docs/faq.md).

## Running the simulator

Let's run the simulator! We'll assume you've already completed the installation + download steps above.

### Step 1: Activate the `pixi` environment

Just like before, we need the `pixi` environment to use ROS 2.

1. Navigate to `~/auto_ros2` with: `cd ~/auto_ros2`
1. Activate the environment: `pixi shell`

```bash
barrett@farts ~> cd auto_ros2/
barrett@farts ~/auto_ros2 (main)> pixi shell

barrett@farts ~/auto_ros2 (main)>                                   # (auto_ros2)
```

### Step 2: Run the simulator

The simulator will automatically build the code, source the `SOURCE_SCRIPT`, then run itself.

So, all you have to do is one step:

1. Start the simulator: `just sim`

```bash
barrett@farts ~/auto_ros2 (main)> just sim                        # (auto_ros2)
Building the ROS 2 workspace...
Summary: 15 packages finished [3.19s]
  1 package had stderr output: navigator
Build complete!
You should be running this script using `. ./SOURCE_SCRIPT.bash`.
If you did not do so, please restart the script.

Sourcing ROS 2 environment files...
Done!

Setting Nav2-compatible ROS middleware implementation...
Done!

Turning on colorized output from ROS 2...
Done!

All source tasks are now complete!
Adding these paths to Gazebo's plugin path: /opt/ros/humble/lib
The simulation is about to begin...
[INFO] [launch]: All log files can be found below /home/barrett/.ros/log/2025-09-01-12-45-20-159782-farts-20001
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [parameter_bridge-1]: process started with pid [20018]
[INFO] [image_bridge-2]: process started with pid [20020]
[INFO] [ign gazebo-3]: process started with pid [20022]
[INFO] [simulator-4]: process started with pid [20024]
[INFO] [joint_state_publisher-5]: process started with pid [20027]
[INFO] [create-6]: process started with pid [20029]
[INFO] [navigator_node-7]: process started with pid [20031]
[INFO] [utm_conversion_node-8]: process started with pid [20033]
[INFO] [robot_state_publisher-9]: process started with pid [20035]
[INFO] [navsat_transform_node-10]: process started with pid [20037]
[INFO] [ekf_node-11]: process started with pid [20039]
[INFO] [ekf_node-12]: process started with pid [20041]
[INFO] [depthimage_to_laserscan_node-13]: process started with pid [20043]
[INFO] [slam_toolbox_start_node-14]: process started with pid [20045]
[INFO] [spawner-15]: process started with pid [20047]
[INFO] [navigation_bringup_node-16]: process started with pid [20052]
[INFO] [spawner-17]: process started with pid [20056]
[robot_state_publisher-9] [INFO] [1756748720.743009471] [robot_state_publisher]: got segment base_link
[robot_state_publisher-9] [INFO] [1756748720.743058111] [robot_state_publisher]: got segment camera_link
[robot_state_publisher-9] [INFO] [1756748720.743065781] [robot_state_publisher]: got segment chassis
[robot_state_publisher-9] [INFO] [1756748720.743071541] [robot_state_publisher]: got segment gps_link
[robot_state_publisher-9] [INFO] [1756748720.743076571] [robot_state_publisher]: got segment imu_link
[robot_state_publisher-9] [INFO] [1756748720.743081562] [robot_state_publisher]: got segment left_back_wheel
[robot_state_publisher-9] [INFO] [1756748720.743086662] [robot_state_publisher]: got segment left_front_wheel
[robot_state_publisher-9] [INFO] [1756748720.743091782] [robot_state_publisher]: got segment left_middle_wheel
[robot_state_publisher-9] [INFO] [1756748720.743097102] [robot_state_publisher]: got segment right_back_wheel
[robot_state_publisher-9] [INFO] [1756748720.743102192] [robot_state_publisher]: got segment right_front_wheel
[robot_state_publisher-9] [INFO] [1756748720.743107262] [robot_state_publisher]: got segment right_middle_wheel
[create-6] [INFO] [1756748720.747545192] [ros_gz_sim]: Requesting list of world names.
[simulator-4] 2025-09-01 12:45:20.902 | INFO     | simulator.main:main:68 - Starting simulator driver...
[simulator-4] 2025-09-01 12:45:20.946 | INFO     | simulator.main:main:72 - Simulator has been initialized.
[navigation_bringup_node-16] 2025-09-01 12:45:20.964 | INFO     | navigation_bringup_node.main:main:110 - Starting Navigation bringup...
[slam_toolbox_start_node-14] [INFO] [launch]: All log files can be found below /home/barrett/.ros/log/2025-09-01-12-45-21-021223-farts-20048
[slam_toolbox_start_node-14] [INFO] [launch]: Default logging verbosity is set to INFO
[navigation_bringup_node-16] [INFO] [1756748721.045819747] [navigation_bringup_node]: Performing navigation stack bringup...
[utm_conversion_node-8] 2025-09-01 12:45:21.091 | INFO     | utm_conversion_node.main:main:103 - Starting UTM conversion Node...
[slam_toolbox_start_node-14] [INFO] [static_transform_publisher-1]: process started with pid [20334]
[slam_toolbox_start_node-14] [INFO] [async_slam_toolbox_node-2]: process started with pid [20336]
[slam_toolbox_start_node-14] [static_transform_publisher-1] [WARN] [1756748721.098760754] []: Old-style arguments are deprecated; see --help for new-style arguments
[slam_toolbox_start_node-14] [static_transform_publisher-1] [INFO] [1756748721.102588770] [tf2_publish_camera_depth_frame_link_node]: Spinning until stopped - publishing transform
[slam_toolbox_start_node-14] [static_transform_publisher-1] translation: ('0.000000', '0.000000', '0.000000')
[slam_toolbox_start_node-14] [static_transform_publisher-1] rotation: ('0.000000', '0.000000', '0.000000', '1.000000')
[slam_toolbox_start_node-14] [static_transform_publisher-1] from 'camera_link' to 'camera_depth_frame`
[slam_toolbox_start_node-14] [async_slam_toolbox_node-2] [INFO] [1756748721.110094031] [slam_toolbox]: Node using stack size 40000000

# ... and so on.
#
# in any case, you should now see the Gazebo simulator on your screen!
#
# watch the Rover complete its goals, and feel free to change the code to make
# it do something else! :)
```

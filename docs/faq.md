# FAQ

Some issues we've experienced and potential solutions.

## ROS 2 randomly blew up!

You were just screwing around and ROS 2 blew up in your face! Maybe it won't even talk to the DDS at this point?

This can happen if you `killall python3`, which messes with ROS 2:

```python3
(autonomous) barrett@sls2 ~/D/p/R/auto_ros2 (main)> ros2 topic list
Traceback (most recent call last):
  File "/opt/ros/humble/bin/ros2", line 33, in <module>
    sys.exit(load_entry_point('ros2cli==0.18.11', 'console_scripts', 'ros2')())
  File "/opt/ros/humble/lib/python3.10/site-packages/ros2cli/cli.py", line 91, in main
    rc = extension.main(parser=parser, args=args)
  File "/opt/ros/humble/lib/python3.10/site-packages/ros2topic/command/topic.py", line 41, in main
    return extension.main(args=args)
  File "/opt/ros/humble/lib/python3.10/site-packages/ros2topic/verb/list.py", line 56, in main
    topic_names_and_types = get_topic_names_and_types(
  File "/opt/ros/humble/lib/python3.10/site-packages/ros2topic/api/__init__.py", line 41, in get_topic_names_and_types
    topic_names_and_types = node.get_topic_names_and_types()
  File "/usr/lib/python3.10/xmlrpc/client.py", line 1122, in __call__
    return self.__send(self.__name, args)
  File "/usr/lib/python3.10/xmlrpc/client.py", line 1464, in __request
    response = self.__transport.request(
  File "/usr/lib/python3.10/xmlrpc/client.py", line 1166, in request
    return self.single_request(host, handler, request_body, verbose)
  File "/usr/lib/python3.10/xmlrpc/client.py", line 1182, in single_request
    return self.parse_response(resp)
  File "/usr/lib/python3.10/xmlrpc/client.py", line 1354, in parse_response
    return u.close()
  File "/usr/lib/python3.10/xmlrpc/client.py", line 668, in close
    raise Fault(**self._stack[0])
xmlrpc.client.Fault: <Fault 1: "<class 'RuntimeError'>:!rclpy.ok()">
(autonomous) barrett@sls2 ~/D/p/R/auto_ros2 (main) [1]> ros2 daemon stop
The daemon has been stopped
(autonomous) barrett@sls2 ~/D/p/R/auto_ros2 (main)> ros2 daemon start
The daemon has been started
(autonomous) barrett@sls2 ~/D/p/R/auto_ros2 (main)> ros2 topic list
/control/wheels
```

To fix this, restart its daemon: `ros2 daemon stop && ros2 daemon start`

## After a clean build, it's saying stuff has no metadata!

If you do a clean build, but only use `colcon build` (as opposed to `colcon build --symlink-install`), you're going to get this kind of error:

```
importlib_metadata.PackageNotFoundError: No package metadata was found for ros2cli
```

Also, make sure to refresh your environment afterwards by re-sourcing the `local_install.bash` file. `. ./SOURCE_SCRIPT.fish` works well for this! :)

## After a clean build, `ros2 run` says "no executable found"!

```fish
(autonomous) barrett@sls2 ~/D/p/R/auto_ros2 (main)> rm -rfd install/ build/ log/
(autonomous) barrett@sls2 ~/D/p/R/auto_ros2 (main)> colcon build --packages-select navigator_node
Starting >>> navigator_node
Finished <<< navigator_node [0.74s]

Summary: 1 package finished [0.94s]
(autonomous) barrett@sls2 ~/D/p/R/auto_ros2 (main)> ros2 run navigator_node navigator_node
No executable found
```

Like above, you need to use `colcon build --symlink-install`.

## `ros2 launch` gives "libexec directory does not exist"

```fish
(autonomous) barrett@sls2 ~/D/p/R/auto_ros2 (main)> ros2 launch simulator sim.launch.py
[INFO] [launch]: All log files can be found below /home/barrett/.ros/log/2025-02-25-01-25-56-666083-sls2-52704
[INFO] [launch]: Default logging verbosity is set to INFO
[ERROR] [launch]: Caught exception in launch (see debug for traceback): package 'navigator_node' found at '/home/barrett/Documents/projects/Rover/auto_ros2/install/navigator_node', but libexec directory '/home/barrett/Documents/projects/Rover/auto_ros2/install/navigator_node/lib/navigator_node' does not exist
(autonomous) barrett@sls2 ~/D/p/R/auto_ros2 (main) [1]>
```

Use `--symlink-install`!

## At runtime, I got a `TypeError`... Unhashable type?

```python3
[simulator-4] 2025-02-25 01:29:30.844 | INFO     | simulator.main:main:127 - starting simulator driver...
[simulator-4] Traceback (most recent call last):
[simulator-4]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py", line 224, in spin
[simulator-4]     executor.add_node(node)
[simulator-4]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 254, in add_node
[simulator-4]     if node not in self._nodes:
[simulator-4] TypeError: unhashable type: 'SoroBridge'
[simulator-4]
[simulator-4] During handling of the above exception, another exception occurred:
[simulator-4]
[simulator-4] Traceback (most recent call last):
[simulator-4]   File "/home/barrett/Documents/projects/Rover/auto_ros2/install/simulator/lib/simulator/simulator", line 33, in <module>
[simulator-4]     sys.exit(load_entry_point('simulator', 'console_scripts', 'simulator')())
[simulator-4]   File "/home/barrett/Documents/projects/Rover/auto_ros2/build/simulator/simulator/main.py", line 139, in main
[simulator-4]     rclpy.spin(bridge_node)
[simulator-4]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py", line 228, in spin
[simulator-4]     executor.remove_node(node)
[simulator-4]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 270, in remove_node
[simulator-4]     self._nodes.remove(node)
[simulator-4] TypeError: unhashable type: 'SoroBridge'
```

Implement Hash for your type:

```python3
from dataclasses import dataclass
from typing_extensions import override

from rclpy.node import Node

@dataclass
class MyClass(Node):
    def __init__():
        super().__init__("my_class")

    @override
    def __hash__(self) -> int:
        return super().__hash__()
```

## Unknown command: just

We use [`just`](https://just.systems/man/en/) to build the code! However, it's included in the virtual environment - not on your system.

To use `just`, run the `uv sync` command first, then source the virtual environment. The `SOURCE_SCRIPT` (either for Fish or Bash shell) will work great for that!

```fish
soro@71b7d4dde026 ~/auto_ros2 (feat/docker_better_compat)> just sim
fish: Unknown command: just
soro@71b7d4dde026 ~/auto_ros2 (feat/docker_better_compat)> uv sync
Resolved 90 packages in 7ms
Audited 68 packages in 0.02ms
soro@71b7d4dde026 ~/auto_ros2 (feat/docker_better_compat)> . ./SOURCE_SCRIPT.fish
You should be running this script using `. ./SOURCE_SCRIPT.fish`.
If you didn't do so, please restart the script.

Sourcing virtual environment...
Done!
Sourcing ROS 2 environment files...
Done!
(autonomous) soro@71b7d4dde026 ~/auto_ros2 (feat/docker_better_compat)> just sim
Building the ROS 2 workspace...
colcon build --symlink-install
Starting >>> aruco_node
```

## I can't run the simulator!

If you're coming from a clean repo, with no build files, then you try to run the simulator, it'll complain loudly:

```fish
soro@71b7d4dde026 ~/auto_ros2 (feat/docker_better_compat)> just sim
fish: Unknown command: just
soro@71b7d4dde026 ~/auto_ros2 (feat/docker_better_compat) [127]> uv sync
Resolved 90 packages in 7ms
Audited 68 packages in 0.02ms
soro@71b7d4dde026 ~/auto_ros2 (feat/docker_better_compat)> . ./SOURCE_SCRIPT.fish
You should be running this script using `. ./SOURCE_SCRIPT.fish`.
If you didn't do so, please restart the script.

Sourcing virtual environment...
Done!
Sourcing ROS 2 environment files...
bass: line 1: install/local_setup.bash: No such file or directory
Done!
(autonomous) soro@71b7d4dde026 ~/auto_ros2 (feat/docker_better_compat)> just sim
Building the ROS 2 workspace...
colcon build --symlink-install
Starting >>> aruco_node
# (...)
Finished <<< sensors_node [7.14s]

Sourcing virtual environment...
gazebo: no process found
ruby: no process found
gzserver: no process found
gzclient: no process found
ign: no process found
Done!

Sourcing ROS 2 environment files...

The simulation is about to begin...
ros2 launch simulator sim.launch.py
Done!
Package 'simulator' not found: "package 'simulator' not found, searching: ['/opt/ros/humble']"
error: Recipe `sim` failed on line 13 with exit code 1
(autonomous) soro@71b7d4dde026 ~/auto_ros2 (feat/docker_better_compat) [1]>
```

You'll need to source again after [building a new ROS 2 package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html).

## After building on the Jetson, a lot of packages say they have "stderr output." What does that even mean?

**tl;dr: ignore the errors!** :D

When running a build, you probably got some logs like this:

```
Finished <<< sensors [9.61s]
[Processing: zed_components]
[Processing: zed_components]
[Processing: zed_components]
[Processing: zed_components]
[Processing: zed_components]
--- stderr: zed_components
gmake: Warning: File 'Makefile' has modification time 1049 s in the future
gmake[2]: Warning: File 'CMakeFiles/zed_camera_component.dir/flags.make' has modification time 3908 s in the future
gmake[2]: Warning: File 'CMakeFiles/zed_camera_one_component.dir/flags.make' has modification time 3908 s in the future
gmake[2]: warning:  Clock skew detected.  Your build may be incomplete.
gmake[2]: Warning: File 'CMakeFiles/zed_camera_component.dir/flags.make' has modification time 3908 s in the future
gmake[2]: warning:  Clock skew detected.  Your build may be incomplete.
gmake[2]: Warning: File 'CMakeFiles/zed_camera_one_component.dir/flags.make' has modification time 3908 s in the future
gmake[2]: warning:  Clock skew detected.  Your build may be incomplete.
gmake[2]: warning:  Clock skew detected.  Your build may be incomplete.
gmake: warning:  Clock skew detected.  Your build may be incomplete.
---
Finished <<< zed_components [3min 28s]
Starting >>> zed_wrapper
--- stderr: zed_wrapper
gmake: Warning: File 'Makefile' has modification time 1050 s in the future
gmake: warning:  Clock skew detected.  Your build may be incomplete.
---
Finished <<< zed_wrapper [1.76s]
Starting >>> zed_ros2
--- stderr: zed_ros2
gmake: Warning: File 'Makefile' has modification time 1049 s in the future
gmake: warning:  Clock skew detected.  Your build may be incomplete.
---
Finished <<< zed_ros2 [0.75s]

Summary: 17 packages finished [3min 32s]
  9 packages had stderr output: custom_interfaces drive_launcher see3cam sensors simulator zed zed_components zed_ros2 zed_wrapper
```

The Jetson doesn't have a clock - it gets the current time from the internet. However, when no internet connection is available, it can't update the time, which makes it assume a time of `0_u64` - which is around January 1st, 1970.

Since a lot of files are uploaded to the Rover directly, or maintain their modification times from `git`, you'll often see these "file is from the future" errors when the Rover is building code while not connected to a WAN network.

**In other words, you can safely ignore the errors.**

## I'm running macOS, and I can't seem to build anything in the Conda environment!

Colcon might accidentally be building with the wrong Python version.

In my case, it tried to build with the `uv` Python, which doesn't ship the same C binding stuff. You'll never get the exact versions to line up on macOS due to system Python version contraints, so libraries found in the correct `pixi` Python won't count as "real" libraries. (on Windows, it might do the same with the Visual Studio Python)

Either way, you'll probably end up with some weird build error:

```fish
(autonomous) ~/D/p/R/auto_ros2 (main|✚8) $ colcon build --symlink-install --packages-skip manual_control                                                                                           (auto_ros2)
Starting >>> custom_interfaces_shim
Starting >>> aruco_node
Starting >>> custom_interfaces
Starting >>> drive_launcher
Starting >>> log_node
Starting >>> navigator
Starting >>> see3cam
Starting >>> simulator
Finished <<< log_node [2.10s]
Starting >>> zed
Finished <<< aruco_node [2.24s]
--- stderr: navigator
...
---
Finished <<< navigator [2.27s]
--- stderr: drive_launcher
...
---
Finished <<< see3cam [3.35s]
Finished <<< custom_interfaces_shim [3.70s]
Starting >>> drive_tui
Starting >>> lights
Starting >>> maps_backend
Starting >>> sensors
Starting >>> wheels
--- stderr: zed
...
---
Finished <<< zed [1.75s]
--- stderr: simulator
...
---
Finished <<< simulator [4.02s]
Finished <<< drive_tui [0.91s]
--- stderr: custom_interfaces
CMake Error at /Users/barrett/Documents/projects/Rover/auto_ros2/.pixi/envs/default/share/cmake-4.1/Modules/FindPackageHandleStandardArgs.cmake:227 (message):
  Could NOT find Python (missing: Python_EXECUTABLE Python_INCLUDE_DIRS
  Python_LIBRARIES Python_NumPy_INCLUDE_DIRS Interpreter Development NumPy
  Development.Module Development.Embed)
Call Stack (most recent call first):
  /Users/barrett/Documents/projects/Rover/auto_ros2/.pixi/envs/default/share/cmake-4.1/Modules/FindPackageHandleStandardArgs.cmake:591 (_FPHSA_FAILURE_MESSAGE)
  /Users/barrett/Documents/projects/Rover/auto_ros2/.pixi/envs/default/share/cmake-4.1/Modules/FindPython.cmake:727 (find_package_handle_standard_args)
  /Users/barrett/Documents/projects/Rover/auto_ros2/.pixi/envs/default/share/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake:34 (find_package)
  /Users/barrett/Documents/projects/Rover/auto_ros2/.pixi/envs/default/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /Users/barrett/Documents/projects/Rover/auto_ros2/.pixi/envs/default/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:28 (rosidl_generate_interfaces)


---
Failed   <<< custom_interfaces [10.5s, exited with code 1]
```

To fix this, you can avoid using the virtual environment and pulling in dependencies directly with Pixi's `PyPi` support.

However, to do so without using the macOS system Python, you'll need some special flags, like so: `colcon build --symlink-install --cmake-args -DPython_ROOT_DIR="$CONDA_PREFIX" -DPython_EXECUTABLE="$(python -c 'import sys; print(sys.executable)')"`.

This correctly builds everything:

```bash
bash-3.2$ colcon build --symlink-install --packages-skip manual_control --cmake-args -DPython_ROOT_DIR="$CONDA_PREFIX" -DPython_EXECUTABLE="$(python -c 'import sys; print(sys.executable)')"
Starting >>> custom_interfaces_shim
Starting >>> aruco_node
Starting >>> custom_interfaces
Starting >>> drive_launcher
Starting >>> log_node
Starting >>> navigator
Starting >>> see3cam
Starting >>> simulator
Finished <<< custom_interfaces_shim [0.57s]
Starting >>> zed
Finished <<< drive_launcher [0.79s]
Starting >>> drive_tui
Finished <<< see3cam [0.81s]
Starting >>> lights
Finished <<< zed [0.31s]
Starting >>> maps_backend
Finished <<< drive_tui [0.37s]
Starting >>> sensors
Finished <<< simulator [1.19s]
Starting >>> wheels
Finished <<< maps_backend [0.34s]
Finished <<< aruco_node [1.36s]
--- stderr: navigator
(meaningless warnings)
---
Finished <<< navigator [1.35s]
Finished <<< log_node [1.36s]
Finished <<< sensors [0.23s]
Finished <<< wheels [0.18s]
Finished <<< lights [1.35s]
--- stderr: custom_interfaces
(meaningless warnings)
---
Finished <<< custom_interfaces [2.43s]

Summary: 14 packages finished [2.61s]
  2 packages had stderr output: custom_interfaces navigator
bash-3.2$
```

## I can't seem to find a package when doing `ros2 run`! It doesn't even show up in `ros2 pkg list`!

Can't find a package?

```bash
bash-3.2$ ros2 pkg list | grep custom_interfaces
bash-3.2$
```

No worries - you've probably forgotten to _source_ the install script. Try this:

```bash
bash-3.2$ . install/local_setup.bash
bash-3.2$ ros2 pkg list | grep custom_interfaces
custom_interfaces       # <----- fixed! :D
bash-3.2$
```

## I'm getting a weird `conda` error when trying to run a node. It can't seem to find a dynamic library ("library not loaded")..?

Yeah - it seems that, on macOS, the ROS 2 source scripts may not work just right in Conda. For that reason, certain packages might not be found at runtime by `rcl` and friends:

```bash
bash-3.2$ ros2 run lights lights_node
dyld[32674]: Library not loaded: @rpath/libgeographic_msgs__rosidl_typesupport_c.dylib
  Referenced from: <6818C4EB-F418-3A4D-B1E4-585677365D8B> /Users/barrett/Documents/projects/Rover/auto_ros2/install/lights/lib/lights/lights_node
  Reason: tried: '/Users/barrett/Documents/projects/Rover/auto_ros2/install/custom_interfaces/lib/libgeographic_msgs__rosidl_typesupport_c.dylib' (no such file), '/Users/barrett/Documents/projects/Rover/auto_ros2/.pixi/envs/default/opt/rviz_ogre_vendor/lib/libgeographic_msgs__rosidl_typesupport_c.dylib' (no such file)
[ros2run]: Abort trap: 6
bash-3.2$
```

Pixi is probably trying to run the `zsh` script for another shell, which causes some packages to not be registered. You can check if that's the case with the following command:

```bash
bash-3.2$ echo $DYLD_LIBRARY_PATH
/Users/barrett/Documents/projects/Rover/auto_ros2/install/custom_interfaces/lib:/Users/barrett/Documents/projects/Rover/auto_ros2/.pixi/envs/default/opt/rviz_ogre_vendor/lib
bash-3.2$
```

Here's a better-formatted version:

- `/Users/barrett/Documents/projects/Rover/auto_ros2/install/custom_interfaces/lib`
- `/Users/barrett/Documents/projects/Rover/auto_ros2/.pixi/envs/default/opt/rviz_ogre_vendor/lib`

As you can see, `$CONDA_PREFIX/lib` (which, for me, evaluates to `/Users/barrett/Documents/projects/Rover/auto_ros2/.pixi/envs/default/lib`) isn't present in the list!

Try running this: `export DYLD_LIBRARY_PATH="$CONDA_PREFIX/lib:$DYLD_LIBRARY_PATH"`

That works for me, allowing me to run the `lights_node`, for example!

```bash
bash-3.2$ export DYLD_LIBRARY_PATH="$CONDA_PREFIX/lib:$DYLD_LIBRARY_PATH"
bash-3.2$ ros2 run lights lights_node
[INFO] [1755467673.395787951] [lights_node]: The `lights_node` has been created.
[INFO] [1755467673.396941477] [lights_node]: Server created! Starting request watcher...
```

> [!CAUTION]
> Running the above `export` command WILL cause builds to stop working. You'll need to run `unset DYLD_LIBRARY_PATH` to get it working again. (otherwise, macOS will use the wrong libraries for building)

## One of the Rust nodes explodes with a linker error when I try to build in the new Pixi environment. What's going on?

This happens when setting `$DYLD_LIBRARY_PATH` as shown above. It causes macOS to use the wrong libraries during linking, which XCode reeeeally doesn't like (lol):

```bash
~/D/p/R/auto_ros2 (main|✚8) $ colcon build --symlink-install --packages-skip manual_control --cmake-args -DPython_ROOT_DIR="$CONDA_PREFIX" -DPython_EXECUTABLE="$(python -c 'import sys; print(sys.executable)')"

...

--- stderr: lights
error: linking with `cc` failed: exit status: 1
  |
  = note:  "cc" "/var/folders/bd/_ch9ytrj07l39d478lyt79l00000gn/T/rustcRUWodD/symbols.o" "<98 object files omitted>" "/Users/barrett/Documents/projects/Rover/auto_ros2/build/lights/debug/deps/{libfeedback.rlib,libtracing-8b021a3922359756.rlib,libtracing_core-6d380ae9cd153ea2.rlib,libthiserror-cf437b33fe0624af.rlib,libcustom_interfaces-29543dedbd622365.rlib,libgeographic_msgs-859c17fe847f2e34.rlib,libunique_identifier_msgs-e9b300bad247ba10.rlib,libgeometry_msgs-ebd8eefd7973ae99.rlib,libstd_msgs-2df969f80f4f561d.rlib,libbuiltin_interfaces-e3c5501948944802.rlib,libsafe_drive-5ef6f5ee911f2e82.rlib,libsignal_hook-94ab49f0a0e2e53a.rlib,libcrossbeam_channel-f9c3e6eb69613ae6.rlib,libcrossbeam_utils-b0f6f16dad74b38e.rlib,libregex-2de19355f5d547cd.rlib,libregex_automata-d7cd47aa8bcf6a39.rlib,libaho_corasick-399951cffc4932fe.rlib,libregex_syntax-93102c852b1d4176.rlib,libnum_traits-e3b29dbad17cb9c6.rlib,libonce_cell-5d51a55a1a353586.rlib,libfutures-0b1a3d0006602cb7.rlib,libfutures_executor-c1ba222ad16213fd.rlib,libfutures_util-0d5cf13edd3e01a6.rlib,libmemchr-5f6df43509e8007e.rlib,libfutures_io-7840600bdc8a191e.rlib,libslab-f8add2a141d83c8d.rlib,libfutures_channel-594c2d552b63de6b.rlib,libfutures_sink-0b87a188f09b64e0.rlib,libfutures_task-ff04a30ae94c4831.rlib,libpin_utils-d60247b6540d1416.rlib,libfutures_core-e5892b2b95d7dcbe.rlib,libparking_lot-f431797de471071a.rlib,libparking_lot_core-3c7b3d369874b9e1.rlib,libcfg_if-6ae6bfdb17ac5f33.rlib,libsmallvec-7ba979a68c120aa3.rlib,liblock_api-a97bb167ef1d9c2d.rlib,libscopeguard-3a94de5a59526d7e.rlib,libpin_project-7314e391fde4d088.rlib,libtokio-9e21f93e3dfed765.rlib,libsignal_hook_registry-5555a8bebe180af7.rlib,libsocket2-b924fecdb9a011ec.rlib,libmio-2b5681e2891c946b.rlib,liblibc-3bfe410cc699aefd.rlib,libpin_project_lite-e2422973c5166238.rlib}.rlib" "<sysroot>/lib/rustlib/aarch64-apple-darwin/lib/{libstd-*,libpanic_unwind-*,libobject-*,libmemchr-*,libaddr2line-*,libgimli-*,librustc_demangle-*,libstd_detect-*,libhashbrown-*,librustc_std_workspace_alloc-*,libminiz_oxide-*,libadler2-*,libunwind-*,libcfg_if-*,liblibc-*,librustc_std_workspace_core-*,liballoc-*,libcore-*,libcompiler_builtins-*}.rlib" "-lcustom_interfaces__rosidl_typesupport_c" "-lcustom_interfaces__rosidl_generator_c" "-lgeographic_msgs__rosidl_typesupport_c" "-lgeographic_msgs__rosidl_generator_c" "-lunique_identifier_msgs__rosidl_typesupport_c" "-lunique_identifier_msgs__rosidl_generator_c" "-lgeometry_msgs__rosidl_typesupport_c" "-lgeometry_msgs__rosidl_generator_c" "-lstd_msgs__rosidl_typesupport_c" "-lstd_msgs__rosidl_generator_c" "-lbuiltin_interfaces__rosidl_typesupport_c" "-lbuiltin_interfaces__rosidl_generator_c" "-lrcl" "-lrcl_action" "-lrcutils" "-lrmw" "-lrosidl_runtime_c" "-lactionlib_msgs__rosidl_typesupport_c" "-lactionlib_msgs__rosidl_generator_c" "-lbuiltin_interfaces__rosidl_typesupport_c" "-lbuiltin_interfaces__rosidl_generator_c" "-ldiagnostic_msgs__rosidl_typesupport_c" "-ldiagnostic_msgs__rosidl_generator_c" "-lgeometry_msgs__rosidl_typesupport_c" "-lgeometry_msgs__rosidl_generator_c" "-lnav_msgs__rosidl_typesupport_c" "-lnav_msgs__rosidl_generator_c" "-lsensor_msgs__rosidl_typesupport_c" "-lsensor_msgs__rosidl_generator_c" "-lshape_msgs__rosidl_typesupport_c" "-lshape_msgs__rosidl_generator_c" "-lstd_msgs__rosidl_typesupport_c" "-lstd_msgs__rosidl_generator_c" "-lstd_srvs__rosidl_typesupport_c" "-lstd_srvs__rosidl_generator_c" "-lstereo_msgs__rosidl_typesupport_c" "-lstereo_msgs__rosidl_generator_c" "-ltrajectory_msgs__rosidl_typesupport_c" "-ltrajectory_msgs__rosidl_generator_c" "-lunique_identifier_msgs__rosidl_typesupport_c" "-lunique_identifier_msgs__rosidl_generator_c" "-lvisualization_msgs__rosidl_typesupport_c" "-lvisualization_msgs__rosidl_generator_c" "-lrcl_interfaces__rosidl_typesupport_c" "-lrcl_interfaces__rosidl_generator_c" "-laction_msgs__rosidl_typesupport_c" "-laction_msgs__rosidl_generator_c" "-liconv" "-lSystem" "-lc" "-lm" "-arch" "arm64" "-mmacosx-version-min=11.0.0" "-L" "/Users/barrett/Documents/projects/Rover/auto_ros2/install/custom_interfaces_shim/lib" "-L" "<sysroot>/lib" "-L" "/Users/barrett/Documents/projects/Rover/auto_ros2/src/custom_interfaces/bindings/custom_interfaces/../../../../install/custom_interfaces/lib" "-L" "/Users/barrett/Documents/projects/Rover/auto_ros2/install/custom_interfaces_shim/lib" "-L" "<sysroot>/lib" "-L" "/Users/barrett/Documents/projects/Rover/auto_ros2/install/custom_interfaces_shim/lib" "-L" "<sysroot>/lib" "-L" "/Users/barrett/Documents/projects/Rover/auto_ros2/install/custom_interfaces_shim/lib" "-L" "<sysroot>/lib" "-L" "/Users/barrett/Documents/projects/Rover/auto_ros2/install/custom_interfaces_shim/lib" "-L" "<sysroot>/lib" "-L" "/Users/barrett/Documents/projects/Rover/auto_ros2/install/custom_interfaces_shim/lib" "-L" "<sysroot>/lib" "-L" "/Users/barrett/Documents/projects/Rover/auto_ros2/install/custom_interfaces_shim/lib" "-L" "<sysroot>/lib" "-o" "/Users/barrett/Documents/projects/Rover/auto_ros2/build/lights/debug/deps/lights_node-1db2c44ccab8761d" "-Wl,-dead_strip" "-nodefaultlibs"
  = note: some arguments are omitted. use `--verbose` to show all linker arguments
  = note: ld: warning: ignoring duplicate libraries: '-lbuiltin_interfaces__rosidl_generator_c', '-lbuiltin_interfaces__rosidl_typesupport_c', '-lgeometry_msgs__rosidl_generator_c', '-lgeometry_msgs__rosidl_typesupport_c', '-lstd_msgs__rosidl_generator_c', '-lstd_msgs__rosidl_typesupport_c', '-lunique_identifier_msgs__rosidl_generator_c', '-lunique_identifier_msgs__rosidl_typesupport_c'
          ld: warning: search path '/Users/barrett/Documents/projects/Rover/auto_ros2/src/custom_interfaces/bindings/custom_interfaces/../../../../install/custom_interfaces/lib' not found
          ld: library 'custom_interfaces__rosidl_typesupport_c' not found
          clang: error: linker command failed with exit code 1 (use -v to see invocation)


error: could not compile `lights` (bin "lights_node") due to 1 previous error
---
Failed   <<< lights [24.2s, exited with code 1]
Aborted  <<< wheels [24.5s]
Aborted  <<< sensors [24.7s]
Aborted  <<< maps_backend [24.9s]
Aborted  <<< custom_interfaces [44.9s]

Summary: 9 packages finished [45.8s]
  1 package failed: lights
  4 packages aborted: custom_interfaces maps_backend sensors wheels
  7 packages had stderr output: custom_interfaces drive_launcher lights navigator see3cam simulator zed
~/D/p/R/auto_ros2 (main|✚8) $
```

The error may also look like this:

```fish
Starting >>> custom_interfaces_shim
Starting >>> aruco_node
Starting >>> custom_interfaces
Starting >>> drive_launcher
Starting >>> log_node
Starting >>> navigator
Starting >>> see3cam
Starting >>> simulator
--- stderr: custom_interfaces_shim
error: linking with `cc` failed: exit status: 1
  |
  = note:  "cc" "/var/folders/bd/_ch9ytrj07l39d478lyt79l00000gn/T/rustcDwbCc2/symbols.o" "<2 object files omitted>" "/Users/barrett/Documents/projects/Rover/auto_ros2/build/custom_interfaces_shim/debug/deps/{libautocfg-e81b1139685d3438.rlib}.rlib" "<sysroot>/lib/rustlib/aarch64-apple-darwin/lib/{libstd-*,libpanic_unwind-*,libobject-*,libmemchr-*,libaddr2line-*,libgimli-*,librustc_demangle-*,libstd_detect-*,libhashbrown-*,librustc_std_workspace_alloc-*,libminiz_oxide-*,libadler2-*,libunwind-*,libcfg_if-*,liblibc-*,librustc_std_workspace_core-*,liballoc-*,libcore-*,libcompiler_builtins-*}.rlib" "-lSystem" "-lc" "-lm" "-arch" "arm64" "-mmacosx-version-min=11.0.0" "-o" "/Users/barrett/Documents/projects/Rover/auto_ros2/build/custom_interfaces_shim/debug/build/num-traits-20de173a03953515/build_script_build-20de173a03953515" "-Wl,-dead_strip" "-nodefaultlibs"
  = note: some arguments are omitted. use `--verbose` to show all linker arguments
  = note: dyld[58746]: Symbol not found: __ZNK4tapi2v119LinkerInterfaceFile28getPlatformsAndMinDeploymentEv
            Referenced from: <565CE761-C1EB-37F5-9738-E1BCF6F2EC75> /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/ld
            Expected in:     <15C501C6-0EF4-3E32-9C14-04EC4CD23D35> /Users/barrett/Documents/projects/Rover/auto_ros2/.pixi/envs/default/lib/libtapi.dylib
          clang: error: unable to execute command: Abort trap: 6
          clang: error: linker command failed due to signal (use -v to see invocation)


error: linking with `cc` failed: exit status: 1
  |
  = note:  "cc" "/var/folders/bd/_ch9ytrj07l39d478lyt79l00000gn/T/rustcBXf5w2/symbols.o" "<2 object files omitted>" "<sysroot>/lib/rustlib/aarch64-apple-darwin/lib/{libstd-*,libpanic_unwind-*,libobject-*,libmemchr-*,libaddr2line-*,libgimli-*,librustc_demangle-*,libstd_detect-*,libhashbrown-*,librustc_std_workspace_alloc-*,libminiz_oxide-*,libadler2-*,libunwind-*,libcfg_if-*,liblibc-*,librustc_std_workspace_core-*,liballoc-*,libcore-*,libcompiler_builtins-*}.rlib" "-lSystem" "-lc" "-lm" "-arch" "arm64" "-mmacosx-version-min=11.0.0" "-o" "/Users/barrett/Documents/projects/Rover/auto_ros2/build/custom_interfaces_shim/debug/build/lexical-core-b3a025ed89c7d5b5/build_script_build-b3a025ed89c7d5b5" "-Wl,-dead_strip" "-nodefaultlibs"
  = note: some arguments are omitted. use `--verbose` to show all linker arguments
  = note: dyld[58747]: Symbol not found: __ZNK4tapi2v119LinkerInterfaceFile28getPlatformsAndMinDeploymentEv
            Referenced from: <565CE761-C1EB-37F5-9738-E1BCF6F2EC75> /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/ld
            Expected in:     <15C501C6-0EF4-3E32-9C14-04EC4CD23D35> /Users/barrett/Documents/projects/Rover/auto_ros2/.pixi/envs/default/lib/libtapi.dylib
          clang: error: unable to execute command: Abort trap: 6
          clang: error: linker command failed due to signal (use -v to see invocation)


error: linking with `cc` failed: exit status: 1
  |
  = note:  "cc" "/var/folders/bd/_ch9ytrj07l39d478lyt79l00000gn/T/rustcZYW0fB/symbols.o" "<2 object files omitted>" "/Users/barrett/Documents/projects/Rover/auto_ros2/build/custom_interfaces_shim/debug/deps/{libversion_check-bbf5ab18e1d12d3a.rlib}.rlib" "<sysroot>/lib/rustlib/aarch64-apple-darwin/lib/{libstd-*,libpanic_unwind-*,libobject-*,libmemchr-*,libaddr2line-*,libgimli-*,librustc_demangle-*,libstd_detect-*,libhashbrown-*,librustc_std_workspace_alloc-*,libminiz_oxide-*,libadler2-*,libunwind-*,libcfg_if-*,liblibc-*,librustc_std_workspace_core-*,liballoc-*,libcore-*,libcompiler_builtins-*}.rlib" "-lSystem" "-lc" "-lm" "-arch" "arm64" "-mmacosx-version-min=11.0.0" "-o" "/Users/barrett/Documents/projects/Rover/auto_ros2/build/custom_interfaces_shim/debug/build/nom-5bd9852eec4e5707/build_script_build-5bd9852eec4e5707" "-Wl,-dead_strip" "-nodefaultlibs"
  = note: some arguments are omitted. use `--verbose` to show all linker arguments
  = note: dyld[58745]: Symbol not found: __ZNK4tapi2v119LinkerInterfaceFile28getPlatformsAndMinDeploymentEv
            Referenced from: <565CE761-C1EB-37F5-9738-E1BCF6F2EC75> /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/ld
            Expected in:     <15C501C6-0EF4-3E32-9C14-04EC4CD23D35> /Users/barrett/Documents/projects/Rover/auto_ros2/.pixi/envs/default/lib/libtapi.dylib
          clang: error: unable to execute command: Abort trap: 6
          clang: error: linker command failed due to signal (use -v to see invocation)


error: could not compile `num-traits` (build script) due to 1 previous error
error: could not compile `lexical-core` (build script) due to 1 previous error
error: could not compile `nom` (build script) due to 1 previous error
error: linking with `cc` failed: exit status: 1
  |
  = note:  "cc" "/var/folders/bd/_ch9ytrj07l39d478lyt79l00000gn/T/rustcwg8vxU/symbols.o" "<4 object files omitted>" "<sysroot>/lib/rustlib/aarch64-apple-darwin/lib/{libstd-*,libpanic_unwind-*,libobject-*,libmemchr-*,libaddr2line-*,libgimli-*,librustc_demangle-*,libstd_detect-*,libhashbrown-*,librustc_std_workspace_alloc-*,libminiz_oxide-*,libadler2-*,libunwind-*,libcfg_if-*,liblibc-*,librustc_std_workspace_core-*,liballoc-*,libcore-*,libcompiler_builtins-*}.rlib" "-lSystem" "-lc" "-lm" "-arch" "arm64" "-mmacosx-version-min=11.0.0" "-o" "/Users/barrett/Documents/projects/Rover/auto_ros2/build/custom_interfaces_shim/debug/build/typenum-e60d33863fd5ed94/build_script_build-e60d33863fd5ed94" "-Wl,-dead_strip" "-nodefaultlibs"
  = note: some arguments are omitted. use `--verbose` to show all linker arguments
  = note: dyld[58758]: Symbol not found: __ZNK4tapi2v119LinkerInterfaceFile28getPlatformsAndMinDeploymentEv
            Referenced from: <565CE761-C1EB-37F5-9738-E1BCF6F2EC75> /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/ld
            Expected in:     <15C501C6-0EF4-3E32-9C14-04EC4CD23D35> /Users/barrett/Documents/projects/Rover/auto_ros2/.pixi/envs/default/lib/libtapi.dylib
          clang: error: unable to execute command: Abort trap: 6
          clang: error: linker command failed due to signal (use -v to see invocation)


error: could not compile `typenum` (build script) due to 1 previous error
---
Failed   <<< custom_interfaces_shim [0.76s, exited with code 1]
```

You can fix the build by unsetting that temporarily, as it's only required at runtime (not build time):

```bash
# first, unset the variable
bash-3.2$ unset DYLD_LIBRARY_PATH

# then, run the build again!
bash-3.2$ colcon build --symlink-install --packages-skip manual_control --cmake-args -DPython_ROOT_DIR="$CONDA_PREFIX" -DPython_EXECUTABLE="$(python -c 'import sys; print(sys.executable)')"
Starting >>> custom_interfaces_shim
Starting >>> aruco_node
Starting >>> custom_interfaces
Starting >>> drive_launcher
Starting >>> log_node
Starting >>> navigator
Starting >>> see3cam
Starting >>> simulator
Finished <<< custom_interfaces_shim [0.57s]
Starting >>> zed
Finished <<< drive_launcher [0.79s]
Starting >>> drive_tui
Finished <<< see3cam [0.81s]
Starting >>> lights
Finished <<< zed [0.31s]
Starting >>> maps_backend
Finished <<< drive_tui [0.37s]
Starting >>> sensors
Finished <<< simulator [1.19s]
Starting >>> wheels
Finished <<< maps_backend [0.34s]
Finished <<< aruco_node [1.36s]
--- stderr: navigator
(meaningless warnings)
---
Finished <<< navigator [1.35s]
Finished <<< log_node [1.36s]
Finished <<< sensors [0.23s]
Finished <<< wheels [0.18s]
Finished <<< lights [1.35s]
--- stderr: custom_interfaces
(meaningless warnings)
---
Finished <<< custom_interfaces [2.43s]

Summary: 14 packages finished [2.61s]
  2 packages had stderr output: custom_interfaces navigator
bash-3.2$
```

### Oh, I'm doing a clean build (or haven't built the code before), and it's still happening!

Yeah, sometimes you'll get a **GIANT LINKER ERROR** like this:

```fish
~/D/p/R/auto_ros2 (refactor/pixi|✚5) $ pixi s

~/D/p/R/auto_ros2 (refactor/pixi|✚5) $ just clean                                        (auto_ros2)
Attempting to clean build files...
rm -rfd install/ install/ log/ .cargo/
rm -rfd build/
 rm -rfd .venv/
Build files have been removed!

~/D/p/R/auto_ros2 (refactor/pixi|✚5) $ colcon build --symlink-install --cmake-args -DPython_ROOT_DIR="$CONDA_PREFIX" -DPython_EXECUTABLE="$(python -c 'import sys; print(sys.executable)')"
Starting >>> custom_interfaces_shim
Starting >>> aruco_node
Starting >>> custom_interfaces
Starting >>> drive_launcher
Starting >>> log_node
Starting >>> manual_control
Starting >>> navigator
Starting >>> see3cam
Finished <<< manual_control [2.31s]
Starting >>> simulator
Finished <<< aruco_node [2.35s]
Starting >>> zed
Finished <<< log_node [2.39s]
Finished <<< navigator [2.39s]
Finished <<< drive_launcher [2.87s]
Finished <<< see3cam [3.35s]
Finished <<< custom_interfaces_shim [3.56s]
Starting >>> drive_tui
Starting >>> lights
Starting >>> maps_backend
Starting >>> sensors
Starting >>> wheels
Finished <<< drive_tui [0.76s]
Finished <<< zed [3.08s]
Finished <<< simulator [5.70s]
--- stderr: lights
error: linking with `cc` failed: exit status: 1
  |
  = note:  "cc" "/var/folders/bd/_ch9ytrj07l39d478lyt79l00000gn/T/rustcuyidcI/symbols.o" "<98 object files omitted>" "/Users/barrett/Documents/projects/Rover/auto_ros2/build/lights/debug/deps/{libfeedback.rlib,libtracing-8b021a3922359756.rlib,libtracing_core-6d380ae9cd153ea2.rlib,libthiserror-cf437b33fe0624af.rlib,libcustom_interfaces-29543dedbd622365.rlib,libgeographic_msgs-859c17fe847f2e34.rlib,libunique_identifier_msgs-e9b300bad247ba10.rlib,libgeometry_msgs-ebd8eefd7973ae99.rlib,libstd_msgs-2df969f80f4f561d.rlib,libbuiltin_interfaces-e3c5501948944802.rlib,libsafe_drive-5ef6f5ee911f2e82.rlib,libsignal_hook-94ab49f0a0e2e53a.rlib,libcrossbeam_channel-f9c3e6eb69613ae6.rlib,libcrossbeam_utils-b0f6f16dad74b38e.rlib,libregex-2de19355f5d547cd.rlib,libregex_automata-d7cd47aa8bcf6a39.rlib,libaho_corasick-399951cffc4932fe.rlib,libregex_syntax-93102c852b1d4176.rlib,libnum_traits-e3b29dbad17cb9c6.rlib,libonce_cell-5d51a55a1a353586.rlib,libfutures-0b1a3d0006602cb7.rlib,libfutures_executor-c1ba222ad16213fd.rlib,libfutures_util-0d5cf13edd3e01a6.rlib,libmemchr-5f6df43509e8007e.rlib,libfutures_io-7840600bdc8a191e.rlib,libslab-f8add2a141d83c8d.rlib,libfutures_channel-594c2d552b63de6b.rlib,libfutures_sink-0b87a188f09b64e0.rlib,libfutures_task-ff04a30ae94c4831.rlib,libpin_utils-d60247b6540d1416.rlib,libfutures_core-e5892b2b95d7dcbe.rlib,libparking_lot-f431797de471071a.rlib,libparking_lot_core-3c7b3d369874b9e1.rlib,libcfg_if-6ae6bfdb17ac5f33.rlib,libsmallvec-7ba979a68c120aa3.rlib,liblock_api-a97bb167ef1d9c2d.rlib,libscopeguard-3a94de5a59526d7e.rlib,libpin_project-7314e391fde4d088.rlib,libtokio-9e21f93e3dfed765.rlib,libsignal_hook_registry-5555a8bebe180af7.rlib,libsocket2-b924fecdb9a011ec.rlib,libmio-2b5681e2891c946b.rlib,liblibc-3bfe410cc699aefd.rlib,libpin_project_lite-e2422973c5166238.rlib}.rlib" "<sysroot>/lib/rustlib/aarch64-apple-darwin/lib/{libstd-*,libpanic_unwind-*,libobject-*,libmemchr-*,libaddr2line-*,libgimli-*,librustc_demangle-*,libstd_detect-*,libhashbrown-*,librustc_std_workspace_alloc-*,libminiz_oxide-*,libadler2-*,libunwind-*,libcfg_if-*,liblibc-*,librustc_std_workspace_core-*,liballoc-*,libcore-*,libcompiler_builtins-*}.rlib" "-lcustom_interfaces__rosidl_typesupport_c" "-lcustom_interfaces__rosidl_generator_c" "-lgeographic_msgs__rosidl_typesupport_c" "-lgeographic_msgs__rosidl_generator_c" "-lunique_identifier_msgs__rosidl_typesupport_c" "-lunique_identifier_msgs__rosidl_generator_c" "-lgeometry_msgs__rosidl_typesupport_c" "-lgeometry_msgs__rosidl_generator_c" "-lstd_msgs__rosidl_typesupport_c" "-lstd_msgs__rosidl_generator_c" "-lbuiltin_interfaces__rosidl_typesupport_c" "-lbuiltin_interfaces__rosidl_generator_c" "-lrcl" "-lrcl_action" "-lrcutils" "-lrmw" "-lrosidl_runtime_c" "-lactionlib_msgs__rosidl_typesupport_c" "-lactionlib_msgs__rosidl_generator_c" "-lbuiltin_interfaces__rosidl_typesupport_c" "-lbuiltin_interfaces__rosidl_generator_c" "-ldiagnostic_msgs__rosidl_typesupport_c" "-ldiagnostic_msgs__rosidl_generator_c" "-lgeometry_msgs__rosidl_typesupport_c" "-lgeometry_msgs__rosidl_generator_c" "-lnav_msgs__rosidl_typesupport_c" "-lnav_msgs__rosidl_generator_c" "-lsensor_msgs__rosidl_typesupport_c" "-lsensor_msgs__rosidl_generator_c" "-lshape_msgs__rosidl_typesupport_c" "-lshape_msgs__rosidl_generator_c" "-lstd_msgs__rosidl_typesupport_c" "-lstd_msgs__rosidl_generator_c" "-lstd_srvs__rosidl_typesupport_c" "-lstd_srvs__rosidl_generator_c" "-lstereo_msgs__rosidl_typesupport_c" "-lstereo_msgs__rosidl_generator_c" "-ltrajectory_msgs__rosidl_typesupport_c" "-ltrajectory_msgs__rosidl_generator_c" "-lunique_identifier_msgs__rosidl_typesupport_c" "-lunique_identifier_msgs__rosidl_generator_c" "-lvisualization_msgs__rosidl_typesupport_c" "-lvisualization_msgs__rosidl_generator_c" "-lrcl_interfaces__rosidl_typesupport_c" "-lrcl_interfaces__rosidl_generator_c" "-laction_msgs__rosidl_typesupport_c" "-laction_msgs__rosidl_generator_c" "-liconv" "-lSystem" "-lc" "-lm" "-arch" "arm64" "-mmacosx-version-min=11.0.0" "-L" "/Users/barrett/Documents/projects/Rover/auto_ros2/install/custom_interfaces_shim/lib" "-L" "<sysroot>/lib" "-L" "/Users/barrett/Documents/projects/Rover/auto_ros2/src/custom_interfaces/bindings/custom_interfaces/../../../../install/custom_interfaces/lib" "-L" "/Users/barrett/Documents/projects/Rover/auto_ros2/install/custom_interfaces_shim/lib" "-L" "<sysroot>/lib" "-L" "/Users/barrett/Documents/projects/Rover/auto_ros2/install/custom_interfaces_shim/lib" "-L" "<sysroot>/lib" "-L" "/Users/barrett/Documents/projects/Rover/auto_ros2/install/custom_interfaces_shim/lib" "-L" "<sysroot>/lib" "-L" "/Users/barrett/Documents/projects/Rover/auto_ros2/install/custom_interfaces_shim/lib" "-L" "<sysroot>/lib" "-L" "/Users/barrett/Documents/projects/Rover/auto_ros2/install/custom_interfaces_shim/lib" "-L" "<sysroot>/lib" "-L" "/Users/barrett/Documents/projects/Rover/auto_ros2/install/custom_interfaces_shim/lib" "-L" "<sysroot>/lib" "-o" "/Users/barrett/Documents/projects/Rover/auto_ros2/build/lights/debug/deps/lights_node-1db2c44ccab8761d" "-Wl,-dead_strip" "-nodefaultlibs"
  = note: some arguments are omitted. use `--verbose` to show all linker arguments
  = note: ld: warning: ignoring duplicate libraries: '-lbuiltin_interfaces__rosidl_generator_c', '-lbuiltin_interfaces__rosidl_typesupport_c', '-lgeometry_msgs__rosidl_generator_c', '-lgeometry_msgs__rosidl_typesupport_c', '-lstd_msgs__rosidl_generator_c', '-lstd_msgs__rosidl_typesupport_c', '-lunique_identifier_msgs__rosidl_generator_c', '-lunique_identifier_msgs__rosidl_typesupport_c'
          ld: warning: search path '/Users/barrett/Documents/projects/Rover/auto_ros2/src/custom_interfaces/bindings/custom_interfaces/../../../../install/custom_interfaces/lib' not found
          ld: library 'custom_interfaces__rosidl_typesupport_c' not found
          clang: error: linker command failed with exit code 1 (use -v to see invocation)


error: could not compile `lights` (bin "lights_node") due to 1 previous error
---
Failed   <<< lights [22.3s, exited with code 1]
Aborted  <<< sensors [23.6s]
Aborted  <<< wheels [23.7s]
Aborted  <<< maps_backend [23.7s]
Aborted  <<< custom_interfaces [43.0s]

Summary: 10 packages finished [43.2s]
  1 package failed: lights
  4 packages aborted: custom_interfaces maps_backend sensors wheels
~/D/p/R/auto_ros2 (refactor/pixi|✚5) $                                                   (auto_ros2)
```

We have to ensure our custom interfaces build first. This is a known problem with the IDL generation stuff in compiled languages like C, C++, and Rust.

Here's an example showing you how to fix it:

```fish
# first, build only the `custom_interfaces` package!
#
# you can specify that with `colcon build --symlink-install --packages-select custom_interfaces`,
# or on macOS like so:
~/D/p/R/auto_ros2 (refactor/pixi) $ colcon build --symlink-install --packages-select custom_interfaces --cmake-args -DPython_ROOT_DIR="$CONDA_PREFIX" -DPython_EXECUTABLE="$(python -c 'import sys; print(sys.executable)')"

Starting >>> custom_interfaces
Finished <<< custom_interfaces [11.7s]

Summary: 1 package finished [11.8s]
~/D/p/R/auto_ros2 (refactor/pixi|✚5) $                                                   (auto_ros2)

# now, build everything else by removing the `--packages-select`:
colcon build --symlink-install --cmake-args -DPython_ROOT_DIR="$CONDA_PREFIX" -DPython_EXECUTABLE="$(python -c 'import sys; print(sys.executable)')"
#
# ...
#
Summary: 15 packages finished [27.1s]
  6 packages had stderr output: custom_interfaces drive_launcher navigator see3cam simulator zed
~/D/p/R/auto_ros2 (refactor/pixi|✚5) $

# source the source script like usual
. ./SOURCE_SCRIPT.fish

# finally, run the node you want to run!
ros2 run lights lights_node

```

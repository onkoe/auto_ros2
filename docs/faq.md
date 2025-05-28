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

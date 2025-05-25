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

## When using `safe_drive`, the linker kinda just... blew up

If your output looks like this:

```console
(autonomous) remi@ubuntu ~/auto_ros2 (main)> colcon build --symlink-install --cargo-args --offline
Starting >>> custom_interfaces
Starting >>> aruco_node
Starting >>> drive_launcher
Starting >>> log_node
Starting >>> manual_control
Starting >>> navigator
Finished <<< drive_launcher [3.17s]
Starting >>> simulator
Finished <<< custom_interfaces [3.38s]
Starting >>> custom_interfaces_shim
Finished <<< custom_interfaces_shim [0.71s]
Starting >>> zed
Finished <<< zed [0.74s]
Starting >>> drive_tui
Finished <<< simulator [1.97s]
Starting >>> lights
Finished <<< drive_tui [0.64s]
Starting >>> sensors
Finished <<< manual_control [5.54s]
Starting >>> wheels
Finished <<< log_node [5.61s]
Finished <<< aruco_node [5.69s]
Finished <<< navigator [5.67s]
Finished <<< wheels [0.58s]
Finished <<< sensors [0.72s]
--- stderr: lights
error: linking with `cc` failed: exit status: 1
  |
  = note: LC_ALL="C" PATH="/home/remi/.rustup/toolchains/stable-aarch64-unknown-linux-gnu/lib/rustlib/aarch64-unknown-linux-gnu/bin:/opt/ros/humble/bin:/home/remi/auto_ros2/.venv/bin:/home/remi/.local/bin:/home/remi//(...SNIP...) "/home/remi/.rustup/toolchains/stable-aarch64-unknown-linux-gnu/lib/rustlib/aarch64-unknown-linux-gnu/lib/libcore-81963bdfbd961b87.rlib" "/home/remi/.rustup/toolchains/stable-aarch64-unknown-linux-gnu/lib/rustlib/aarch64-unknown-linux-gnu/lib/libcompiler_builtins-533cfa3936283188.rlib" "-Wl,-Bdynamic" "-lcustom_interfaces__rosidl_typesupport_c" "-lcustom_interfaces__rosidl_generator_c" "-lgeographic_msgs__rosidl_typesupport_c" "-lgeographic_msgs__rosidl_generator_c" "-lunique_identifier_msgs__rosidl_typesupport_c" "-lunique_identifier_msgs__rosidl_generator_c" "-lgeometry_msgs__rosidl_typesupport_c" "-lgeometry_msgs__rosidl_generator_c" "-lstd_msgs__rosidl_typesupport_c" "-lstd_msgs__rosidl_generator_c" "-lbuiltin_interfaces__rosidl_typesupport_c" "-lbuiltin_interfaces__rosidl_generator_c" "-lrcl" "-lrcl_action" "-lrcutils" "-lrmw" "-lrosidl_runtime_c" "-lactionlib_msgs__rosidl_typesupport_c" "-lactionlib_msgs__rosidl_generator_c" "-lbuiltin_interfaces__rosidl_typesupport_c" "-lbuiltin_interfaces__rosidl_generator_c" "-ldiagnostic_msgs__rosidl_typesupport_c" "-ldiagnostic_msgs__rosidl_generator_c" "-lgeometry_msgs__rosidl_typesupport_c" "-lgeometry_msgs__rosidl_generator_c" "-lnav_msgs__rosidl_typesupport_c" "-lnav_msgs__rosidl_generator_c" "-lsensor_msgs__rosidl_typesupport_c" "-lsensor_msgs__rosidl_generator_c" "-lshape_msgs__rosidl_typesupport_c" "-lshape_msgs__rosidl_generator_c" "-lstd_msgs__rosidl_typesupport_c" "-lstd_msgs__rosidl_generator_c" "-lstd_srvs__rosidl_typesupport_c" "-lstd_srvs__rosidl_generator_c" "-lstereo_msgs__rosidl_typesupport_c" "-lstereo_msgs__rosidl_generator_c" "-ltrajectory_msgs__rosidl_typesupport_c" "-ltrajectory_msgs__rosidl_generator_c" "-lunique_identifier_msgs__rosidl_typesupport_c" "-lunique_identifier_msgs__rosidl_generator_c" "-lvisualization_msgs__rosidl_typesupport_c" "-lvisualization_msgs__rosidl_generator_c" "-lrcl_interfaces__rosidl_typesupport_c" "-lrcl_interfaces__rosidl_generator_c" "-laction_msgs__rosidl_typesupport_c" "-laction_msgs__rosidl_generator_c" "-lgcc_s" "-lutil" "-lrt" "-lpthread" "-lm" "-ldl" "-lc" "-Wl,--eh-frame-hdr" "-Wl,-z,noexecstack" "-L" "/home/remi/auto_ros2/install/custom_interfaces_shim/lib" "-L" "/opt/ros/humble/lib" "-L" "/home/remi/auto_ros2/install/custom_interfaces_shim/lib" "-L" "/opt/ros/humble/lib" "-L" "/home/remi/auto_ros2/install/custom_interfaces_shim/lib" "-L" "/opt/ros/humble/lib" "-L" "/home/remi/auto_ros2/install/custom_interfaces_shim/lib" "-L" "/opt/ros/humble/lib" "-L" "/home/remi/auto_ros2/install/custom_interfaces_shim/lib" "-L" "/opt/ros/humble/lib" "-L" "/home/remi/auto_ros2/install/custom_interfaces_shim/lib" "-L" "/opt/ros/humble/lib" "-L" "/home/remi/auto_ros2/install/custom_interfaces_shim/lib" "-L" "/opt/ros/humble/lib" "-L" "/home/remi/.rustup/toolchains/stable-aarch64-unknown-linux-gnu/lib/rustlib/aarch64-unknown-linux-gnu/lib" "-o" "/home/remi/auto_ros2/build/lights/debug/deps/lights_node-189569ce1dcca893" "-Wl,--gc-sections" "-pie" "-Wl,-z,relro,-z,now" "-nodefaultlibs"
  = note: /usr/bin/ld: cannot find -lcustom_interfaces__rosidl_typesupport_c: No such file or directory
          /usr/bin/ld: cannot find -lcustom_interfaces__rosidl_generator_c: No such file or directory
          collect2: error: ld returned 1 exit status


error: could not compile `lights` (bin "lights_node") due to 1 previous error
---
Failed   <<< lights [1.65s, exited with code 1]

Summary: 12 packages finished [8.28s]
  1 package failed: lights
  5 packages had stderr output: custom_interfaces drive_launcher lights simulator zed
```

then `custom_interfaces` (or your equivalent) is not being placed on the `AMENT_PREFIX_PATH` environment variable. It needs to be there, as our Rust bindings generation code does the following:

```rust
fn main() {
    println!("cargo:rustc-link-lib=custom_interfaces__rosidl_typesupport_c");
    println!("cargo:rustc-link-lib=custom_interfaces__rosidl_generator_c");

    // IMPORTANT!! that env variable prevents the builds from failing!
    if let Some(e) = std::env::var_os("AMENT_PREFIX_PATH") {
        let env = e.to_str().unwrap();
        for path in env.split(':') {
            println!("cargo:rustc-link-search={path}/lib");
        }
    }
}
```

...which tells `cargo` to ask the linker to consider all the paths in `AMENT_PREFIX_PATH`. However, it'll only work if those libraries are in the `AMENT_PREFIX_PATH`.

So, to tell if that's your problem, you can type:

```console
(autonomous) barrett@p14s ~/auto_ros2 > echo $AMENT_PREFIX_PATH
/opt/ros/humble /home/barrett/auto_ros2/install/zed /home/barrett/auto_ros2/install/wheels /home/barrett/auto_ros2/install/simulator /home/barrett/auto_ros2/install/sensors /home/barrett/auto_ros2/install/navigator /home/barrett/auto_ros2/install/manual_control /home/barrett/auto_ros2/install/log_node /home/barrett/auto_ros2/install/lights /home/barrett/auto_ros2/install/drive_tui /home/barrett/auto_ros2/install/drive_launcher /home/barrett/auto_ros2/install/custom_interfaces_shim /home/barrett/auto_ros2/install/aruco_node
(autonomous) barrett@p14s ~/auto_ros2 >
```

If you don't see the library (in this case, `custom_interfaces`) in the list, then you'll need to add it to the builder package's `package.xml` as a `build_depend`.

You can also do a hacky workaround in your `build.rs` like so:

```rust
use std::path::Path;

fn main() {
    let dependencies: &[&str] = &["std_msgs", "custom_interfaces"];

    //          THIS RIGHT HERE vvvvvvv
    //
    // add `custom_interfaces` path to ld path
    let k = "/home/remi/auto_ros2/install/custom_interfaces/lib";
    println!("cargo:rustc-link-search={k}");

    safe_drive_msg::depends(
        &Path::new("../../lib/bindings")
            .canonicalize()
            .expect("Failed to canonicalize path (useful to prevent other build errors)"),
        dependencies,
        safe_drive_msg::SafeDrive::Version("0.4.3"),
    )
    .expect("Failed to build `custom_interfaces` bindings through `safe_drive_msg`!");
}
```

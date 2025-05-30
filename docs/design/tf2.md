# `tf2`

ROS 2's transformations library, `tf2`, provides a few constructs for building robotics projects:

- **Frames**: also known as "coordinate frames," these may be layered on top of one another to describe stuff in different coordinate spaces.
  - For example, the `base_link` frame exists on top of the `odom` frame. Since `base_link` _moves around_ on the `odom` frame, the entire system seems the Rover as moving within its local map of the real world.
- **Transforms**: each transform says how one frame works with another.
  - `odom` is local to `map` - it just moves around on `map`. So, if the Rover is at (0, 0) on the `map`, but the `odom` frame has noticed 1m of necessary corrections with its sensors, it'll move to (0, 1) on `map`.
  - In other words, the transform between the two frames changes.
- **Chains**: a set of multiple frames linked together within the tree.
  - These can start from the global frame (sometimes called the "static layer," like `map`), or anywhere else within the tree.
  - A common example is `map -> odom -> base_link`, which defines the transforms between the static layer, local odometry layer, and Rover's position.
  - Other examples, like `base_link -> chassis -> imu_link` are just as valid - even if they're from the middle of the tree.
- **Tree**: the `tf` tree includes all frames and their links.
  - It can include several chains.
  - When there are multiple `tf2` trees in one ROS 2 domain, you'll start to see weird behavior (because nodes don't know which tree to use)!
    - Try combining the two trees by adding transforms. [`tf2_ros::static_transform_publisher` is a good start](https://github.com/ros2/geometry2/blob/d43b1975b40235388c64fa5c354ec38a507bdfc7/tf2_ros/doc/cli_tools.rst#3-tf-manipulation)!

## Models

<!--
TODO: remove the out-of-date ROS Wiki link below when a proper URDF spec exists.

I've asked about it in the ROS 2 discord:
https://discord.com/channels/1077825543698927656/1077836632658542634/1378032135901806695
-->

There's also the modeling side of all this. The [URDF](https://wiki.ros.org/urdf) ("Unified Robot Description Format"), is a description file made in XML. URDF currently lacks a specification, so you'll probably want to borrow ideas from other projects.

The [Awesome URDF collection](https://github.com/ami-iit/awesome-urdf) provides lots of useful examples! :D

> [!WARNING]
> SDF is similar, but intended only for usage in the Gazebo simulator. It provides support for plugins, which marks its difference from URDF - SDF can actually provide **behavior** to a robot.
>
> Its primary use is in creating Gazebo worlds. Note that its behavior can be added inside of a URDF [with the `<gazebo>` tag](http://sdformat.org/tutorials?tut=sdformat_urdf_extensions&cat=specification&)!
>
> For more info about SDF, see [its specification here](http://sdformat.org/spec)!

In any case, let's touch briefly on two vital elements of URDF. These two constructs help **describe `tf2` transformations implicitly**!

- **Links**: the rigid parts of a robot.
  - Since links don't change much, they're the perfect "base" for transformations.
- **Joints**: moving parts of a robot which attach links together.
  - These are moving parts of the robot - think of your thigh + shin being linked by your knee.

Since these two parts provide nice coordinate frames on top of mapping/odometry stuff, `robot_state_publisher` and other nodes often parse URDF to get info about where things on the Rover are. For example, we can tell `robot_localization::navsat_transform_node` where the GPS is, which offsets the Rover's posiiton for Nav2 according to its distance in the URDF file.

## In `auto_ros2`...

Great, we've described how all that works! Now, let's describe how we use `tf2` in our project...

### Tree

Here's how the `tf` tree looks. Note that top-level frames are left-most in the list.

- `map`: the global frame. Provided by `slam_toolbox`.
  - `odom`: wheel/sensor odometry, which is essentially where the Rover believes it has moved. Provided by `ekf_filter_node_odom` (which is `robot_localization::ekf_filter_node`).
    - `base_link`: The Rover model. Provided by `robot_state_publisher::robot_state_publisher`.

<!--
TODO: notes on each frame, its transform, link, and who provides it

### Frames

#### `map`

...
-->

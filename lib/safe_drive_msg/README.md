# `safe_drive_msg`

A custom source of Tier4's [`safe_drive_msg` library](https://github.com/tier4/safe_drive_msg), with a couple custom patches to remove warnings + [improve generation](https://github.com/tier4/safe_drive_msg/pull/7).

## Usage

You shouldn't use this library directly. It's only used in `src/custom_interfaces_shim/build.rs` to generate ROS 2 types for usage in Rust, as Colcon doesn't have a "main" community extension to do that just yet.

## Why's it here?

It's kept here since `git` repos need to be gathered before the Rover is online, and we forget like 98% of the time. So, yay! Monorepo! :D

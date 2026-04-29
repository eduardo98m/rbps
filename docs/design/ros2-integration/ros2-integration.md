@page feature_ros2 ROS 2 Bridge

# ROS 2 Bridge

## Current State
RBPS is standalone — no ROS-side hooks. Communication out of the
engine goes through [include/visr/Command.hpp](../../../include/visr/Command.hpp)
(render → physics) and [include/visr/Snapshot.hpp](../../../include/visr/Snapshot.hpp)
(physics → render). The transport contract in
[Transport.hpp](../../../include/visr/Transport.hpp) is generic enough that
a ROS bridge can plug in as another transport rather than as a special
case bolted onto `World`.

## Motivation
ROS 2 is the lingua franca of robotics. Wiring RBPS into the ROS
ecosystem makes it usable as a hardware-in-the-loop / software-in-
the-loop tester for real robots: subscribe to `cmd_vel`, publish
`tf`, `joint_states`, sensor data. Combined with
[scene-format.md](../scene-format/scene-format.md) (URDF) and
[sensors-gpu.md](../sensors-gpu/sensors-gpu.md), RBPS becomes a credible Gazebo
alternative for small projects.

## Proposed Approach
**Decouple via a new transport, not a deep code change.**

1. New module `rbps_ros2/` (built only when ROS 2 is detected via
   `find_package(rclcpp)`). Disabled by default, like `RBPS_BUILD_VISR`.
2. `Ros2Transport` implements the `is_debug_transport` contract:
   - `push_snapshot(const FrameSnapshot&)` translates each `BodySnap`
     to a `geometry_msgs/TransformStamped`, publishes the full TF tree,
     plus per-body `nav_msgs/Odometry`. Joints become
     `sensor_msgs/JointState`.
   - `poll_command(Command&)` translates incoming
     `geometry_msgs/Twist` (per body) and joint targets back into
     RBPS `Command` variants.
3. Sensor pipeline: when sensors land
   ([sensors-gpu.md](../sensors-gpu/sensors-gpu.md)), publish `sensor_msgs/Image` and
   `sensor_msgs/PointCloud2`. Mounting is already in
   `CameraSensor::mount` so the TF frame is just the parent body's
   frame + the mount offset.
4. Clock: publish `/clock` from the physics thread so ROS 2 nodes can
   run in simulation time. Required by every ROS controller that uses
   `ros::Time::now()`.
5. Topic remapping is configured via the standard ROS 2 launch file —
   no RBPS-side knobs needed.

## Risks / Open Questions
- ROS 2 `rclcpp` is heavy (~100 MB of dependencies). Keep the
  `RBPS_BUILD_ROS2` opt-in default-OFF and out of CI unless a
  ROS-tagged runner is added.
- Threading: `rclcpp::spin` wants its own executor thread. The
  `Ros2Transport` wraps it so neither the physics nor the render thread
  touches `rclcpp` directly.
- Clock skew: simulation time must NOT drift relative to wall clock by
  default. Flag a "real-time factor" knob in the bridge config so
  faster-than-real-time runs are explicit.
- Should `Ros2Transport` co-exist with `InProcessTransport` (visualizer
  + ROS at once)? Easiest answer: ship a `BroadcastTransport` wrapper
  that fans out to both — covers the common debug case.

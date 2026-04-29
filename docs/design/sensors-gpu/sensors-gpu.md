@page feature_sensors_gpu GPU Camera + LiDAR Sensors

# GPU Camera + LiDAR Sensors

## Current State
No sensor model exists. The visualizer only renders a debug view of
the world; nothing is exported as image or point-cloud data. raylib is
linked in [CMakeLists.txt](../../../CMakeLists.txt) and supports compute
shaders (`rlLoadComputeShaderProgram`, `rlComputeShaderDispatch`) on
OpenGL 4.3+ — those primitives are unused today.

## Motivation
Robotics, perception, and SLAM research all need synthetic sensor data
generated at simulation speed. Off-the-shelf solutions (Gazebo,
Isaac Sim) carry heavy dependencies; a small native sensor module
unlocks RBPS as a teaching / prototyping tool for those workflows
without leaving C++.

## Proposed Approach
**New module `rbps_sensors` (header-only first, library if shaders
warrant it).** Sensors are owned alongside `World` so they can be
ticked from the same step loop.

1. Sensor types in `include/rbps_sensors/`:
   ```cpp
   struct CameraSensor   { uint32_t body_id; m3d::tf mount; int w, h; float fov; };
   struct LiDARSensor    { uint32_t body_id; m3d::tf mount; int rays_h, rays_v; float fov_h, fov_v; float max_range; };
   ```
   Both attach to a body so they move with the simulation.
2. CPU fallback first: `update_camera(CameraSensor&, const World&)`
   ray-casts every pixel against the existing
   [Dispatcher.hpp](../../../include/rbc/Dispatcher.hpp) — slow but
   reference-correct. Same for LiDAR.
3. GPU acceleration via raylib compute shaders:
   - Pack collider data into a SSBO each frame (the same way
     `SnapshotBuilder` packs it for the visualizer — a candidate for
     reusing the snapshot path).
   - Compute shader dispatches one workgroup per ray; writes hit
     distance + normal + collider ID to an output buffer.
   - Read back via `rlReadShaderBuffer`. For tight loops, keep the
     buffer GPU-resident if the consumer is also a shader.
4. Outputs: image `std::vector<uint8_t>` (RGBA) for cameras, point
   cloud `std::vector<vec3>` for LiDAR. Add ROS 2 message conversion
   helpers when [ros2-integration.md](../ros2-integration/ros2-integration.md) lands.
5. Demo panel: add `ui::draw_sensor_panel` that renders the camera
   image as an `ImGui::Image` and the LiDAR point cloud as a 3-D
   overlay in the main view.

## Risks / Open Questions
- Shape coverage: GPU intersection requires per-shape closed-form ray
  tests. Sphere / box / plane / capsule are easy; mesh and heightmap
  need a BVH ([broadphase-bvh.md](../broadphase-bvh/broadphase-bvh.md)) on the GPU side.
  Start with the easy shapes and document the limitation.
- GPU readback latency adds 1 frame; flag whether sensors are tied to
  the physics tick or to render frames in their config struct.
- Compute-shader support is OpenGL 4.3+ only; raylib falls back to
  OpenGL 2.1 on some platforms. Provide the CPU fallback unconditionally.
- LiDAR noise modelling (Gaussian / drop-out / multi-return) is out of
  scope for v1 — emit clean data and let consumers add noise.

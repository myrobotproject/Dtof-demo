# tof_demo


`tof_demo` is a ROS1 Melodic catkin package for reverse-driving automatic braking with an HM-LD1 UVC ToF camera. The node publishes a reverse velocity to `cmd_vel` while the path is clear. When the number of point-cloud samples that match the configured height range and distance threshold reaches the obstacle threshold, it immediately publishes a zero-velocity command.

## Build

Keep the workspace layout as follows:

```text
/home/agilex/tof_ws/src/
  hm_ld1_sdk/
  tof_demo/
```

Build the workspace:

```bash
cd /home/agilex/tof_ws
source /opt/ros/melodic/setup.bash
catkin_make
```

## Run

Set the speed magnitude with the runtime parameter `reverse_speed_mps`. The node always publishes it as a negative `linear.x`, so the vehicle only drives backward:

```bash
source /opt/ros/melodic/setup.bash
source /home/agilex/tof_ws/devel/setup.bash
roslaunch tof_demo tof_reverse_brake.launch
```

Common parameter overrides:

```bash
roslaunch tof_demo tof_reverse_brake.launch \
  uvc_device:=/dev/video0 \
  cmd_vel_topic:=/cmd_vel \
  reverse_speed_mps:=0.10
```

## YAML Parameters

The default configuration is in `config/tof_reverse_brake.yaml`:

- `stop_distance_m`: obstacle distance threshold.
- `min_distance_m`: filters out very near noise points.
- `min_obstacle_points`: number of obstacle points required to trigger braking.
- `clear_required_frames`: number of consecutive clear point-cloud frames required to release braking. The default is 3; braking still triggers immediately from a single frame.
- `height_axis`, `height_sign`, `height_min_m`, `height_max_m`: height-range filter.
- `distance_axis`, `distance_sign`: distance axis and direction.
- `publish_rate_hz`: `cmd_vel` publish rate.
- `frame_timeout_ms`, `stop_on_no_data`: whether to keep zero velocity when no fresh point cloud is available. The default timeout is 2000 ms.

SDK point-cloud coordinates are in millimeters: `x` points right, `y` points down, and `z` points forward from the camera. By default, distance uses `z` and height uses `-y`. If the camera is mounted in a different orientation, adjust the axis and sign fields in the YAML file first.

## Test

When the chassis control node is not running, you can still check the camera and topic outputs:

```bash
rostopic echo /tof_reverse_brake_node/obstacle_point_count
rostopic echo /tof_reverse_brake_node/nearest_obstacle_distance
rostopic echo /tof_reverse_brake_node/stop_active
rostopic echo /cmd_vel
```

When the path is clear and the point cloud is valid, `/cmd_vel.linear.x` should be `-abs(reverse_speed_mps)`. After the obstacle point count reaches `min_obstacle_points`, `/cmd_vel` should be all zeros. After startup, the node keeps zero velocity until it receives the first valid point-cloud frame.

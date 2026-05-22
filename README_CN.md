# tof_demo

[English](README.md) | 中文

`tof_demo` 是一个 ROS1 Melodic catkin 包，用 HM-LD1 UVC ToF 相机点云实现倒车自动刹停。节点无障碍时持续向 `cmd_vel` 发布倒车速度；满足高度范围和距离阈值的障碍点数量达到阈值后，立即发布零速。

## 编译

保持工作空间结构如下：

```text
/home/agilex/tof_ws/src/
  hm_ld1_sdk/
  tof_demo/
```

编译：

```bash
cd /home/agilex/tof_ws
source /opt/ros/melodic/setup.bash
catkin_make
```

## 运行

速度大小在运行参数 `reverse_speed_mps` 中填写，节点会强制发布为负 `linear.x`，即只倒车：

```bash
source /opt/ros/melodic/setup.bash
source /home/agilex/tof_ws/devel/setup.bash
roslaunch tof_demo tof_reverse_brake.launch
```

常用覆盖参数：

```bash
roslaunch tof_demo tof_reverse_brake.launch \
  uvc_device:=/dev/video0 \
  cmd_vel_topic:=/cmd_vel \
  reverse_speed_mps:=0.10
```

## YAML 参数

默认配置在 `config/tof_reverse_brake.yaml`：

- `stop_distance_m`：障碍距离阈值。
- `min_distance_m`：过滤过近噪声点。
- `min_obstacle_points`：触发刹停所需障碍点数量。
- `clear_required_frames`：解除刹停需要连续低于阈值的点云帧数，默认 3；触发刹停仍是单帧立即生效。
- `height_axis`、`height_sign`、`height_min_m`、`height_max_m`：高度范围过滤。
- `distance_axis`、`distance_sign`：距离轴和方向。
- `publish_rate_hz`：`cmd_vel` 发布频率。
- `frame_timeout_ms`、`stop_on_no_data`：无新点云时是否保持零速，默认超时为 2000 ms。

SDK 点云单位为 mm，坐标为 `x` 向右、`y` 向下、`z` 向相机前方。默认距离用 `z`，高度用 `-y`。如果相机安装方向不同，先改 YAML 里的轴和符号。

## 测试

不开启底盘控制节点时，可以只检查相机和话题输出：

```bash
rostopic echo /tof_reverse_brake_node/obstacle_point_count
rostopic echo /tof_reverse_brake_node/nearest_obstacle_distance
rostopic echo /tof_reverse_brake_node/stop_active
rostopic echo /cmd_vel
```

无障碍且点云正常时，`/cmd_vel.linear.x` 应为 `-abs(reverse_speed_mps)`；障碍点数量达到 `min_obstacle_points` 后，`/cmd_vel` 应为全零。节点启动后在收到第一帧有效点云前默认保持零速。

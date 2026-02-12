# Nav2 入门教程（基于 ROS 2）

本教程面向初学者，帮助你从零到一完成 Nav2（Navigation2）的基础集成与运行，涵盖坐标系、必要消息、里程计设置、传感器融合、参数配置、启动与验证等关键环节。示例基于常见差速驱动移动机器人。

## 1. 前置条件
- 已安装 ROS 2（建议 Humble 及以上）与工作空间
- 已安装 Nav2 包（nav2_bringup 等）
- 能启动机器人底层驱动并输出基本传感器数据（TF、IMU、编码器/LiDAR）
- 熟悉 ROS 2 节点、话题、参数、TF 变换基础

## 2. 坐标系与 TF 结构
- map：全局、长期准确的坐标系（来自定位/SLAM）
- odom：短期连续的里程计坐标系，会随时间/距离漂移
- base_link：机器人机体坐标系
- 常见链路：map → odom → base_link → base_footprint（或直接 base_link）
- 关键要求：
  - 持续发布 odom → base_link 的 TF 变换
  - 若使用定位（AMCL/SLAM），发布 map → odom 的 TF 变换

## 3. Nav2 所需核心接口
- TF 变换
  - odom → base_link（里程计来源或 robot_localization 发布）
  - map → odom（定位模块如 AMCL/SLAM 发布）
- 话题
  - nav_msgs/Odometry：包含位姿（pose）与速度（twist）
    - header.frame_id：位姿父坐标系（通常为 odom）
    - child_frame_id：子坐标系（通常为 base_link）
    - pose：相对 header.frame_id 的位置与姿态
    - twist：相对 child_frame_id 的线速度与角速度
- 传感器
  - 激光/LiDAR（/scan 或 PointCloud2）
  - IMU（/imu）
  - 编码器（供里程计估算）

## 4. 里程计入门（差速驱动）
- 差速估算公式（示意）：
  - 线速度：linear = (v_right + v_left) / 2
  - 角速度：angular = (v_right - v_left) / wheel_separation
- v_left/v_right 可由左右轮角度随时间的变化计算得到
- 发布内容：
  - nav_msgs/Odometry（提供 pose/twist）
  - TF：odom → base_link
- 重要认识：
  - IMU 会随时间漂移，编码器会随距离漂移；常组合使用，并通过融合减小误差

## 5. 传感器融合（robot_localization）
- 使用 robot_localization（EKF/UKF）融合 IMU、编码器等，生成平滑的里程计并发布 odom → base_link
- 基本步骤：
  1) 准备输入：wheel odom（或里程计节点输出）、IMU
  2) 配置 EKF 节点参数（输入话题、使用的状态变量、噪声）
  3) 启动并验证输出的 odom → base_link TF 与 /odometry/filtered

示例（ekf.yaml，示意）：

```yaml
ekf_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    odom0: /wheel/odom
    odom0_config: [true, true, false,
                   false, false, true,
                   true, true, false,
                   false, false, true]
    odom0_differential: false

    imu0: /imu
    imu0_config: [false, false, false,
                  true,  true,  true,
                  false, false, false,
                  false, false, true]
    imu0_differential: false
    imu0_remove_gravitational_acceleration: true
```

说明：
- odom0/imu0 为输入话题；*_config 指定使用哪些状态变量（位置、姿态、速度等）
- two_d_mode 用于平面移动机器人
- world_frame 选择 odom，以输出连续里程计并发布 odom → base_link

## 6. Nav2 组件概览
- 地图与定位
  - map_server：加载静态地图
  - AMCL：在静态地图上定位并发布 map → odom
  - SLAM（替代 AMCL+map_server，在线构图与定位）
- 规划与控制
  - Global Planner：全局路径规划
  - Local Planner/Controller：局部轨迹跟踪（如 DWB、RegulatedPurePursuit）
  - Costmap（Global/Local）：障碍物栅格
  - Behavior Tree：高层任务逻辑（导航、恢复等）
- 任务接口
  - /navigate_to_pose 等 action，用于发送导航目标

## 7. 最小化示例参数与启动
示例 nav2_params.yaml（示意）：

```yaml
amcl:
  ros__parameters:
    use_map_topic: true
    base_frame_id: base_link
    odom_frame_id: odom
    scan_topic: /scan

map_server:
  ros__parameters:
    yaml_filename: /path/to/map.yaml

planner_server:
  ros__parameters:
    expected_planner_frequency: 10.0

controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]

bt_navigator:
  ros__parameters:
    default_bt_xml_filename: "navigate_w_recovery.xml"
```

示例启动（Python Launch，简化示意）：

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_params = os.path.join(
        get_package_share_directory('your_pkg'),
        'config', 'nav2_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            output='screen',
            parameters=[nav2_params]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('your_pkg'),
                'config', 'ekf.yaml'
            )]
        )
    ])
```

## 8. 启动前检查清单
- TF 连通且方向正确
  - 连续的 odom → base_link
  - 若使用 AMCL/SLAM，存在 map → odom
- nav_msgs/Odometry 的 header.frame_id 与 child_frame_id 合理
  - 通常 header.frame_id 为 odom，child_frame_id 为 base_link
- 传感器数据频率稳定（IMU、LiDAR）
- 机器人控制器（/cmd_vel）与底盘联通正常

## 9. 运行与验证
- 可视化
  - 使用 RViz2 加载 TF、地图、路径、机器人模型，检查坐标系一致性
- 里程计与融合
  - 观察 /odometry/filtered（若用 robot_localization）与 TF 的连续性
- 定位
  - 若使用 AMCL，初始位姿设置后应能稳定发布 map → odom
- 导航
  - 在 RViz2 发送 “2D Nav Goal” 或调用 /navigate_to_pose

## 10. 常见问题与排查
- TF 断链或方向错误：检查父子关系与名称一致性
- 速度单位/轮距配置错误：确认编码器换算与参数（wheel_separation、轮径等）
- header.frame_id/child_frame_id 混用：遵循 Odometry 规范
- two_d_mode 未启用：平面机器人启用，避免 Z 轴与俯仰/翻滚干扰
- 话题名不一致：统一 IMU、LiDAR、Odometry、cmd_vel 等话题名
- 频率过低/不稳定：确认数据与融合频率（50Hz 常见）

## 11. 实践建议
- 先实现可用的轮式里程计与 IMU 融合，获得稳定的 odom → base_link
- 再接入 AMCL（静态地图）或 SLAM（在线构图），提供 map → odom
- 逐步引入 Nav2 组件，从最小参数集开始，逐项调优

## 12. 参考
- REP 105 坐标系约定
- Nav2 官方文档与示例
- 里程计与 robot_localization：IMU 与编码器漂移特性互补，融合提升稳定性；发布 nav_msgs/Odometry 与 odom → base_link 是 Nav2 正常运行的基础


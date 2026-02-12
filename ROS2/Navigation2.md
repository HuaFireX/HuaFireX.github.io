# Navigation 2入门

## 安装与运行示例（以 ROS2 发行版 `humble` 为例）

### 安装

```bash
sudo apt install ros-<ros2-distro>-navigation2
sudo apt install ros-<ros2-distro>-nav2-bringup
sudo apt install ros-<ros2-distro>-turtlebot3*
```

> 以上命令中的 `<ros2-distro>` 请替换为你的发行版（例如 `humble`）。

### 运行示例

```bash
source /opt/ros/<ros2-distro>/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/<ros2-distro>/share/turtlebot3_gazebo/models
# 如果 GPU 不可用，可强制软件渲染：
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

注意：`headless` 默认是 `true`，如需使用 `gzclient` 请设置为 `False`（或 `headless:=False`）。

## 导航相关概念

- TF / TF2
- RViz（可视化）
- Gazebo（3D 仿真）
- URDF（机器人描述）
- SDF（Gazebo 使用的机器人/场景描述）

## 首次机器人配置指南

- 推荐使用启动指令：

```bash
gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so
```

## 编写机器人

### 1. 设置转换TF2

TF2（Transform Framwork 2）用于管理机器人坐标系之间的变换关系。

### 2. 创建功能包

```bash
ros2 pkg create --build-type ament_cmake sam_bot_description
```

### 3. 编写 URDF（Unified Robot Description Format）

URDF 是 ROS 中描述机器人结构、链接和关节的标准格式。

**安装必要工具：**

```bash
sudo apt install ros-<ros2-distro>-joint-state-publisher-gui
sudo apt install ros-<ros2-distro>-xacro
```

**xacro**：允许在 URDF 中使用变量和宏，简化复杂的机器人描述。

### 4. 编写 SDF（Simulation Description Format）

SDF 是 Gazebo 仿真环境中使用的机器人/环境描述格式。

**安装工具：**

```bash
sudo apt install ros-<ros2-distro>-sdformat-urdf
```

### 5. 设置里程计 - Odometry（Gazebo 仿真）

**安装 3D 模拟器 Gazebo：**

```bash
sudo apt install ros-<ros2-distro>-ros-gz
```

### 6. 实现里程平滑化 - 拓展卡尔曼滤波融合

使用 robot_localization 包融合多个传感器（里程计、IMU 等）以获得平滑的位姿估计。

**安装定位包：**

```bash
sudo apt install ros-<ros2-distro>-robot-localization
```

**编写配置文件 ekf.yaml：**

ekf.yaml 包含 EKF 节点的参数配置，定义输入话题、融合的传感器数据和噪声模型。

### 7. 设置传感器 - Sensors（Gazebo 仿真）

在 Gazebo 中配置机器人传感器：
- **雷达（LiDAR）**：生成环境的点云数据
- **相机**：RGB 或深度相机用于视觉导航

### 8. 制图与定位

#### 8.1 SLAM - Simultaneous Localization and Mapping

**slam_toolbox**：二维同时定位与地图构建工具包

**AMCL**：自适应蒙特卡洛定位工具包

#### 8.2 二维代价地图

**Costmap 2D 软件包**：生成全局和局部代价地图，用于路径规划和碰撞检测。

#### 8.3 运行验证

**步骤 1：启动机器人描述节点**

```bash
colcon build
. install/setup.bash
ros2 launch sam_bot_description display.launch.py
```

**步骤 2：启动 slam_toolbox**

```bash
# 开启新终端
sudo apt install ros-<ros2-distro>-slam-toolbox

# 启动 slam 节点
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

在 RViz 中查看地图：`Add → Topic → map`

**步骤 3：创建变换树（frame 关系图）**

```bash
ros2 run tf2_tools view_frames
```

生成 `frames.pdf` 显示所有 TF 关系。

**步骤 4：启动 Nav2**

```bash
# 确保已安装 Nav2 软件包
sudo apt install ros-<ros2-distro>-navigation2
sudo apt install ros-<ros2-distro>-nav2-bringup

# 启动 Nav2
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

#### 8.4 代价地图可视化

**全局代价地图（Global Costmap）：** 基于全局地图的成本栅格

**局部代价地图（Local Costmap）：** 基于机器人周围传感器数据的成本栅格

**可视化体素表示：**

```bash
ros2 run nav2_costmap_2d nav2_costmap_2d_markers voxel_grid:=/local_costmap/voxel_grid visualization_marker:=/my_marker
```

在 RViz 中查看标记：`Add → /mymarker topic → Marker`

### 9. 建立 Footprint（机器人占地面积）

机器人的占地面积定义用于碰撞检测和代价地图膨胀。

**步骤 1：启动机器人描述**

```bash
# 终端 1
colcon build
. install/setup.bash
ros2 launch sam_bot_description display.launch.py
```

**步骤 2：发布静态 TF 变换**

```bash
# 终端 2
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

**步骤 3：启动 Nav2 导航**

```bash
# 终端 3
# 复制配置文件到 config 目录
# 来源：https://github.com/ros-navigation/navigation2_tutorials/blob/humble/sam_bot_description/config/nav2_params.yaml
# 注意：选择与 Ubuntu 版本对应的分支！

ros2 launch nav2_bringup navigation_launch.py params_file:=<full/path/to/config/nav2_params.yaml>
```

**步骤 4：可视化局部代价地图**

在 RViz 中：`Add → LocalCostmap → Add`，固定坐标系为 `odom`，应看到机器人的矩形足迹

**步骤 5：可视化全局代价地图**

在 RViz 中：`Add → GlobalCostmap → Add`，固定坐标系为 `map`，应看到机器人的圆形轮廓

### 10. 设置导航插件

Nav2 支持多种规划器、控制器和行为树插件，可根据需求配置。

### 11. 设置生命周期和组合节点

生命周期管理允许节点在不同的生命周期状态间转换（unconfigured → inactive → active → finalize）。

组合节点将多个功能整合为单个节点，便于管理复杂的系统。

## fishbot 配置

- 两轮差速控制插件示例：

![fishbot 控制示意图](./img/屏幕截图%202026-02-09%20145539.png)

- 键盘控制：

```bash
sudo apt install ros-foxy-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- 使用 `rqt` 显示速度（Plugin → Visualization → Plot）

![rqt plot](./img/屏幕截图%202026-02-09%20145549.png)

## 导航实战

### Gazebo 仿真环境准备

常用模型下载（示例使用脚本）：

```bash
cd ~/.gazebo && wget https://gitee.com/ohhuo/scripts/raw/master/gazebo_model.py && python3 gazebo_model.py
```

启动时加载世界模型和机器人模型。

## SLAM 建图

- Cartographer（示例，视具体需求选择 SLAM 算法）

## Nav2 导航

APT 安装示例：

```bash
sudo apt install ros-humble-nav2-*
```

测试是否安装成功：

```bash
ros2 pkg list | grep navigation2
```

为 fishbot 配置 Nav2 示例：

```bash
ros2 pkg create fishbot_navigation2 --dependencies nav2_bringup
```

## 运行官方示例

- 运行 `nav_to_pose_example_launch.py` 等教程示例。

## 常用节点

- 键盘操控节点：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 杂项

- 在当前虚拟环境中打开 `rqt`（避免依赖问题）：

```bash
python -m rqt_gui.main
```

- colcon 构建时忽略指定文件夹（示例）：

```bash
touch py_venv/COLCON_IGNORE
```

- 关闭 Gazebo 与 RViz2 建议流程：`File → Quit`；如遇无法启动仿真，可尝试：

```bash
wsl --shutdown
# 然后重启 WSL
```

## 参考

- navigation2_tutorials 示例：
	- https://github.com/ros-planning/navigation2_tutorials/tree/master/sam_bot_description
- FishROS 教程（humble）：
	- https://fishros.com/d2lros2/#/humble
- FishROS Nav2 中文文档：
	- http://dev.nav2.fishros.com
- Nav2 官方文档：
	- https://docs.nav2.org/
- ROS 2 URDF 指南：
	- https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html

---

*笔记由导出的 HTML 转换并整理，若需调整标题层级或把部分内容拆分为子页面，请告诉我。*
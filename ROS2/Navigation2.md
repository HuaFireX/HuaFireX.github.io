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

（略 — 可在此处补充机器人模型/URDF/SDF 相关说明）

## fishbot 配置

- 两轮差速控制插件示例：

![fishbot 控制示意图](https://api2.mubu.com/v3/document_image/22676279_2b1c38ee-a8da-4fc8-99fb-dc13f978cad3.png)

- 键盘控制：

```bash
sudo apt install ros-foxy-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- 使用 `rqt` 显示速度（Plugin → Visualization → Plot）

![rqt plot](https://api2.mubu.com/v3/document_image/22676279_c8b5e53c-4241-4926-cfce-25d3e179f8b8.png)

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
# VDA5050_tb3_Nav2测试运行流程

本文档介绍如何在 Nav2 仿真环境中运行 VDA5050 接口的 TurtleBot3 机器人。

---

## 1. 克隆仓库

```bash
mkdir -p vda5050_nav2_ws/src	#创建项目工作空间
cd vda5050_nav2_ws/src
git clone https://github.com/inorbit-ai/vda5050_adapter_examples.git
git clone https://github.com/inorbit-ai/ros_amr_interop.git
#克隆后把其中的vda5050_connector、vda5050_msgs、vda5050_serializer挪到src目录下，之后可删掉ros_amr_interop

```

---

## 2. 环境准备

确保已修复 NumPy 2.0 兼容性问题：

```bash
python3 -m venv .venv		#建议先创建py虚拟环境
source .venv/bin/activate   	#激活虚拟环境
pip install "numpy<2.0"
```

---

## 3. 修改配置文件

在启动 VDA5050 适配器前，需要修改配置文件中的 MQTT 地址：

```bash
cd vda5050_adapter_examples/vda5050_tb3_adapter/config
vim connector.yaml
```

修改 `mqtt_address` 为你的 MQTT 服务器地址（如果是本机则保持默认）：

```yaml
mqtt_address: "localhost"
```

---

## 4. 准备构建目录

```bash
#进入ros_amr_interop目录
colcon build
```

---

## 5. 运行步骤 (多终端顺序执行)

### 终端 1: 启动 MQTT 代理 (Docker)

```bash
cd src/vda5050_adapter_examples/docker
docker run --rm -p 18883:1883 --name mosquitto eclipse-mosquitto
```

### 终端 2: 启动 Nav2 仿真 (TurtleBot3 Simulation)

```bash
export TURTLEBOT3_MODEL=waffle
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

### 终端 3: 启动 VDA5050 TB3 连接器

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vda5050_tb3_adapter connector_tb3.launch.py
```

### 终端 4: 设定机器人初始位姿 (关键)

由于这是标准的 ROS2 话题，通常只需要 `source` ROS2 基础环境即可。如果该命令报错，请加上 `source install/setup.bash`。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash	#不source，话题发不出去
ros2 topic pub -1 --qos-reliability reliable /initialpose geometry_msgs/PoseWithCovarianceStamped \
   "{header: {frame_id: map}, pose: {pose: {position: {x: -2.1, y: -0.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0, w: 1.0000000}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]}}"
```

> 注意：初始位姿设定是让机器人能够进行导航的关键步骤。发布 `initialpose` 后，机器人在 Gazebo 中会定位到地图的正确位置。

---

## 6. 发送测试订单 (MQTT)

当机器人在 Gazebo 中定位成功（`initialpose` 发布后机器人会出现在地图正确位置）后，发送目标点：

### 方式一：命令行发送

```bash
mosquitto_pub -h localhost -p 18883 -t uagv/v2/OSRF/TB3_1/order -m '{
    "orderId": "'$(cat /proc/sys/kernel/random/uuid)'",
    "orderUpdateId": 0,
    ".0.0version": "2",
    "manufacturer": "OSRF",
    "serialNumber": "TB3_1",
    "nodes": [
        {
            "nodeId": "start_node",
            "released": true,
            "sequenceId": 0,
            "nodePosition": {"x": -2.1, "y": -0.5, "theta": 0.0},
            "actions": []
        },
        {
            "nodeId": "target_node",
            "released": true,
            "sequenceId": 1,
            "nodePosition": {"x": 2.0, "y": 0.5, "theta": 0.0},
            "actions": []
        }
    ],
    "edges": [
        {
            "edgeId": "edge1",
            "sequenceId": 0,
            "released": true,
            "startNodeId": "start_node",
            "endNodeId": "target_node",
            "actions": []
        }
    ]
}'
```

### 方式二：MQTT Explorer（可视化工具）

推荐使用 MQTT Explorer 进行可视化接收和发布：

1. 下载安装：
2. 配置连接：

   - Host: `localhost`
   - Port: `18883`
3. 订阅话题：接收机器人状态

   - 订阅 `uagv/v2/OSRF/TB3_1/state`
4. 发布话题：发送订单

   - 发布到 `uagv/v2/OSRF/TB3_1/order`
   - 粘贴上面的 JSON 内容

> MQTT Explorer 可以实时查看话题消息，方便调试。

---

## 流程总结

| 步骤            | 说明                               |
| --------------- | ---------------------------------- |
| 1. 克隆仓库     | 克隆 vda5050_adapter_examples 仓库 |
| 2. 环境准备     | 安装兼容版本的 NumPy               |
| 3. 修改配置文件 | 修改 MQTT 地址                     |
| 4. 构建项目     | 创建目录结构并编译                 |
| 5. 启动 MQTT    | Docker 运行 Mosquitto 代理         |
| 6. 启动仿真     | Nav2 + TurtleBot3 Gazebo 仿真      |
| 7. 启动适配器   | VDA5050 TB3 连接器                 |
| 8. 设定初始位姿 | 发布 `/initialpose` 话题         |
| 9. 发送订单     | 通过 MQTT 下发任务                 |

这样整个流程就闭环了：**克隆仓库 -> 环境准备 -> 修改配置 -> 构建项目 -> 基础组件启动 -> 仿真定位 -> VDA5050 任务下发**。

# VDA5050 RCS5.0 Adapter

## 1. 核心架构

这个包就像一个“翻译官”，它把复杂的 VDA5050 标准翻译成您的机器人能听懂的 ROS2 指令。

```text
[ 外部调度系统 ] <---(MQTT/VDA5050)---> [ mqtt_bridge ] <---(内部ROS话题)---> [ vda5050_controller ] <---(ROS Action)---> [ 适配器 (如 tb3_adapter) ] <---(机器人具体驱动)---> [ 机器人硬件 ]
```

- **`mqtt_bridge`**: 负责把 MQTT 上的 VDA5050 消息搬运到 ROS2 话题上。
- **`vda5050_controller`**: 负责逻辑处理。比如：收到一个订单（Order），它会检查订单合法性，然后发指令给适配器。
- **`适配器 (Adapter)`**: 这是**您最需要关注的部分**。它接收控制器的通用指令，并将其转换成 rcs5.0 系统具体的 API 或话题。

---

### **2. 移植到 rcs5.0 的清单**

要将这个包成功移植，您需要搬运以下三个核心包：

1. **`vda5050_msgs`**: 定义了 VDA5050 标准的所有消息格式（必须有）。
2. **`vda5050_serializer`**: 负责 JSON 与 ROS 消息的互相转换（必须有）。
3. **`vda5050_connector`**: 业务逻辑核心包（必须有）。

**依赖项**:

- **Python 库**: `paho-mqtt`, `numpy<2.0`, `transforms3d`, `jsonschema`。
- **ROS2**: 建议使用 Humble 或更高级版本。

---

### **3. 如何进行针对性开发（核心修改点）**

如果您要将该包适配到 rcs5.0，以下是您必须动刀的地方：

#### **A. 全局修改话题前缀 (如把 `uagv` 改成 车型名)**

- **修改文件**: [utils.py](file:///home/yanhuan/rcs5.0_dev_ws/vda5050_test/ros_amr_interop/src/vda5050_connector/vda5050_connector_py/utils.py)
- **操作**: 将 `get_vda5050_topic_name` 和 `get_ros_topic_name` 函数中的 `interface_name="uagv"` 默认值修改为您自己的车型名。

#### **B. 统一内外版本 (如全用 `v2`)**

- **修改文件**: [utils.py](file:///home/yanhuan/rcs5.0_dev_ws/vda5050_test/ros_amr_interop/src/vda5050_connector/vda5050_connector_py/utils.py)
- **操作**: 将 `get_ros_topic_name` 函数中的 `major_version="v1"` 修改为 `"v2"`。

#### **C. 增加自定义动作 (Action)**

- **场景**: 比如 rcs5.0 有一个特殊的“开启顶升”动作。
- **修改点**: 在您的**适配器 (Adapter)** 节点中，注册一个新的 Action Handler。
- **参考**: 查看 `vda5050_tb3_adapter` 是如何处理 `NavigateToNode` 动作的。

---

### **4. 如何对接**

#### **第一步：配置参数 (最简单)**

修改 `connector.yaml` 配置文件：

- `manufacturer_name`: 改为您公司的名字。
- `serial_number`: 改为机器人的唯一编号。
- `mqtt_address`: 您的 MQTT 服务器地址。

#### **第二步：实现适配器 (最重要的工作)**

您不需要修改 `vda5050_connector` 的核心逻辑，只需要写一个新的**适配器节点**。

- **任务**: 监听来自控制器的 `NavigateToNode` 动作，并将其转发给 rcs5.0 的导航模块。
- **参考**: 直接参考 tb3_adapter.py 的实现方式。

#### **第三步：状态反馈**

您的适配器需要定期发布机器人的状态（位置、电池、错误等）到内部 ROS 话题。

- **目标话题**: `/uagv/v1/<manufacturer>/<serial>/state`。
- `mqtt_bridge` 会自动检测到这个话题，并把它推送到 MQTT 给调度系统。

---

### **5. 需要注意**

1. **不要去改 `vda5050_controller.py`**: 除非您发现了严重的逻辑漏洞，否则这个文件的通用逻辑足以覆盖 90% 的场景。
2. **善用 `utils.py`**: 所有的消息转换函数都在这里，直接调用 `vda2ros` 或 `ros2vda` 即可，不要自己手动拼 JSON。
3. **日志是好朋友**: 如果机器人不动，先看 `connector_tb3` 的控制台输出，那里会清晰地告诉你是因为“找不到 Action 服务器”还是“订单解析失败”。

通过这种方式，您只需要专注于编写一个“适配器”来对接 rcs5.0 的 API，剩下的通信、协议解析和订单管理都可以交给这个包来处理。

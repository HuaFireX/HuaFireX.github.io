# ROS2 快速入门

本教程以**实现一个完整功能**为目标，在动手过程中学会 ROS2。

**最终目标**：让小海龟自动走正方形路径。

---

## 目标1：让小海龟动起来

### 任务

让小海龟从起点移动到指定位置。

### 步骤

```bash
# 终端 1：启动海龟模拟器
ros2 run turtlesim turtlesim_node

# 终端 2：启动键盘控制
ros2 run turtlesim turtle_teleop_key
```

按键盘方向键控制海龟移动。

> **学会**：ROS2 节点的概念 - `ros2 run` 启动的就是节点

---

## 目标2：用程序控制海龟

### 任务

不按键盘，让程序控制海龟移动。

### 步骤

```bash
# 用命令发布移动指令
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}}"
```

海龟动起来了！

> **学会**：话题 (Topic) - 发布者/订阅者模式
> - `/turtle1/cmd_vel` 是话题名
> - `geometry_msgs/msg/Twist` 是消息类型

### 知识扩展

```bash
# 查看所有话题
ros2 topic list

# 查看话题类型
ros2 topic info /turtle1/cmd_vel

# 实时查看话题数据
ros2 topic echo /turtle1/cmd_vel
```

---

## 目标3：让海龟走正方形

### 任务

让海龟自动走一个正方形并回到起点。

### 分析

走正方形需要：
1. 前进 → 转90度 → 前进 → 转90度 → 前进 → 转90度 → 前进 → 转90度
2. 每个动作执行完要等待完成

### 步骤1：创建工作空间

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone https://github.com/ros2/examples.git -b humble
rosdep install --from-paths src -y
colcon build
source install/setup.bash
```

> **学会**：工作空间概念 - `colcon build` 编译，`source setup.bash` 加载

### 步骤2：创建控制节点

```bash
cd ~/ros2_ws/src
ros2 pkg create square_pkg --dependencies rclpy geometry_msgs std_msgs
```

创建 `square_pkg/square.py`：

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SquareNode(Node):
    def __init__(self):
        super().__init__('square_node')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
    def move_forward(self, distance=2.0):
        msg = Twist()
        msg.linear.x = 1.0
        self.publisher.publish(msg)
        time.sleep(distance)
        self.stop()
        
    def rotate(self, angle=1.57):  # 约90度
        msg = Twist()
        msg.angular.z = 1.0
        self.publisher.publish(msg)
        time.sleep(angle)
        self.stop()
        
    def stop(self):
        msg = Twist()
        self.publisher.publish(msg)
        time.sleep(0.5)
        
    def run(self):
        for _ in range(4):
            self.move_forward(2.0)  # 前进
            self.rotate(1.57)       # 转90度
        self.get_logger().info('完成正方形！')

def main(args=None):
    rclpy.init(args=args)
    node = SquareNode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

配置 `setup.py` 添加入口点，编译运行：

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 run square_pkg square
```

> **学会**：Python 节点编程 - 发布消息到话题

---

## 目标4：用服务控制海龟

### 任务

通过服务调用让海龟移动到指定位置。

### 知识

之前用话题发命令，像"广播"。服务像"打电话"，对方必须回复。

### 步骤

```bash
# 查看已有服务
ros2 service list

# 调用服务重置海龟位置
ros2 service call /reset std_srvs/srv/Empty

# 生成新海龟
ros2 service call /spawn turtlesim/srv/Spawn "{x: 3.0, y: 3.0, name: 'turtle2'}"
```

> **学会**：服务 (Service) - 请求-响应模式

---

## 目标5：用动作完成复杂任务

### 任务

让海龟转圈，同时可以随时取消。

### 知识

动作适合长时间任务，可以取消并有进度反馈。

### 步骤

```bash
# 发送旋转动作（带反馈）
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 3.14}" --feedback

# 取消动作（在另一个终端）
ros2 action cancel /turtle1/rotate_absolute
```

> **学会**：动作 (Action) - 可取消、有反馈的异步任务

---

## 目标6：用参数调整行为

### 任务

不修改代码，通过参数调整海龟移动速度。

### 步骤

```bash
# 查看节点参数
ros2 param list /turtlesim

# 获取当前参数
ros2 param get /turtlesim background_b

# 修改背景色（立即生效）
ros2 param set /turtlesim background_b 255
```

> **学会**：参数 (Parameter) - 运行时配置

---

## 目标7：可视化观测

### 任务

用工具可视化看到节点、话题的连接关系。

### 步骤

```bash
# 启动节点关系图
ros2 run rqt_graph rqt_graph
```

可以看到节点之间通过话题连接的可视化图。

> **学会**：rqt_graph 工具 - 可视化调试

---

## 目标8：录制和回放

### 任务

录制海龟运动轨迹，然后回放。

### 步骤

```bash
# 录制话题数据
ros2 bag record /turtle1/cmd_vel /turtle1/pose -o my_bag

# 移动海龟一段时间后停止录制 (Ctrl+C)

# 回放
ros2 bag play my_bag
```

> **学会**：ros2 bag - 数据录制与回放

---

## 综合练习：自动巡检机器人

### 任务描述

实现一个简单的自动巡检机器人，通过参数控制实现：

1. **多种轨迹模式**：正方形、圆形轨迹可选
2. **参数化配置**：通过命令行参数动态设置模式和尺寸
3. **精确控制**：使用位置反馈实现闭环控制，确保精确到达目标位置
4. **实时日志**：输出运行状态和位置信息

### 涉及知识点

| 功能 | ROS2 概念 |
|------|----------|
| 速度控制 | Topic (cmd_vel 发布者) |
| 位置反馈 | Topic (pose 订阅者) |
| 参数配置 | Parameter (declare_parameter) |
| 闭环控制 | P控制算法 |
| 日志输出 | ROS2 Logger |

### 实现效果

- 运行 `ros2 run simple_patrol run` → 海龟走正方形
- 运行 `ros2 run simple_patrol run --ros-args -p mode:=circle -p size:=1.5` → 海龟走圆形

---

### 参考代码（已验证可运行）

#### 1. 创建包

```bash
cd ~/ros2_ws/src
ros2 pkg create simple_patrol --dependencies rclpy geometry_msgs --build-type ament_python
```

#### 2. 创建节点文件

创建 `simple_patrol/simple_patrol/simple_patrol.py`：

```python
"""
简单巡检机器人节点

功能：
  - 支持正方形和圆形轨迹
  - 通过参数动态配置模式和尺寸
  - 使用位置反馈实现精确控制

用法：
  ros2 run simple_patrol run --ros-args -p mode:=square -p size:=2.0
  ros2 run simple_patrol run --ros-args -p mode:=circle -p size:=1.5
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time


class SimplePatrol(Node):
    """巡检机器人节点"""
    
    def __init__(self):
        super().__init__('simple_patrol')
        
        # 参数声明
        # mode: 运行模式，'square'(正方形) 或 'circle'(圆形)
        # size: 尺寸，正方形为边长，圆形为半径
        self.declare_parameter('mode', 'square')
        self.declare_parameter('size', 2.0)
        
        self.mode = self.get_parameter('mode').value
        self.size = self.get_parameter('size').value
        
        # 发布者：发送速度指令控制海龟
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # 订阅者：获取海龟当前位置
        self.current_pose = None
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        # 等待获取第一个位置消息
        while self.current_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        self.get_logger().info(f'初始位置: x={self.current_pose.x:.2f}, y={self.current_pose.y:.2f}')
        self.get_logger().info(f'模式: {self.mode}, 尺寸: {self.size}')
        
    def pose_callback(self, msg):
        """位置话题回调函数"""
        self.current_pose = msg
        
    def run_circle(self, radius):
        """
        走圆形轨迹
        
        原理：同时发送线速度和角速度
        v = r * ω (线速度 = 半径 * 角速度)
        """
        self.get_logger().info(f'开始走圆形, 半径={radius}米')
        
        msg = Twist()
        linear_speed = 0.5
        angular_speed = linear_speed / radius
        
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed
        
        # 转一圈所需时间: T = 2π / ω
        duration = 2 * math.pi / angular_speed
        start_time = self.get_clock().now()
        
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            
        self.stop()
        self.get_logger().info('圆形完成!')
        
    def run_square(self, size):
        """
        走正方形轨迹
        
        原理：前进 -> 左转90度 -> 前进 -> 左转90度 -> ... (重复4次)
        """
        self.get_logger().info(f'开始走正方形, 边长={size}米')
        
        for i in range(4):
            self.get_logger().info(f'第 {i+1} 边: 前进 {size}米')
            self.move_forward(size)
            
            self.get_logger().info(f'第 {i+1} 边: 左转90度')
            self.rotate(90)
            
        self.stop()
        self.get_logger().info('正方形完成!')
        
    def move_forward(self, distance):
        """
        沿当前方向前进指定距离（位置闭环控制）
        
        原理：订阅位置话题，根据当前朝向判断目标位置，达到后停止
        """
        # 根据当前朝向确定目标坐标
        if abs(math.cos(self.current_pose.theta)) > abs(math.sin(self.current_pose.theta)):
            # 沿X轴移动
            target = self.current_pose.x + distance if math.cos(self.current_pose.theta) > 0 else self.current_pose.x - distance
            axis = 'x'
        else:
            # 沿Y轴移动
            target = self.current_pose.y + distance if math.sin(self.current_pose.theta) > 0 else self.current_pose.y - distance
            axis = 'y'
        
        msg = Twist()
        msg.linear.x = 1.0
        
        while True:
            rclpy.spin_once(self, timeout_sec=0.05)
            
            # 检查是否到达目标位置（考虑方向）
            if axis == 'x':
                reached = (math.cos(self.current_pose.theta) > 0 and self.current_pose.x >= target) or \
                          (math.cos(self.current_pose.theta) < 0 and self.current_pose.x <= target)
            else:
                reached = (math.sin(self.current_pose.theta) > 0 and self.current_pose.y >= target) or \
                          (math.sin(self.current_pose.theta) < 0 and self.current_pose.y <= target)
            
            if reached:
                break
                        
            self.cmd_vel_pub.publish(msg)
            
    def rotate(self, degrees):
        """
        旋转指定角度（角度闭环控制）
        
        原理：订阅位置话题获取当前角度，计算与目标的差值
        使用P控制：差值越大转速越快，接近目标时自动减速
        """
        target_angle = self.current_pose.theta + math.radians(degrees)
        target_angle = math.atan2(math.sin(target_angle), math.cos(target_angle))
        
        msg = Twist()
            
        while True:
            rclpy.spin_once(self, timeout_sec=0.05)
            
            diff = target_angle - self.current_pose.theta
            diff = math.atan2(math.sin(diff), math.cos(diff))
            
            # 误差小于阈值认为到达目标
            if abs(diff) < 0.02:
                break
                
            # P控制：限制在 0.2~1.0 rad/s 之间
            msg.angular.z = max(0.2, min(1.0, abs(diff)))
            msg.angular.z = msg.angular.z if diff > 0 else -msg.angular.z
                
            self.cmd_vel_pub.publish(msg)
            
    def stop(self):
        """停止运动"""
        self.cmd_vel_pub.publish(Twist())
        time.sleep(0.3)
        
    def run(self):
        """根据参数选择运行模式"""
        if self.mode == 'circle':
            self.run_circle(self.size)
        else:
            self.run_square(self.size)


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = SimplePatrol()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### 3. 运行命令

```bash
# 编译
cd ~/ros2_ws
colcon build --packages-select simple_patrol
source install/setup.bash

# 先启动海龟模拟器
ros2 run turtlesim turtlesim_node

# 走正方形（默认，边长2米）
ros2 run simple_patrol run

# 走正方形，边长3米
ros2 run simple_patrol run --ros-args -p mode:=square -p size:=3.0

# 走圆形，半径1.5米
ros2 run simple_patrol run --ros-args -p mode:=circle -p size:=1.5
```

---

### 进阶

- 学习 Navigation2 实现真实机器人导航
- 学习 URDF 建模
- 学习 TF 坐标变换

---

## 常用命令速查

```bash
# 运行节点
ros2 run <包> <节点>

# 话题操作
ros2 topic list/echo/pub/info

# 服务操作
ros2 service list/call/type

# 动作操作
ros2 action list/send_goal/cancel

# 参数操作
ros2 param list/get/set/dump

# 调试工具
ros2 doctor          # 检查系统健康
rqt_graph          # 可视化节点图
ros2 bag           # 录制回放
```

---

## 常见问题

| 问题 | 解决方法 |
|------|----------|
| 海龟不动 | 检查是否 source 了环境 |
| 消息发不出去 | 确认话题名正确，用 `ros2 topic list` 查看 |
| 节点找不到 | 确认包是否编译，是否 source 了工作空间 |
| 编译报错 | 检查 Python 缩进，查看依赖是否完整 |

---

## 下一步

- [Navigation2](/ROS2/Navigation2) - 真实机器人导航
- 学习 C++ 编写节点
- 学习 Launch 文件
- 学习 URDF 机器人建模

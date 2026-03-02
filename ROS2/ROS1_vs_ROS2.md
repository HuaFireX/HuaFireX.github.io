# ROS1 与 ROS2 区别

本文档详细介绍 ROS1 和 ROS2 在使用层面的区别。

---

## 1. 架构区别

| 对比项     | ROS1                 | ROS2                  |
| ---------- | -------------------- | --------------------- |
| 通信中间件 | ROS Master（集中式） | DDS（去中心化）       |
| 实时支持   | 无                   | 支持（RTOS）          |
| 安全性     | 无                   | DDS安全（加密、认证） |
| 启动命令   | `roscore`          | 无需 `roscore`      |

### ROS1 Master 架构

ROS1 使用 Master 节点管理所有节点通信，节点需要先向 Master 注册才能通信。

### ROS2 DDS 架构

ROS2 使用 DDS（Data Distribution Service）分布式架构，无需 Master，节点直接通过 DDS 发现和通信。

![ROS1架构](https://i-blog.csdnimg.cn/direct/064626ffe4b04fbe931fcf6cf9047cdc.png)

---

## 2. 目录结构

| 目录         | ROS1                 | ROS2             |
| ------------ | -------------------- | ---------------- |
| `src/`     | 源代码目录           | 源代码目录       |
| `build/`   | 编译输出目录         | 编译输出目录     |
| `install/` | 安装后的文件目录     | 安装后的文件目录 |
| `devel/`   | 开发环境相关文件目录 | 无               |
| `log/`     | 无                   | 编译时的日志目录 |

![ROS2架构](https://i-blog.csdnimg.cn/direct/b8c9e80c31b4432a8ca25bb8b5726ff2.png)

---

## 3. 工作空间命令

### ROS1

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make
source devel/setup.sh
```

### ROS2

```bash
mkdir -p ~/dev_ws/src
cd ~/dev_ws
colcon build
source install/local_setup.sh
```

---

## 4. 功能包命令

| 操作           | ROS1                                | ROS2                                                               |
| -------------- | ----------------------------------- | ------------------------------------------------------------------ |
| 创建包         | `catkin_create_pkg <name> <deps>` | `ros2 pkg create --build-type [ament_cmake\|ament_python] <name>` |
| 列出包         | `rospack list`                    | `ros2 pkg list`                                                  |
| 列出可执行文件 | `rospack find <pkg>`              | `ros2 pkg executables`                                           |
| 查看包路径     | `rospack find <pkg>`              | `ros2 pkg prefix <pkg>`                                          |

---

## 5. 节点命令

| 操作         | ROS1                    | ROS2                           |
| ------------ | ----------------------- | ------------------------------ |
| 列出节点     | `rosnode list`        | `ros2 node list`             |
| 查看节点信息 | `rosnode info <node>` | `ros2 node info <node_name>` |

---

## 6. 话题命令

| 操作         | ROS1                                     | ROS2                                       |
| ------------ | ---------------------------------------- | ------------------------------------------ |
| 列出话题     | `rostopic list`                        | `ros2 topic list`                        |
| 打印话题数据 | `rostopic echo <topic>`                | `ros2 topic echo <topic>`                |
| 发布话题     | `rostopic pub <topic> <type> <values>` | `ros2 topic pub <topic> <type> <values>` |
| 查看发布频率 | `rostopic hz <topic>`                  | `ros2 topic hz <topic>`                  |
| 查看话题类型 | `rostopic type <topic>`                | `ros2 topic type <topic>`                |
| 查看带宽     | `rostopic bw <topic>`                  | `ros2 topic bw <topic>`                  |

---

## 7. 服务命令

| 操作         | ROS1                                          | ROS2                                            |
| ------------ | --------------------------------------------- | ----------------------------------------------- |
| 列出服务     | `rosservice list`                           | `ros2 service list`                           |
| 查看服务类型 | `rosservice type <service>`                 | `ros2 service type <service>`                 |
| 查找服务     | `rosservice find <type>`                    | `ros2 service find <type>`                    |
| 调用服务     | `rosservice call <service> <type> <values>` | `ros2 service call <service> <type> <values>` |

---

## 8. 动作命令

ROS1 没有内置动作命令，ROS2 支持：

```bash
# 列出动作
ros2 action list

# 查看动作信息
ros2 action info <action_name>

# 发送动作目标
ros2 action send_goal <action_name> <action_type> <goal>
```

---

## 9. 参数命令

| 操作     | ROS1                         | ROS2                                      |
| -------- | ---------------------------- | ----------------------------------------- |
| 删除参数 | `rosparam delete /name`    | `ros2 param delete /node_name /name`    |
| 保存参数 | `rosparam dump file`       | `ros2 param dump /node_name`            |
| 获取参数 | `rosparam get /name`       | `ros2 param get /node_name /name`       |
| 列出参数 | `rosparam list`            | `ros2 param list [node_name]`           |
| 设置参数 | `rosparam set /name value` | `ros2 param set /node_name /name value` |

> 注意：ROS2 的参数存储在节点中，ROS1 的参数存储在 ROS Master 中。

---

## 10. 录包播包

| 操作     | ROS1                       | ROS2                         |
| -------- | -------------------------- | ---------------------------- |
| 录制     | `rosbag record <topics>` | `ros2 bag record <topics>` |
| 播放     | `rosbag play <bag>`      | `ros2 bag play <bag>`      |
| 查看信息 | `rosbag info <bag>`      | `ros2 bag info <bag>`      |

---

## 11. rqt 工具

| 工具     | ROS1                                       | ROS2                                 |
| -------- | ------------------------------------------ | ------------------------------------ |
| 节点图   | `rqt_graph`                              | `rqt_graph`                        |
| 日志查看 | `rqt_console`                            | `ros2 run rqt_console rqt_console` |
| 参数配置 | `rosrun rqt_reconfigure rqt_reconfigure` | `rqt`                              |
| 二维绘图 | `rqt_plot`                               | `rqt`                              |

---

## 12. rviz

| ROS1                 | ROS2                     |
| -------------------- | ------------------------ |
| `rosrun rviz rviz` | `ros2 run rviz2 rviz2` |

---

## 13. 接口命令

ROS2 将 ROS1 的 `rosmsg` 和 `rossrv` 合并为 `ros2 interface`：

```bash
# 查看所有接口
ros2 interface list

# 查看接口定义
ros2 interface show std_msgs/msg/String
ros2 interface show std_srvs/srv/Empty
```

---

## 14. Launch 文件

### ROS1

使用 XML 格式：

```xml
<launch>
  <node name="turtlesim" pkg="turtlesim" type="turtlesim_node"/>
</launch>
```

### ROS2

使用 Python 脚本：

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="sim",
        )
    ])
```

运行：

```bash
ros2 launch <package> <launch_file.py>
```

---

## 15. ROS2 命令树

```
ros2
├── action
│   ├── cancel
│   ├── send_goal
│   └── show
├── bag
│   ├── compress
│   ├── decompress
│   ├── info
│   ├── play
│   ├── record
│   └── replay
├── component
│   ├── kill
│   ├── list
│   ├── load
│   └── restart
├── daemon
│   ├── start
│   ├── stop
│   └── status
├── doctor
│   ├── check
│   └── info
├── interface
│   ├── list
│   └── show
├── lifecycle
│   ├── change_state
│   ├── get_state
│   └── set_state
├── logging
│   ├── get_logger_level
│   ├── set_level
│   └── set_logger_level
├── msg
│   └── show
├── node
│   ├── cleanup
│   ├── info
│   ├── kill
│   ├── list
│   ├── log
│   └── wait
├── param
│   ├── delete
│   ├── describe
│   ├── dump
│   ├── get
│   ├── list
│   ├── load
│   └── set
├── pkg
│   ├── create
│   ├── describe
│   ├── executables
│   ├── list
│   └── prefix
├── run
│   ├── launch
│   └── python
├── service
│   ├── call
│   ├── find
│   ├── info
│   ├── list
│   └── type
├── srv
│   └── show
├── topic
│   ├── bw
│   ├── echo
│   ├── find
│   ├── hz
│   ├── info
│   ├── list
│   ├── pub
│   ├── type
│   └── relay
└── version
```

---

## 16. 常见问题

| 问题                | 说明                                       |
| ------------------- | ------------------------------------------ |
| ROS1 的 `roscore` | ROS2 不需要 `roscore`，使用 DDS 自动发现 |
| rosdep vs rosdepc   | rosdepc 是国内镜像版本，网络更稳定         |

### 环境变量查看

```bash
printenv | grep -i ROS
```

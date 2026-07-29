# 自动驾驶最小闭环系统需求文档

## 1. 目标

本项目使用 C++17 实现自动驾驶主干模块，使用 Python 实现 launch、场景、回灌和评测工具，在 ROS 2 Galactic 上构建可在普通笔记本 CPU 环境运行的最小自动驾驶闭环。

首版覆盖定位、感知、决策、状态机、规划、控制、Gazebo/CPU 仿真、录制回灌和评测，并为感知及规划控制保留插件/外部 ROS2 节点替换接口。

## 2. 范围和非目标

范围：单车、简化道路/停车场、NOA、ACC、APA、AVP、传感器噪声、超时降级、rosbag2 和离线指标。

非目标：真实车辆 CAN、高精地图、完整交通规则、多摄像头 BEV、物理级 Radar、训练大型 VLA、端到端模型直接控制车辆。

## 3. 系统分层

```text
Gazebo或CPU仿真 → 统一传感器接口 → 定位/感知 → 状态机 → 规划/控制 → 仿真车辆
                                      └──────── 录制/回灌/评测 ────────┘
```

Gazebo 是首选仿真后端；未安装 Gazebo 或图形渲染不可用时，CPU simulator 提供同样的 ROS2 业务接口。

## 4. 包和职责

| 包 | 职责 |
|---|---|
| `ad_msgs` | 自定义消息、状态和控制数据契约 |
| `ad_common` | 参数、坐标、时间和公共工具 |
| `ad_gazebo_sim` | Gazebo world/model、CPU 传感器与真值替身 |
| `ad_localization` | 真值定位、噪声定位和简化滤波 |
| `ad_perception` | 传统对象感知、Radar 目标和可替换接口 |
| `ad_behavior` | NOA/ACC/APA/AVP 状态机 |
| `ad_planning_control` | 参考线规划、停车轨迹、PID 控制 |
| `ad_pipeline` | 统一 launch 和参数 |
| `ad_tools` | rosbag 和评测脚本 |

## 5. ROS2 接口

### 5.1 传感器与真值

```text
/sim/camera/front/image       sensor_msgs/Image
/sim/lidar/points             sensor_msgs/PointCloud2
/sim/radar/targets            ad_msgs/DetectedObjectArray
/sim/ground_truth/objects     ad_msgs/DetectedObjectArray
/sim/vehicle_state            ad_msgs/VehicleState
```

### 5.2 自动驾驶主干

```text
/localization/ego_pose        ad_msgs/EgoPose
/localization/vehicle_state   ad_msgs/VehicleState
/perception/objects           ad_msgs/DetectedObjectArray
/behavior/state               ad_msgs/BehaviorState
/planning/trajectory          ad_msgs/Trajectory
/control/command              ad_msgs/ControlCommand
/sim/vehicle/control           ad_msgs/ControlCommand
```

所有时间戳使用 ROS time；仿真运行时支持 `/clock`，CPU simulator 默认使用节点时钟。

## 6. 功能需求

### FR-LOC

定位至少支持真值模式和带噪声模式，输出位置、航向、速度、有效性和时间戳。

### FR-PER

首版使用真值投影/结构化目标作为传统感知基线；后续可接相机检测模型。LiDAR/Radar 目标需包含类别、位置、速度、尺寸、置信度和有效性。

### FR-SM

状态包含 `INIT`、`STANDBY`、`NOA`、`ACC`、`APA`、`AVP`、`EMERGENCY`、`FINISHED`。传感器超时、定位失效、碰撞风险必须进入 `EMERGENCY` 并输出零速度控制。

### FR-PLAN

NOA 沿简化参考线行驶；ACC 使用前车距离和相对速度跟驰；APA 跟踪停车位目标；AVP 跟踪停车场预定义路点。

### FR-PLUGIN

感知和规划控制必须使用稳定的 ROS2 输出消息作为边界。首版提供传统内置实现；后续实现可用独立 ROS2 node 替换，不改变下游模块。

### FR-DATA

支持记录传感器、真值、状态、轨迹和控制话题；支持不启动仿真的 rosbag2 回放评测；输出 JSON/CSV 指标。

## 7. 场景

| 场景 | 成功条件 |
|---|---|
| NOA | 在规定时间内沿参考线到达终点，无碰撞 |
| ACC | 前车减速时保持安全距离，无碰撞 |
| APA | 自车进入停车位，位置和航向误差低于阈值 |
| AVP | 沿停车场路点到达目标位，无碰撞 |

## 8. 性能和资源

目标硬件为 16 GB 内存笔记本、CPU-only。首版允许低于实时速度；CPU simulator 必须能稳定运行，Gazebo 目标为低画质单车单场景。相机默认 320×180/5 Hz，LiDAR 默认 8～16 线/5～10 Hz。

## 9. 测试验收

必须覆盖消息契约、节点启动、传感器发布、状态转移、紧急停车、规划控制、四个场景、rosbag 回放和指标生成。最小验收命令：

```bash
colcon test --event-handlers console_direct+
ros2 launch ad_pipeline cpu_demo.launch.py scenario:=acc
python3 scripts/evaluate_run.py --scenario acc
```


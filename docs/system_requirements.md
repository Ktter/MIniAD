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

Gazebo 是首选仿真后端；未安装 Gazebo 或图形渲染不可用时，CPU simulator 提供同样的业务接口。Gazebo 控制使用 Ackermann plugin，控制适配器将 `ad_msgs/ControlCommand` 转换为 `geometry_msgs/Twist`；状态适配器将 Gazebo odometry 转换为 `ad_msgs/VehicleState`。

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
/sim/camera/front/image_raw   sensor_msgs/Image
/sim/camera/front/camera_info sensor_msgs/CameraInfo
/sim/lidar/points             sensor_msgs/PointCloud2
/sim/radar/targets            ad_msgs/DetectedObjectArray
/sim/ground_truth/objects     ad_msgs/DetectedObjectArray
/sim/vehicle/odom             nav_msgs/Odometry
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

采用分层状态机，分别管理系统生命周期、行车域、泊车域和行泊切换。`STANDBY` 只表示系统级待机，不表示“行车待机”或“泊车待机”。

系统状态机负责生命周期、安全和控制权；行车域负责 `ACC/NOA`；泊车域负责 `APA/AVP`；行泊切换状态机负责停车、控制权交接和目标域条件检查。传感器超时、定位失效、碰撞风险、轨迹无效或车辆状态异常必须进入系统级 `EMERGENCY`。

本版本采用简化的功能域状态：功能域只保留 `IDLE`、功能 `STANDBY`、功能 `ACTIVE` 和域任务完成状态。停车位搜索、轨迹规划和详细切换步骤作为状态内部处理，不单独暴露为功能域状态。

状态跳转采用事件驱动模型。ROS 输入或内部事实先转换为事件，事件更新状态机上下文，再由 Guard 判断条件，最后执行状态转移和 Entry/Exit Action：

```text
ROS 输入/内部事实
    → Event
    → 更新 Context
    → Guard 条件判断
    → 状态转移
    → Entry/Exit Action
    → 发布 BehaviorState
```

状态机是唯一允许决定状态跳转的组件。事件只描述外部请求、数据变化或事实，不直接指定目标状态。

#### FR-SM-0 事件模型、上下文和 Guard

外部命令和内部事实统一通过 `/behavior/event` 进入状态机。事件字段使用 `uint8` 枚举，不使用字符串状态名。事件至少包含：

```text
std_msgs/Header header
uint8 source               # 事件来源
uint8 type                 # 事件类型
uint8 requested_function   # NONE/ACC/NOA/APA/AVP
bool enable
bool reset_emergency
```

事件来源包括：

```text
COMMAND / VEHICLE / LOCALIZATION / PERCEPTION / PLANNING / SYSTEM / TIMER
```

事件类型包括：

```text
REQUEST_FUNCTION
CANCEL_FUNCTION
RESET_EMERGENCY
VEHICLE_STATE_UPDATED
VEHICLE_STOPPED
VEHICLE_MOVING
VEHICLE_INVALID
LOCALIZATION_VALID
LOCALIZATION_INVALID
SENSOR_TIMEOUT
PARKING_TARGET_VALID
PARKING_TARGET_INVALID
TRAJECTORY_VALID
TRAJECTORY_INVALID
TASK_SUCCESS
TASK_FAILED
SWITCH_TIMEOUT
```

高频传感器话题不直接触发大量状态转移。传感器回调先更新上下文，只有状态变化、超时、边沿变化或稳定窗口满足时，才生成离散事件。

状态机上下文至少包含：

```text
vehicle_valid
vehicle_speed
vehicle_stopped
localization_valid
perception_valid
parking_target_valid
parking_route_valid
trajectory_valid
active_function
requested_function
last_event_time
emergency_reset_requested
```

Guard 由状态机统一执行，不由传感器节点或命令发布者提前判断。典型 Guard 包括：

```text
CanEnterAccActive()
CanEnterNoaActive()
CanEnterApaActive()
CanEnterAvpActive()
CanStartSwitching()
IsVehicleStopped()
CanRecoverEmergency()
```

车辆停止、传感器有效和目标有效等条件支持参数化确认窗口：

```text
vehicle_stop_speed_threshold
condition_confirmation_window
sensor_timeout
switch_timeout
```

#### FR-SM-1 状态定义

##### 1. 系统状态机

| 状态 | 说明 | `autonomous_enabled` | 主要输出行为 |
|---|---|---:|---|
| `INIT` | 节点启动，等待车辆、定位、感知和控制接口有效 | `false` | 不输出自动驾驶功能控制 |
| `STANDBY` | 系统健康且没有运行中的自动驾驶功能 | `false` | 保持停车，等待功能请求 |
| `SWITCHING` | 行泊切换流程正在执行 | `false` | 由切换状态机负责停车和控制权交接 |
| `ACTIVE` | 行车域或泊车域已激活；具体是否执行由域状态决定 | 按域状态 | 域状态为 `*_ACTIVE` 时输出规划和控制结果 |
| `FINISHED` | 当前功能任务成功完成 | `false` | 停止自动驾驶任务，等待新请求 |
| `EMERGENCY` | 传感器、定位、规划或安全异常 | `false` | 输出急停，目标速度为零 |

`ACC`、`NOA`、`APA`、`AVP` 不再作为系统级状态，而是功能域中的功能标识。

##### 2. 行车域状态机

```text
IDLE → ACC_STANDBY → ACC_ACTIVE → PILOT_FINISHED
  └→ NOA_STANDBY → NOA_ACTIVE ───────────────┘
```

`IDLE` 表示行车域未选择具体功能；`ACC_STANDBY/NOA_STANDBY` 表示功能已选择但尚未满足执行条件；`ACC_ACTIVE/NOA_ACTIVE` 表示正在执行；`PILOT_FINISHED` 表示行车任务成功完成。

行车域状态定义：

| 状态 | 状态职责 | 进入条件 | 正常出口 | 取消/异常处理 |
|---|---|---|---|---|
| `IDLE` | 行车域已激活，但尚未选择 ACC 或 NOA | 系统激活行车域，或行车任务复位 | 收到 `ACC` 请求进入 `ACC_STANDBY`；收到 `NOA` 请求进入 `NOA_STANDBY` | 取消行车域回系统 `STANDBY` |
| `ACC_STANDBY` | ACC 已选择，等待车辆、定位、前向目标和安全条件满足 | 请求功能为 `ACC` | 条件满足进入 `ACC_ACTIVE` | 条件失效回 `IDLE`；跨域请求进入 `SWITCHING` |
| `ACC_ACTIVE` | 基于前车目标进行跟驰控制 | ACC 条件持续满足 | 任务完成进入 `PILOT_FINISHED` | 取消或跨域请求先停车；安全异常进入 `EMERGENCY` |
| `NOA_STANDBY` | NOA 已选择，等待定位、参考线和感知条件满足 | 请求功能为 `NOA` | 条件满足进入 `NOA_ACTIVE` | 条件失效回 `IDLE`；跨域请求进入 `SWITCHING` |
| `NOA_ACTIVE` | 执行参考线跟踪和障碍物处理 | NOA 条件持续满足 | 到达终点进入 `PILOT_FINISHED` | 取消或跨域请求先停车；安全异常进入 `EMERGENCY` |
| `PILOT_FINISHED` | 行车任务成功结束，释放行车控制权 | 收到任务成功结果 | 新行车任务回 `IDLE`；系统任务结束回 `FINISHED` | 新泊车请求进入 `SWITCHING` |

##### 3. 泊车域状态机

```text
IDLE → APA_STANDBY → APA_ACTIVE → PARKING_FINISHED
  └→ AVP_STANDBY → AVP_ACTIVE ────────────────┘
```

`APA_STANDBY/AVP_STANDBY` 内部完成停车位、路点、路线和轨迹准备；`APA_ACTIVE/AVP_ACTIVE` 表示正在执行泊车；`PARKING_FINISHED` 表示泊车任务成功完成。

泊车域状态定义：

| 状态 | 状态职责 | 进入条件 | 正常出口 | 取消/异常处理 |
|---|---|---|---|---|
| `IDLE` | 泊车域已激活，但尚未选择 APA 或 AVP | 系统激活泊车域，且车辆已停止 | 收到 `APA` 请求进入 `APA_STANDBY`；收到 `AVP` 请求进入 `AVP_STANDBY` | 取消泊车域回系统 `STANDBY` |
| `APA_STANDBY` | APA 已选择，内部搜索车位并准备泊车轨迹 | 请求功能为 `APA`，车辆已停止 | 车位和轨迹有效进入 `APA_ACTIVE` | 条件失效回 `IDLE`；跨域请求进入 `SWITCHING` |
| `APA_ACTIVE` | 执行低速自动泊车 | APA 轨迹有效且控制权已交接 | 入位成功进入 `PARKING_FINISHED` | 取消或跨域请求先停车；安全异常进入 `EMERGENCY` |
| `AVP_STANDBY` | AVP 已选择，内部校验目标路点、路线并准备轨迹 | 请求功能为 `AVP`，车辆已停止 | 路点、路线和轨迹有效进入 `AVP_ACTIVE` | 条件失效回 `IDLE`；跨域请求进入 `SWITCHING` |
| `AVP_ACTIVE` | 执行低速路径跟踪和动态避障 | AVP 轨迹有效且控制权已交接 | 到达目标位进入 `PARKING_FINISHED` | 取消或跨域请求先停车；安全异常进入 `EMERGENCY` |
| `PARKING_FINISHED` | 泊车任务成功结束，释放泊车控制权 | 收到任务成功结果 | 新泊车任务回 `IDLE`；系统任务结束回 `FINISHED` | 新行车请求进入 `SWITCHING` |

##### 4. 行泊切换状态机

```text
SWITCH_IDLE
  → REQUEST_ACCEPTED
  → SOURCE_STOPPING
  → VEHICLE_STOPPED
  → CONTROL_HANDOVER
  → TARGET_CHECKING
  → SWITCH_COMMITTED
```

切换失败进入 `SWITCH_FAILED`，车辆保持停止；可安全恢复时回系统 `STANDBY`，存在安全风险时进入 `EMERGENCY`。切换未提交前，目标域不得输出车辆控制命令。`SWITCH_IDLE` 到 `SWITCH_COMMITTED` 的步骤属于切换控制器内部状态，不作为系统对外状态枚举。

切换流程由事件推进：

```text
REQUEST_FUNCTION(目标域)
  → REQUEST_ACCEPTED
  → SOURCE_STOPPING
  → VEHICLE_STOPPED
  → CONTROL_HANDOVER
  → TARGET_CHECKING
  → SWITCH_COMMITTED
```

对应事件依次为：

```text
跨域请求事件
车辆停止事件
控制权释放事件
目标域条件满足事件
切换提交事件
```

切换期间必须满足以下约束：

- 目标域不能输出车辆控制命令。
- 车辆未停止不能进行控制权交接。
- 目标条件失败进入 `SWITCH_FAILED`。
- 切换超时进入 `EMERGENCY`，或在车辆安全停止后回 `STANDBY`。
- 同时只能存在一个活动域和一个控制权持有者。

#### FR-SM-2 状态转移图

行车和泊车之间不允许直接切换，必须经过系统 `SWITCHING`，确保车辆先停止并完成控制权交接。

```text
┌──────────────────────────────────────────────────────────────────────────────────────────────┐
│                                      系统状态机                                                │
└──────────────────────────────────────────────────────────────────────────────────────────────┘

┌──────────┐       数据稳定        ┌──────────────┐       功能请求       ┌──────────────┐
│  INIT    │ ─────────────────────▶ │   STANDBY    │ ───────────────────▶ │    ACTIVE    │
│ 初始化   │                        │ 系统待机     │                      │ 当前域执行   │
└────┬─────┘                        └──────┬───────┘                      └──┬─────┬─────┘
     │ 初始化故障                           │                                 │     │
     │                                      │ 取消且车辆停止                   │     │任务完成
     ▼                                      ▼                                 │     ▼
┌──────────────┐                                                                  ┌──────────────┐
│  EMERGENCY   │ ◀────────────── 任意状态安全异常 ────────────────────────────── │  FINISHED    │
│ 急停/零速度  │                                                                  │ 任务完成     │
└──────┬───────┘                                                                  └──────┬───────┘
       │ 故障恢复+车辆停止+人工复位                                                       │ 新任务/复位
       └──────────────────────────────▶ STANDBY ◀────────────────────────────────────────┘

跨域请求：ACTIVE ───────────────────────────────────────────────────────────────▶ SWITCHING

┌──────────────────────────────────────────────────────────────────────────────────────────────┐
│                                    行泊切换状态机                                              │
└──────────────────────────────────────────────────────────────────────────────────────────────┘

┌──────────────┐   ┌────────────────┐   ┌────────────────┐   ┌────────────────┐
│ SWITCH_IDLE  │ → │REQUEST_ACCEPTED│ → │SOURCE_STOPPING │ → │VEHICLE_STOPPED │
│ 等待切换请求 │   │ 接受目标域请求 │   │ 源域减速停车   │   │ 确认车辆静止   │
└──────────────┘   └────────────────┘   └────────────────┘   └───────┬────────┘
                                                                      ▼
                    ┌────────────────┐   ┌────────────────┐   ┌────────────────┐
                    │CONTROL_HANDOVER│ → │TARGET_CHECKING │ → │SWITCH_COMMITTED│
                    │ 释放源域控制权 │   │ 检查目标域条件 │   │ 激活目标域      │
                    └────────────────┘   └───────┬────────┘   └────────────────┘
                                                  │ 条件失败/超时
                                                  ▼
                                           ┌────────────────┐
                                           │ SWITCH_FAILED  │
                                           │ 保持车辆停止   │
                                           └────────────────┘

┌──────────────────────────────────────┐        ┌──────────────────────────────────────┐
│              行车域                  │        │              泊车域                  │
├──────────────────────────────────────┤        ├──────────────────────────────────────┤
│                                      │        │                                      │
│  ┌──────────────┐                    │        │  ┌──────────────┐                    │
│  │     IDLE     │                    │        │  │     IDLE     │                    │
│  └──────┬───────┘                    │        │  └──────┬───────┘                    │
│         ▼                            │        │         ▼                            │
│  ┌──────────────┐   ┌──────────────┐ │        │  ┌──────────────┐   ┌──────────────┐ │
│  │ACC_STANDBY  │ → │  ACC_ACTIVE  │ │        │  │APA_STANDBY  │ → │  内部准备    │ │
│  └──────────────┘   └──────┬───────┘ │        │  └──────────────┘   └──────┬───────┘ │
│                            │          │        │                              ▼       │
│  ┌──────────────┐   ┌──────▼───────┐ │        │                       ┌──────────────┐ │
│  │NOA_STANDBY  │ → │  NOA_ACTIVE  │ │        │                       │ APA_ACTIVE  │ │
│  └──────────────┘   └──────┬───────┘ │        │                       └──────────────┘ │
│                            ▼          │        │                                      │
│                   ┌────────────────┐ │        │  ┌──────────────┐   ┌──────────────┐ │
│                   │PILOT_FINISHED │ │        │  │AVP_STANDBY  │ → │  内部准备    │ │
│                   └────────────────┘ │        │  └──────────────┘   └──────┬───────┘ │
│  ACC/NOA_ACTIVE ──阻塞/取消──▶        │        │                              ▼       │
│  取消/阻塞 → IDLE                   │        │                       ┌──────────────┐ │
│  车辆安全停止                       │        │                       │ AVP_ACTIVE  │ │
│                                      │        │                       └──────────────┘ │
└──────────────────────────────────────┘        │  APA/AVP_ACTIVE ──阻塞/取消──▶       │
                                                │  取消/阻塞 → IDLE                     │
                                                │  车辆安全停止                         │
                                                └──────────────────────────────────────┘

行车域 → 泊车域：ACC/NOA_ACTIVE → SWITCHING → APA_STANDBY 或 AVP_STANDBY
泊车域 → 行车域：APA/AVP_ACTIVE → SWITCHING → ACC_STANDBY 或 NOA_STANDBY
```

#### FR-SM-3 状态转移条件

| 当前状态 | 事件 | Guard 条件 | 目标状态 | Action |
|---|---|---|---|---|
| `INIT` | 数据有效事件 | 车辆、定位、感知和控制接口持续稳定 | `STANDBY` | 清空活动功能，发布系统待机状态 |
| `STANDBY` | `REQUEST_FUNCTION` | 请求合法且无安全风险；具体执行条件由目标域 `*_STANDBY` Guard 判断 | `ACTIVE` | 初始化目标域和功能上下文 |
| `ACTIVE` | `REQUEST_FUNCTION` | 请求来自另一功能域且目标请求合法 | `SWITCHING` | 锁定目标功能，禁止目标域输出控制 |
| `ACTIVE` | `CANCEL_FUNCTION` | 当前功能已取消且车辆安全停止 | `STANDBY` | 释放当前域控制权 |
| `ACTIVE` | `TASK_SUCCESS` | 任务上下文和当前功能匹配 | `FINISHED` | 释放控制权，保存任务结果 |
| `FINISHED` | `REQUEST_FUNCTION` | 新请求合法 | `ACTIVE` | 创建新任务上下文 |
| `FINISHED` | `RESET_EMERGENCY` | 复位请求合法 | `STANDBY` | 清除完成态上下文 |
| 任意状态 | 安全异常事件 | 异常达到急停条件 | `EMERGENCY` | 急停，目标速度置零 |
| `EMERGENCY` | `RESET_EMERGENCY` | 故障恢复、车辆停止且关键数据稳定 | `STANDBY` | 清除故障上下文，不自动恢复原功能 |

行车域事件转移：

| 当前状态 | 事件 | Guard 条件 | 目标状态 |
|---|---|---|---|
| `IDLE` | `REQUEST_FUNCTION(ACC)` | 请求功能为 `ACC` | `ACC_STANDBY` |
| `ACC_STANDBY` | 车辆/定位/感知有效事件 | `CanEnterAccActive()` | `ACC_ACTIVE` |
| `ACC_ACTIVE` | `TASK_SUCCESS` | 当前任务为 ACC 且任务成功 | `DRIVING_FINISHED` |
| `IDLE` | `REQUEST_FUNCTION(NOA)` | 请求功能为 `NOA` | `NOA_STANDBY` |
| `NOA_STANDBY` | 定位/参考线/感知有效事件 | `CanEnterNoaActive()` | `NOA_ACTIVE` |
| `NOA_ACTIVE` | `TASK_SUCCESS` | 当前任务为 NOA 且任务成功 | `DRIVING_FINISHED` |
| 任意行车态 | `CANCEL_FUNCTION` | 车辆已安全停止 | `IDLE` |

泊车域事件转移：

| 当前状态 | 事件 | Guard 条件 | 目标状态 |
|---|---|---|---|
| `IDLE` | `REQUEST_FUNCTION(APA)` | 请求功能为 `APA` | `APA_STANDBY` |
| `APA_STANDBY` | 车辆停止/车位/轨迹有效事件 | `CanEnterApaActive()` | `APA_ACTIVE` |
| `APA_ACTIVE` | `TASK_SUCCESS` | 当前任务为 APA 且任务成功 | `PARKING_FINISHED` |
| `IDLE` | `REQUEST_FUNCTION(AVP)` | 请求功能为 `AVP` | `AVP_STANDBY` |
| `AVP_STANDBY` | 车辆停止/路点/路线/轨迹有效事件 | `CanEnterAvpActive()` | `AVP_ACTIVE` |
| `AVP_ACTIVE` | `TASK_SUCCESS` | 当前任务为 AVP 且任务成功 | `PARKING_FINISHED` |
| 任意泊车态 | `CANCEL_FUNCTION` | 车辆已安全停止 | `IDLE` |

#### FR-SM-4 行泊切换规则

行车到泊车：

```text
行车域 ACC/NOA_ACTIVE
  → REQUEST_ACCEPTED
  → SOURCE_STOPPING
  → VEHICLE_STOPPED
  → CONTROL_HANDOVER
  → TARGET_CHECKING（停车位或 AVP 路点有效）
  → SWITCH_COMMITTED
  → 泊车域 APA_STANDBY/AVP_STANDBY
```

泊车到行车：

```text
泊车域 APA/AVP_ACTIVE
  → REQUEST_ACCEPTED
  → SOURCE_STOPPING
  → VEHICLE_STOPPED
  → CONTROL_HANDOVER
  → TARGET_CHECKING（定位、参考线和行车数据有效）
  → SWITCH_COMMITTED
  → 行车域 ACC_STANDBY/NOA_STANDBY
```

切换期间系统状态为 `SWITCHING`，不能伪装成普通 `STANDBY`。不满足切换条件时进入 `SWITCH_FAILED`，车辆保持停止，并在 `BehaviorState.reason` 中输出稳定原因，例如：

```text
waiting_vehicle_stop
handover_in_progress
parking_target_unavailable
parking_route_unavailable
localization_invalid
sensor_timeout
trajectory_invalid
switch_timeout
invalid_mode_request
```

#### FR-SM-5 异常和恢复策略

进入 `EMERGENCY` 后必须满足以下输出约束：

```text
BehaviorState.system_state       = "EMERGENCY"
BehaviorState.active_domain      = "NONE"
BehaviorState.active_function    = "NONE"
BehaviorState.autonomous_enabled = false
ControlCommand.emergency_stop    = true
ControlCommand.target_speed      = 0.0
```

故障恢复后不自动恢复到故障前功能，必须先进入系统 `STANDBY`，再通过新的模式请求启动对应域和功能。这样可以避免传感器短暂恢复后车辆未经确认自动重新运动。

模式请求和内部事实统一通过 `/behavior/event` 传入，事件消息至少包含：

```text
std_msgs/Header header
uint8 source
uint8 type
uint8 requested_function   # NONE/ACC/NOA/APA/AVP
bool enable
bool reset_emergency
```

`/behavior/state` 建议至少包含以下字段：

```text
uint8 vehicle_state       # SystemStateMachine
uint8 function_state      # 当前活动域状态
uint8 active_domain       # NONE/DRIVING/PARKING
uint8 active_function     # NONE/ACC/NOA/APA/AVP
uint8 transition_phase    # 无切换时为 SWITCH_IDLE
uint8 requested_function
uint64 event_id
string reason
bool autonomous_enabled
```

状态优先级固定为：

```text
EMERGENCY > SWITCHING > ACTIVE > FINISHED > STANDBY > INIT
```

安全异常可以从任意状态抢占进入 `EMERGENCY`；同一时刻只能有一个活动域和一个控制权持有者；切换未提交前，目标域不得输出车辆控制命令。

事件处理规则：

- 按事件时间戳顺序处理事件。
- 同一事件重复到达必须幂等，不能重复执行 Entry Action。
- 过期事件丢弃并记录 `stale_event`。
- 非法事件不改变状态，只记录 `invalid_event`。
- 不满足 Guard 的事件不执行跳转，并记录具体失败原因。
- 安全异常事件拥有最高优先级，可以抢占普通事件。
- 一个事件最多触发一次状态转移。
- 状态发生变化后发布一次新的 `BehaviorState`。

当前 `BehaviorState.msg` 的字段映射约定为：

```text
vehicle_state  = SystemStateMachine
function_state = 当前活动域状态
```

由于行车和泊车功能枚举值存在重叠，状态消息建议补充以下字段，用于区分活动域和切换过程：

```text
uint8 active_domain
uint64 event_id
uint8 transition_phase
```

本版本先在系统需求中定义该接口要求；消息文件和 C++ 实现可在后续实现阶段同步升级。

事件驱动验收场景至少包括：

- `INIT → STANDBY`：关键数据稳定后才能转移。
- ACC 请求但条件不满足：停留在 `ACC_STANDBY`。
- ACC 条件满足：进入 `ACC_ACTIVE`。
- APA 请求但车辆未停止：停留在 `APA_STANDBY`。
- 车辆停止且车位、轨迹有效：进入 `APA_ACTIVE`。
- 行车到泊车：必须经过 `SWITCHING`。
- 切换未完成时目标域不得输出控制。
- 传感器超时：任意状态抢占到 `EMERGENCY`。
- 故障恢复但未收到复位事件：保持 `EMERGENCY`。
- 重复事件：不得重复执行状态 Entry Action。
- 过期事件和非法事件：不得改变状态。
- 任务成功事件：进入对应域 Finished 状态，再回系统 `FINISHED`。

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

# Autonomous Driving MiniStack

ROS 2 Galactic + Gazebo Classic 11 的无 GPU 自动驾驶最小闭环 demo。

当前仓库提供两条路径：

* `ad_gazebo_sim`：Gazebo 场景、车辆模型和 Gazebo 可选配置；
* `ad_gazebo_sim/cpu_simulator`：不依赖 Gazebo 的 CPU 传感器/车辆替身，便于在未安装 Gazebo 或没有 NVIDIA 驱动的笔记本上先跑通消息闭环。

## 快速运行（无需 Gazebo）

```bash
source /opt/ros/galactic/setup.bash
cd Autonomous_Driving_MiniStack
colcon build --symlink-install
source install/setup.bash
ros2 launch ad_pipeline cpu_demo.launch.py scenario:=acc
```

可选场景：`noa`、`acc`、`apa`、`avp`。查看输出：

```bash
ros2 topic list
ros2 topic echo /behavior/state
ros2 topic echo /planning/trajectory
ros2 topic echo /control/command
```

## Gazebo 路径

Gazebo Classic 11 后端可使用：

```bash
ros2 launch ad_gazebo_sim gazebo.launch.py
```

该 launch 会启动 Gazebo server、加载 Ackermann 自车，并发布前视相机、CPU LiDAR、IMU、GNSS、odometry 和统一 `VehicleState`。CPU fallback 仍通过 `cpu_demo.launch.py` 保留。

检查本机环境：

```bash
python3 scripts/check_environment.py
```

验证传感器：

```bash
ros2 topic list | grep '^/sim/'
ros2 topic echo /sim/vehicle/odom
ros2 topic echo /sim/lidar/points
ros2 topic echo /sim/imu/data
```

Gazebo 集成通过 `ControlCommand → Twist → Ackermann plugin` 和 `Odometry → VehicleState` 适配器衔接业务模块。完整需求见 [docs/system_requirements.md](docs/system_requirements.md)。

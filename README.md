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

当前环境若安装 Gazebo Classic 11，可使用：

```bash
ros2 launch ad_gazebo_sim gazebo.launch.py
```

仓库已准备 Gazebo world/model 资源目录，但当前开发环境尚未安装 `gazebo` 可执行程序，因此该 launch 暂时使用 CPU fallback。安装 Gazebo Classic 11 后，将车辆 SDF、传感器插件和 Gazebo process 接入相同 topic 即可替换。

检查本机环境：

```bash
python3 scripts/check_environment.py
```

Gazebo 集成是可选后端；业务节点只依赖 `ad_msgs` 的统一接口。完整需求见 [docs/system_requirements.md](docs/system_requirements.md)。

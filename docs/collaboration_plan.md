# 开发协作计划

## 角色

### 主开发者（用户）

负责 `ad_msgs`、定位、状态机、规划控制、插件边界、主 pipeline 和最终集成。

### 仿真 Agent

负责 Gazebo Classic 11 安装验证、车辆 SDF/URDF、道路与停车场 world、相机/LiDAR/IMU/GNSS、Radar simulator、真值发布和四类场景。

### 测试 Agent

负责消息契约测试、节点启动测试、仿真传感器 smoke test、状态机异常测试、rosbag 回放测试和功能指标。

### 工具/文档 Agent

负责环境检查、运行脚本、数据格式、README、评测报告和问题归档。

## 分支与交付约定

各 Agent 使用独立分支，提交必须包含：

1. 变更说明；
2. 运行命令；
3. 测试结果；
4. 已知限制。

主开发者只合并通过 `colcon build` 和对应 smoke test 的提交。消息和 topic 变更必须先更新 `docs/system_requirements.md`。

## 里程碑

1. M1：消息、launch 和 CPU fallback 闭环可启动；
2. M2：Gazebo 车辆、传感器和场景接入；
3. M3：NOA/ACC/APA/AVP 和录制回灌可运行；
4. M4：自动评测、异常场景和笔记本验收完成。


# MID-360 实时位姿与轨迹

在ros2容器的工程根目录使用Bash：

```sh
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch gkd_nav_bringup point_lio.launch.py
```

默认启动官方驱动、Rust Point-LIO适配节点及RViz。无图形界面追加 `rviz:=false`；已有CustomMsg驱动时追加 `start_driver:=false`，避免重复启动驱动。单独的mid360_rviz启动使用PointCloud2，不满足本适配器输入要求。

参数在 `src/gkd_nav_bringup/config/point_lio.yaml`。核心仍是纯Rust，C++只承担ROS2消息、队列与发布，通过固定布局C ABI调用Rust。IMU加速度由g转换为m/s²；点时间由纳秒偏移转换。

- `/point_lio/odometry`：位姿、机体系速度与协方差，约10Hz。
- `/point_lio/path`：2Hz轨迹，最多3000个位姿，重置时清空。
- TF：动态odom→lio_imu，静态lio_imu→livox_frame；不代表底盘base_link。
- `/point_lio/diagnostics`：处理耗时、队列、匹配数、丢帧及重置。

合成ROS验证用 `python3 tools/test_point_lio_bridge.py`；实时观测用 `python3 tools/point_lio_probe.py`。只有保持雷达和场景静止时才追加 `--stationary`，启用静态漂移阈值判断。运动观测中的相对位姿变化不表示误差。

实机记录293个位姿、约10.10Hz，293个对应动态TF匹配；处理耗时末次约18.30ms，轨迹显示已获用户确认。记录中包含实际转动，不能据此认定30秒静态漂移合格，也没有运动真值精度结论。历史文件static.*是按最初静止假设命名，实际包含运动；live.*及dynamic-scene-stationary.*对应雷达静止但场景非静止的尝试。initial-overlap.csv涉及重复驱动排查，不作为有效验收数据。

设计见[design.md](design.md)，阶段结果见[log.md](log.md)。

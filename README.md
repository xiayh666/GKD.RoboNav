# GKD.RoboNav

激光雷达 + IMU 导航工程。

- [Rust Point-LIO 核心库](src/point_lio/README.md)：原生 Rust，无 ROS
  依赖，基于作者官方模型重写，在线建图，不加载先验点云。
- [共享观测协议](src/map_updates/README.md)：源会话、位姿与按体素分组的采样观测，供两库传递增量。
- [terrain_map](src/terrain_map/README.md)：纯 Rust 地形地图库，已接入 Point-LIO 观测增量，提供独立几何视图、Tokio 调度、帧内并行和人工标注；地形与隧道分析待实现。
- [MID-360 官方驱动](doc/mid360/README.md)：Livox driver2子模块和固定SDK，发布点云与IMU。
- `src/gkd_nav_bringup`：ROS 2/Nav2参数、MID-360配置及统一启动入口，尚未与Rust里程计桥接。

在已加载 ROS 2 环境的终端中，统一编译：

```sh
colcon build
```

`point_lio` 通过普通 CMake 入口调用 Cargo，默认编译 Release 库；它没有
`package.xml`，不依赖 ament，也不是 ROS 包。Rust 产物默认位于
`build/point_lio/cargo/release/`（本工程默认配置）。只编译该库可用 `colcon build --packages-select point_lio`。

本项目实际在 `ros2` 容器内构建。首次使用需在容器内安装 Rust ≥1.85
（建议 rustup minimal stable），宿主机安装的 Rust 不会自动进入容器。
工程以 `ros2` 容器为构建环境，使用colcon标准目录 `build/`、`install/`、`log/`：

```sh
# 在容器的 /workspace/GKD.RoboNav 中使用Bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

CMake缓存包含绝对路径，不能直接在宿主机和容器之间复用。切换构建环境时应清理旧的包构建缓存和安装产物，再重新构建；不再通过ros2子目录隔离。

此时 Rust 产物位于 `build/point_lio/cargo/release/`，ROS 环境入口为
`install/setup.bash`。CMake 可自动发现容器用户 `~/.cargo/bin/cargo`。


单独使用 Cargo 和运行回归：

```sh
cargo test --workspace --release --lib --test fusion
cargo run --release -p point-lio --example replay
```

实现范围、算法差异、输入约定和真实数据验证状态见核心库 README。

2026-11-12 已修正滤波时钟、联合观测及地图流程，并增加直接编译官方
C++ 滤波核心生成的数值基准。最新[修正报告](doc/point-lio-rust/correction.md)
包含轨迹对照与仍未通过的 300 体素压力测试；尚未完成实车验收。

官方 C++ 参考源码已从项目移除；数值基准、对照轨迹和出处仍保留。
[验证资料索引](doc/point-lio-rust/README.md)包含容量对照及完整测试命令。

MID-360首次在宿主项目目录执行 `bash tools/build_livox.sh`，准备官方SDK和ROS2清单并统一构建。以后在容器里按上面的colcon命令编译即可；驱动Jazzy配置由colcon.meta提供。

容器中加载 `install/setup.bash` 后启动：

```sh
# 默认：CustomMsg点云 + IMU
ros2 launch gkd_nav_bringup mid360.launch.py
# 可视化：PointCloud2 + 官方RViz配置
ros2 launch gkd_nav_bringup mid360_rviz.launch.py
```

宿主图形终端也可运行 `bash tools/view_mid360.sh`。首次容器依赖、雷达IP及实机接收记录见[接入说明](doc/mid360/README.md)。

实时位姿与运动轨迹（默认同时打开RViz）：

```sh
ros2 launch gkd_nav_bringup point_lio.launch.py
# 无图形界面：追加 rviz:=false
```

输出 `/point_lio/odometry`、`/point_lio/path` 和 `odom → lio_imu → livox_frame` TF。
使用薄ROS2适配层通过C ABI调用Rust核心；参数和验证说明见[位姿接入说明](doc/point-lio-integration/README.md)。

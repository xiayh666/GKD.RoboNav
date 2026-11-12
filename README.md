# GKD.RoboNav

激光雷达 + IMU 导航工程。

- [Rust Point-LIO 核心库](src/point_lio/README.md)：原生 Rust，无 ROS
  依赖，基于作者官方模型重写，在线建图，不加载先验点云。
- [terrain_map](src/terrain_map/README.md)：纯 Rust 地形地图库，已实现体素更新、Tokio 调度、帧内并行和人工标注；地形与隧道分析待实现。
- `src/gkd_nav_bringup`：现有 ROS 2/Nav2 参数，尚未与 Rust 里程计桥接。

在已加载 ROS 2 环境的终端中，统一编译：

```sh
colcon build
```

`point_lio` 通过普通 CMake 入口调用 Cargo，默认编译 Release 库；它没有
`package.xml`，不依赖 ament，也不是 ROS 包。Rust 产物默认位于
`build/point_lio/cargo/release/`。只编译该库可用 `colcon build --packages-select point_lio`。

本项目实际在 `ros2` 容器内构建。首次使用需在容器内安装 Rust ≥1.82
（建议 rustup minimal stable），宿主机安装的 Rust 不会自动进入容器。
如果宿主机也执行过构建，请使用独立缓存目录：

```sh
# 在 ros2 容器内，进入 /workspace/GKD.RoboNav
source /opt/ros/jazzy/setup.bash
colcon --log-base log/ros2 build --build-base build/ros2 --install-base install/ros2
```

此时 Rust 产物位于 `build/ros2/point_lio/cargo/release/`，ROS 环境入口为
`install/ros2/setup.bash`。CMake 可自动发现容器用户 `~/.cargo/bin/cargo`。


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

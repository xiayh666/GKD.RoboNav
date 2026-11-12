# terrain_map

纯 Rust 在线地形地图库，Cargo 包名 `terrain-map`，无 ROS 类型或 ament 依赖。
目前完成体素地图、Tokio 运行层与人工标注管理。地面特征、自动隧道识别、机器人通行性和导航栅格仍待实现，不能将当前输出直接当作导航通行结论。

## 已实现架构

- `Mapper`：同步状态所有者；8×8×8 稀疏体素块，局部范围与过期淘汰。
- `voxel`：逐回波 DDA 射线更新，空闲/占用/未知分离。每帧每格最多一次证据更新，同帧命中优先于空闲；证据限幅为 [-8,8]，命中 +2、空闲 -1，这不是标定后的占用概率。
- 私有 Rayon CPU 池：`Config.cpu_threads` 控制工作线程，默认 2。按点云块并行计算证据位图、合并后按帧顺序提交，不并发写共享体素。
- `TerrainRuntime`：使用调用方 Tokio runtime，通过有界 `mpsc` 接收帧和标注命令；最多一个 `spawn_blocking` 计算在途，内部再用 Rayon 并行。线程数不包含 Tokio 和 blocking 协调线程。
- `MapSnapshot`：完整版本化快照。块通过 `Arc` 共享，写时复制；`watch` 允许慢读者跳过旧版本，而不会漏掉增量。保留旧快照会保留相应内存，调用方应释放不用的版本。

这使同步离线回放与异步在线运行共用同一核心，帧内并行度可调，帧之间的状态提交保持串行。

## 输入约定

`Frame` 包含严格递增的秒时间、局部地图中心和 `Vec<Ray>`。每条射线的 `origin` 与 `endpoint` 必须已变换到同一个重力对齐、米制坐标系，且对应回波采样时刻。

本库不做去畸变、外参变换和自体点过滤。Point-LIO 目前输出的 `world_points` 不包含逐点观测原点，因此当前尚未自动连接两个库；不能用帧末 IMU 位置替代所有射线原点。

超出 `max_range` 的回波仅清除截断射线，不在截断位置制造占用点。没有回波并不代表该方向可清空；空帧仅推进时间和地图窗口。窗口外、未观察或过期的格查询为未知。没有看到地面也不能视为可通行。

地图容量按 `max_blocks` 限制；输入点数、单束遍历步数、暂存唯一体素数也有上限。超限或无效帧拒绝整帧，不提交部分地图。并行暂存与历史快照会产生额外峰值内存，`max_blocks` 不是进程总内存上限。默认参数是可配置的工程起点，不是机器人通过能力参数。

## 异步运行与背压

```sh
cargo run --release --locked -p terrain-map --example online
```

`TerrainRuntime::start(config)` 要求已有 Tokio runtime，支持 current_thread。

- `handle.try_submit(frame)`：立即返回回执，队列满则返回 `RejectedFrame { error: Busy, frame }`，保留原帧供调用方选择重试或舍弃，不静默丢帧。
- `handle.submit(frame).await`：等待队列空间；拿到回执代表接收，`receipt.wait().await` 才报告提交成功或帧校验失败。取消等待提交的 future 会丢弃 future 持有的帧；已接收的请求不会因回执被丢弃而撤销。
- `handle.subscribe()`：接收完整最新快照。`revision` 是总提交版本，`map.version` 是点云地图版本，`annotation_version` 独立记录标注变化。
- `upsert_annotation` / `remove_annotation`：走同一个有界 FIFO，返回生效后的快照。最多保留 1024 条标注。
- `runtime.shutdown().await`：关闭接收，处理已接收请求，等待计算退出；不是强制中断正在运行的几何计算。单独 drop runtime 不会取消外部 handle 仍持有的服务；所有 handle 都释放后自动排空并退出。

人工标注当前支持地图系轴对齐三维框：`TunnelHint` 仅保存隧道位置提示，不冒充自动检测结果；`Forbidden` 表示强制禁行。
`manually_forbidden(point)` 只查询点是否处于人工禁行框内，不包含车体碰撞膨胀；返回 false 不代表该位置可通行。

## 构建与验证

Tokio 当前锁定版本要求 Rust ≥1.85。ROS 2 容器已经安装的 Rust 1.99.1 满足要求。

```sh
cargo test --release --locked -p terrain-map
cargo run --release --locked -p terrain-map --example throughput -- 10000 5
```

实际 `ros2` 容器挂载工程到 `/workspace/GKD.RoboNav`，统一构建：

```sh
source /opt/ros/jazzy/setup.bash
colcon --log-base log/ros2 build --build-base build/ros2 --install-base install/ros2
```

只编译本库可追加 `--packages-select terrain_map`。默认产物位于 `build/ros2/terrain_map/cargo/release/libterrain_map.rlib`。
CMake 安装仅包含此说明；Rust 消费方仍使用 Cargo path 依赖。`TERRAIN_MAP_CARGO_PROFILE` / `TERRAIN_MAP_CARGO_TARGET_DIR` 可控制构建配置和缓存目录。

设计与验证记录见 [doc/terrain-map](../../doc/terrain-map/design.md)。已确定的后续需求：全向轮底盘的净空/台阶分析，以及“有顶、有两侧墙且连续延伸”的隧道自动识别；隧道即使几何可通过也禁止通行，同时支持人工提示与强制禁行区域。

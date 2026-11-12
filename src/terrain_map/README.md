# terrain_map

纯 Rust 在线地形地图库，Cargo 包名 `terrain-map`，无 ROS 类型或 ament 依赖。
目前完成上游观测增量接入、独立表面几何视图、可选射线占据地图、Tokio 运行层与人工标注管理。地面特征、自动隧道识别、机器人通行性和导航栅格仍待实现，不能将当前输出直接当作导航通行结论。

## 已实现架构

- `Mapper`：同步状态所有者；8×8×8 稀疏体素块，局部范围与过期淘汰。
- `voxel`：逐回波 DDA 射线更新，空闲/占用/未知分离。每帧每格最多一次证据更新，同帧命中优先于空闲；证据限幅为 [-8,8]，命中 +2、空闲 -1，这不是标定后的占用概率。
- 私有 Rayon CPU 池：`Config.cpu_threads` 控制工作线程，默认 2。按点云块并行计算证据位图、合并后按帧顺序提交，不并发写共享体素。
- `TerrainRuntime`：使用调用方 Tokio runtime，通过有界 `mpsc` 接收帧和标注命令；最多一个 `spawn_blocking` 计算在途，内部再用 Rayon 并行。线程数不包含 Tokio 和 blocking 协调线程。
- `MapSnapshot`：完整版本化快照。块通过 `Arc` 共享，写时复制；`watch` 允许慢读者跳过旧版本，而不会漏掉增量。保留旧快照会保留相应内存，调用方应释放不用的版本。

这使同步离线回放与异步在线运行共用同一核心，帧内并行度可调，帧之间的状态提交保持串行。

## 输入约定

主入口是 `MapUpdate`：先发送显式Reset，再发送Point-LIO的`ScanResult.observations`。
`SurfaceSnapshot`直接使用生产者分辨率和i64体素键，保留独立的局部几何视图；不跟随iVox LRU删除。
第一次无需裁剪/限样的观测数组可直接共享Arc，后续只合并被观测的格，每格最多保存
`max_surface_samples_per_voxel`个最近样本（默认256），最多`max_surface_voxels`格（默认200000）。
`surfaces.live_samples(key)`排除单个过期样本，几何算法应使用此接口；格整体按最近观测时间老化。
未出现在本批次中的格不会直接删除，仍按本模块窗口/老化管理。

`MapSnapshot.surfaces`是主几何视图，含源会话、下一序号、分辨率、帧末位姿及状态。
`integrate_observation_rays`默认false，此时`MapSnapshot.map`不会重复进行射线建图；如需独立自由空间证据可显式开启，使用本模块`resolution`积分。
两个视图的分辨率可不同，主几何不重新分组。`SurfaceVoxelKey`是上游几何键，原`VoxelKey`是可选占据地图键。

出现缺失序号后返回SequenceGap，清空可用几何/占据视图并发布NeedsReset状态，拒绝后续融合，直到显式Reset。
同会话重同步保留人工标注；换源/epoch时旧坐标系标注清空，不能假定新里程计坐标与旧坐标一致。
重复批次和迟到的旧会话消息仅拒绝，不推进版本。初次Reset会清空未绑定会话的旧标注。

旧`Frame`是独立的低层射线输入，接入观测会话后禁止混用。它包含递增秒时间、局部中心和逐回波世界系原点/终点；本库不做去畸变、外参变换和自体过滤。

超出 `max_range` 的回波仅清除截断射线，不在截断位置制造占用点。没有回波并不代表该方向可清空；空帧仅推进时间和地图窗口。窗口外、未观察或过期的格查询为未知。没有看到地面也不能视为可通行。

地图容量按 `max_blocks` 限制；输入点数、单束遍历步数、暂存唯一体素数也有上限。超限或无效帧拒绝整帧，不提交部分地图。并行暂存与历史快照会产生额外峰值内存，`max_blocks` 不是进程总内存上限。默认参数是可配置的工程起点，不是机器人通过能力参数。

## 异步运行与背压

```sh
cargo run --release --locked -p terrain-map --example online
```

`TerrainRuntime::start(config)` 要求已有 Tokio runtime，支持 current_thread。
主接口为 `try_submit_update` / `submit_update`，拒绝时返回包含原MapUpdate的RejectedUpdate；批次Arc移动而不复制整帧。Reset与批次走相同FIFO。丢序的错误回执同时会通过watch发布失效状态。
旧射线接口如下，仅用于独立低层输入：

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
colcon build
```

只编译本库可追加 `--packages-select terrain_map`。默认产物位于 `build/terrain_map/cargo/release/libterrain_map.rlib`。
CMake 安装仅包含此说明；Rust 消费方仍使用 Cargo path 依赖。`TERRAIN_MAP_CARGO_PROFILE` / `TERRAIN_MAP_CARGO_TARGET_DIR` 可控制构建配置和缓存目录。

设计与验证记录见 [doc/terrain-map](../../doc/terrain-map/design.md)。已确定的后续需求：全向轮底盘的净空/台阶分析，以及“有顶、有两侧墙且连续延伸”的隧道自动识别；隧道即使几何可通过也禁止通行，同时支持人工提示与强制禁行区域。

## Point-LIO连接示例

```sh
cargo run --release --locked -p terrain-map --example lio_connection
```

该合成示例在单线程Tokio调度器上运行，Point-LIO计算放入spawn_blocking，结果通过Arc和有界通道送入地形模块。
配置iVox只保留5格，地形视图仍能独立积累49、91、133格，验证定位缓存淘汰不等于环境消失。尚未接入实际ROS传感器，也未实现地形/隧道分析。

异步人工区域命令会校验会话。UI请用`upsert_annotation_in_session` / `remove_annotation_in_session`传入所展示快照的session，避免队列处理期间Reset造成坐标错用。默认接口捕获实际提交时latest会话，不能替代UI版本关联。每个Mapper保留至多1024个生产源的最大epoch，迟到旧源Reset被拒绝。
本阶段验证见[观测接口报告](../../doc/terrain-map/observation-interface.md)。

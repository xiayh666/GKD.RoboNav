# Point-LIO Rust

基于作者 [hku-mars/Point-LIO](https://github.com/hku-mars/Point-LIO)
的 Rust 重写，参考提交 `4b86a469eb5572e70ed575af25b5f15dd06e8e3c`。
不依赖 ROS、PCL、Python、C++ 或 FFI，仅使用 Rust `nalgebra`。
地图由当前传感器数据在线建立，无先验点云输入。

## 构建与运行

在工作区根目录执行（需 Rust 1.82+）：

```sh
cargo test --workspace --release
cargo run --release -p point-lio --example replay
cargo run --release -p point-lio --example replay -- events.csv > odometry.csv
```

不传文件运行静止平面演示。CSV 回放按文件顺序处理三类记录，不包含表头：

```text
# IMU: 时间秒,角速度 rad/s 三轴,比力 m/s² 三轴
I,0.000,0,0,0,0,0,9.81
I,0.005,0,0,0,0,0,9.81
# 继续提供默认至少 100 个静止 IMU 样本，完成初始化
# 初始化结束后，继续提供覆盖扫描结束时刻的 IMU，再提交扫描
# 点: 绝对采样时间秒,雷达坐标系 xyz 米,强度
P,0.550,1,0,-1.5,10
# 扫描提交: 起始时间秒,结束时间秒
S,0.500,0.600
```

上面展示记录格式，省略的 IMU 数据必须补齐。回放示例使用默认配置；
真实设备请在构造 `Config` 时填入外参、量程和噪声后调用库。
输出 CSV 为扫描结束时刻 IMU 位姿（四元数 xyzw）、匹配数和地图点数。
它不是 `base_link` 位姿，底盘外参转换由上层负责。

## 库接口与时间约定

```rust
use point_lio::{Config, PointLio, ImuSample, Scan};

fn process(imu: impl IntoIterator<Item = ImuSample>, scan: &Scan)
    -> Result<(), point_lio::Error>
{
    let mut lio = PointLio::new(Config::default())?;
    for sample in imu { lio.push_imu(sample)?; }
    let result = lio.process_scan(scan)?;
    println!("{:?}", result.state.position);
    Ok(())
}
```

实际应用中持续保留同一个 `PointLio` 实例：

1. 输入静止 IMU 窗口（默认 100 点），估计重力方向并检查静止条件。
   比力按已知重力模长初始化，零偏初值与官方默认模型一样置零，随后参与滤波。
   世界坐标系 Z 向上，起点为零，航向由初始 IMU 轴确定。运动或饱和
   窗口返回 `NotStationary` 并清空初始化窗口，可继续收集重试。
2. 按严格递增时间调用 `push_imu`，再调用 `process_scan`。IMU 与雷达
   必须使用同一时钟；IMU 时间不得重复。默认最大允许间隔 50ms。
3. `Scan.start_time/end_time` 是绝对秒；`LidarPoint.offset` 是相对起始
   时刻的秒。原始毫秒/微秒/纳秒必须先转换；点坐标是采样瞬间的雷达系
   坐标。不要预先做整帧运动补偿。库内部按点时间排序并传播状态。
4. 初始化之前的扫描应丢弃；扫描区间不得重叠或早于当前滤波时刻。
   空扫描允许，会消耗该时段 IMU 并推进状态。
5. `NeedMoreImu` 表示可保留扫描，补足覆盖到扫描结束的 IMU 后重试；
   `ImuGap` 表示覆盖中断，应由调用者处理，重启时用 `reset()`。
   失败扫描不消耗 IMU、不修改滤波器和地图。
6. IMU 队列默认最多 20,000 点，满时返回 `ImuBufferFull`，样本未被接收；
   消费扫描后可重试。时间重置需显式 `reset()`。实例之间状态完全隔离。

外参定义与作者一致：`p_imu = R_LI * p_lidar + t_LI`。
默认固定外参；`estimate_extrinsics=true` 启用六维在线估计，需要足够运动激励。
默认量程/噪声只是启动配置，不是设备标定结果。加速度必须是含静止重力
反作用的比力，不能传已经去重力的线加速度。

## 算法对应与差异

| 作者源码 | Rust 实现 |
|---|---|
| `common_lib.h::state_output` | `filter::State`，30 维误差状态及协方差 |
| `Estimator.cpp::get_f_output/df_dx_output` | 名义状态逐点传播，协方差使用独立 IMU 时钟 |
| `h_model_IMU_output` | `observe_imu`，角速度/比力及零偏观测，0.99 量程饱和屏蔽 |
| `h_model_output` | `plane_jacobian/observe_planes`，同采样时刻联合更新，含外参和门限 |
| `laserMapping.cpp` | `PointLio::process_scan`，IMU/逐点时间事件与帧末在线地图提交 |
| iVox / `esti_plane` | 同体素多点、26 邻域、按写入更新 LRU、五近邻 QR 平面拟合 |

保留官方默认 `use_imu_as_input=false` 模式，尚未实现可选 24 维
input-state 分支。没有 ROS 消息解析、厂商逐点时间提取、RViz、PCD
保存、回环和 Nav2 桥接；现有 Nav2 配置尚未接到 Rust 库。

名义状态传播到每个点的时间，协方差按 IMU 时间传播，避免把 `Q * dt²`
按激光点数切碎。IMU 六轴联合观测并屏蔽饱和轴；同时间激光点在同一预测
状态上线性化、联合更新。恢复官方 `P - K H P` 更新，不额外插入 Joseph
稠密乘法或姿态误差重置。大激光批量使用与小批量一致的方差公式；官方源码
的大批量信息分支存在噪声权重不一致，未复制该分支，区别见修正报告。

初始点跨帧累计到 `initial_map_min_points`（默认 10）后建图，不能让稀疏
首帧形成的单点体素阻止后续建图。初始地图保留同体素的多个点；后续按
查询时缓存的近邻决定哪些体素需要新点。`filter_size_map` 控制插点筛选格，
`voxel_size` 仅控制 iVox 索引格（对应官方 `ivox_grid_resolution`）。
两者独立配置，默认均为 0.3m；例如可设置前者 0.5m、后者 0.2m。
这不包含输入点云的 PCL VoxelGrid 降采样。地图容量指索引体素数量，
`ScanResult.map_voxels` 报告体素数，`map_points` 报告实际保留点数；两者
不再相同。查询遍历中心及 26 相邻体素，`neighbor_radius` 是其中候选点
的距离上限（默认 5m），不会扩大体素枚举范围。写入时维护 LRU。

仍有启动、浮点精度、秩亏 QR 基本解选择和容量边界等实现差异；没有声称
整个程序与官方逐位等效。秩亏近邻可能不能唯一确定平面，匹配数量不能
证明位姿可观测。静止初始化的有效性检查和空扫描推进属于本库接口约定。

修正后新增直接来自官方 C++ 的滤波数值基准：七个检查点的状态与协方差
最大绝对差约 `1.3e-14`。旋转和默认容量含噪静止测试通过；300 体素压力
场景仍失败，完整测试集仍不是全绿，失败断言保留。真实雷达数据和论文
4–8kHz 输出能力尚未验证。

最新结果见[修正报告](../../doc/point-lio-rust/correction.md)。此前版本的
[官方对照](../../doc/point-lio-rust/official-comparison.md) 与
[场景验证](../../doc/point-lio-rust/validation.md) 保留为历史记录。

原作者版权和许可保存在 `LICENSE`；来源与差异见 `NOTICE`。

## 接入工程构建

目录中的 `CMakeLists.txt` 是普通 CMake 构建入口，供 colcon 调用 Cargo；不使用 ROS、ament 或 `package.xml`。

```sh
colcon build
# 或只编译库
colcon build --packages-select point_lio
```

默认执行 `cargo build --lib --locked --profile release`，产物位于
`build/point_lio/cargo/release/`；每次工程构建都会调用 Cargo，由 Cargo 判断是否增量编译。
首次构建需要 Rust 工具链及可用的 Cargo 依赖缓存或网络。配置缺少 Cargo 或 Cargo 编译失败时，工程构建也会失败。

可用 `--cmake-args -DPOINT_LIO_CARGO_PROFILE=dev` 切换 Cargo 开发配置，
或用 `-DPOINT_LIO_CARGO_TARGET_DIR=/absolute/path` 指定产物目录。
CMake 的 `CMAKE_BUILD_TYPE` 不控制 Cargo 配置。
安装阶段只安装 README、许可证及 NOTICE；不会将 rlib 当作 C/C++ 库导出。
其他 Rust crate 仍通过 Cargo `path` 依赖引用本库；供 ROS 节点调用的接口不在此构建入口中实现。

实际 `ros2` 容器挂载工程到 `/workspace/GKD.RoboNav`；已安装 Rust 工具链。
与宿主机共用工程时，容器构建使用 `colcon --log-base log/ros2 build --build-base build/ros2 --install-base install/ros2`，避免混用 CMake 绝对路径缓存。该配置的产物位于 `build/ros2/point_lio/cargo/release/`。

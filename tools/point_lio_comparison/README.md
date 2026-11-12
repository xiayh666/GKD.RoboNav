# 官方 Point-LIO 同数据对照

最新修正后的 Rust 回放与共同帧指标见
`doc/point-lio-rust/correction.md`。`replay_rust.py INPUT_DIR OUTPUT_DIR`
验证输入校验值后运行当前 Rust，比较保存在 results/ 的原始官方轨迹。
`filter_oracle.py OUTPUT.csv` 则直接编译官方滤波核心生成数值基准，
不涉及地图或完整 ROS 节点，两种验证范围不同。

使用作者仓库提交 `4b86a469eb5572e70ed575af25b5f15dd06e8e3c`，
`point-lio-with-grid-map` 分支。`run_reference.py` 运行完整 ROS 1 节点，
不提取、重写或替换其滤波与地图算法。

环境：ROS Noetic / Ubuntu 20.04 Docker。Livox 仅编译官方消息定义，
不用驱动硬件，不需要 SDK。`build_reference.sh` 创建消息包构建元数据；
Point-LIO 源码复制到临时工作区编译，不改工作区里的官方源码。

## 流程

1. `python3 tools/point_lio_comparison/export_data.py /tmp/gkd-point-lio-comparison`
   输出静止 60 秒、旋转正常量程及旋转裁剪量程三份 CSV。
   场景公式对应 `tests/usability.rs`；两种实现实际读取同一份文件。
2. Rust 回放使用 `cargo run --release -p point-lio --example replay -- INPUT --ros-f32`。
   裁剪场景增加 `--gyro-saturation=0.3`。`--ros-f32` 将输入点坐标和时间偏移
   转换到官方 PointCloud2/预处理使用的 float32 精度；IMU 仍为 float64。
3. 构建 Dockerfile 后，在容器中挂载 `/official`（作者源码）、`/livox_messages`
   （Livox 官方 msg 目录）、`/work`（临时构建目录），运行 `build_reference.sh`。
4. 启动 roscore，source `/work/catkin/devel/setup.bash`，运行
   `run_reference.py INPUT OUTPUT [--gyro-limit 0.3]`。
5. `compare_metrics.py CSV... [--rotating]` 按各自输出时间计算解析真值误差。

## 对照约束

- 统一外参、SI 单位、量程、噪声数值、平面门限和全部输入点。
  官方关闭扫描空间降采样，`point_filter_num=1`，以接收与 Rust 相同的点。
  参数写入每个输出旁的 `.params.json`。
- 官方与当前 Rust 地图均保留同体素多点。容量补测在官方临时副本增加配置入口，
  同时测配置 300/301 以处理官方容量边界差异，证据见
  `doc/point-lio-rust/results-capacity/README.md`。
- 官方初始化和首帧建图流程保持原样，可能比 Rust 少输出第一帧。
- 官方输出时间是末点时间，Rust 是 Scan.end_time。指标分别在各自时间戳
  上计算真值，不把这约 1ms 的差异当作姿态误差。
- 每帧等待官方里程计/初始地图应答再提交下一帧，避免离线输入淹没队列。
  壁钟回放耗时包含通信和等待，不能用作官方纯算法实时性指标。
- 这些是合成数据结果，没有真实雷达射线可见性、遮挡或设备误差模型，
  不用于宣称论文精度或实车性能。

## 目录整理后

项目 `src/point_lio` 现为 Rust 库，官方 C++ 源码已移除。复现官方测试时需
在项目外单独检出上文指定提交，挂载到容器 `/official`。
生成滤波基准时显式传入 `filter_oracle.py OUTPUT --upstream /path/to/official-checkout`。
已有 Rust 基准测试直接读取已保存 CSV，不依赖 C++ 源码或 ROS。

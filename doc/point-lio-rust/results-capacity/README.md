# 官方与 Rust 容量压力对照

2026-11-12；官方 hku-mars/Point-LIO 提交 4b86a469eb5572e70ed575af25b5f15dd06e8e3c，point-lio-with-grid-map 分支。ROS Noetic 测试镜像 gkd-pointlio-reference:noetic。Rust Release。

## 数据及口径

同一 stationary.csv，60 秒、600 帧、每帧 999 点，200Hz IMU，静止含噪，体素索引及地图插点尺寸均为 0.3m。SHA256 见 summary.json。Rust 使用 --ros-f32 统一点坐标与偏移输入精度。官方不输出首帧里程计，因此在共同的 599 帧上统计位置范数、相对真实姿态的旋转角。真值位置均为零、姿态为单位旋转。

官方 AddPoints 在体素数 >= capacity 时淘汰，因此 300 配置最多保留 299 格，301 配置最多保留 300 格；Rust 配置 300 最多保留 300 格。本报告同时给出相同配置值和相同实际上限的对照。上限由源码逻辑确定，本次未新增运行时地图数量遥测。

| 实现 | 配置容量 | 最大位置误差 | 最大姿态误差 | 位姿阈值通过 |
|---|---:|---:|---:|---|
| 官方 | 100000 | 0.025459m | 0.009055rad | 是 |
| Rust | 100000 | 0.026380m | 0.009103rad | 是 |
| 官方 | 300（最多299格） | 2.123980m | 0.358340rad | 否 |
| 官方 | 301（最多300格） | 2.196736m | 0.372025rad | 否 |
| Rust | 300（最多300格） | 1.572311m | 0.298405rad | 否 |

沿用 Rust 压力测试位姿阈值：最大位置 <0.15m、最大姿态角 <0.05rad。本次报告仅据轨迹判定位姿阈值，不声称检查了官方内部协方差和最少匹配数。

小容量会使两者在本场景明显漂移。该结果不证明整体等效，也不能用单个合成场景宣称 Rust 精度优于官方。之前 Rust 原生 f64 测试的 1.642475m 与本次共享 CSV/float32 输入结果不是同一输入精度口径。

## 复现

1. 用 tools/point_lio_comparison/export_data.py 生成共享输入目录。
2. 将外部检出的上述官方提交复制至临时目录（项目内 src/point_lio 现为 Rust 库），仅在副本中应用 reference.patch（patch -p1），增加 comparison_capacity 参数入口。官方原仓库未修改。
3. 按 tools/point_lio_comparison/build_reference.sh 构建副本，挂载官方 Livox 提交 3d240d5666129e1a3052e78ee8487a04b08fdda3 的 livox_ros_driver/msg 消息定义。
4. 容器内运行本目录 run.sh；挂载临时构建目录为 /work，共享输入为 /input，本目录为 /results，本目录 run_reference.py 所在目录为 /capacity。脚本执行三档官方容量。
5. Rust：cargo run --release -p point-lio --example replay -- INPUT/stationary.csv --ros-f32 --map-capacity=300，保存 stdout 为 rust_300.csv；100000 同理。
6. python3 tools/point_lio_comparison/report_capacity.py doc/point-lio-rust/results-capacity INPUT/stationary.csv

各 CSV 是原始轨迹；官方 .params.json 和 .node.log 保留配置与节点日志；reference.patch 是完整源码改动，run_reference.py 仅在原回放器增加容量参数。

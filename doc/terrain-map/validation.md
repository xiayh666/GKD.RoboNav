# T2 体素与异步并行底座验证

2026-11-12。实际 ros2 容器，Rust 1.99.1，Release，0.1m 体素。当前范围不包含地形特征、隧道自动检测或机器人通行性。

## 正确性

宿主与容器均通过13项测试：9项地图/标注，4项异步运行。覆盖未知/空闲/占用、负坐标、对角与精确边界、量程截断、过期与局部范围、非法及超容量帧的事务拒绝、COW历史快照、串并行结果一致、人工提示/强制禁行区别、current_thread调度、满队列返还输入、提交确认、关闭排空、丢弃回执及慢watch读者。

workspace clippy（all-targets，-D warnings）和fmt通过。容器统一colcon构建成功，3 packages finished。online示例完成3帧和一次标注提交，明确输出自动隧道检测未实现。

## 吞吐

`throughput`使用确定性合成射线，原点[0.05,0.05,1.05]，约10m远回波，覆盖三维空间；包含射线遍历、并行位图合并、预检、地图提交与快照生成，不包含输入构造和线程池启动。不包含LIO、地形、隧道、导航，也不代表实际雷达完整系统吞吐。

每档预热一帧，随后5帧；1、2、4工作线程依次测量，没有同时运行编译任务。线程数指私有Rayon CPU池，不包括Tokio/blocking协调线程。地图约2550个块。

| 射线数/帧 | 1工作线程中位 | 2工作线程中位 | 4工作线程中位 |
|---|---:|---:|---:|
| 10000 | 38.986ms | 22.287ms | 15.322ms |
| 20000 | 75.109ms | 41.132ms | 27.208ms |

4工作线程范围：10000条14.285～15.900ms；20000条24.800～30.347ms。短样本不构成长时间最坏延迟保证。详细记录为results/container-throughput-*.log。宿主机初测也留档，其中20000条后半段与容器构建有重叠，不用于正式性能判断。

早期逐体素HashSet合并实测10000条1线程197.155ms、4线程146.271ms；改为块内512位掩码后再运行全部正确性测试。最终报告使用上述容器单独测量结果，不将早期临时实现作为官方基线。

## 复现

在ros2容器工程根目录中：

```sh
source "$HOME/.cargo/env"
cargo test --release --locked -p terrain-map --target-dir build/terrain_map/cargo
cargo run --release --locked -p terrain-map --example online --target-dir build/terrain_map/cargo
cargo run --release --locked -p terrain-map --example throughput --target-dir build/terrain_map/cargo -- 10000 5
cargo run --release --locked -p terrain-map --example throughput --target-dir build/terrain_map/cargo -- 20000 5
```

## 尚未覆盖

真实雷达输入；Point-LIO采样时刻射线原点接入；长期动态场景与峰值内存；地形/隧道检测精度；通行性及导航联调。人工强制禁行当前仅做点在轴对齐框内查询，尚无机器人外形膨胀。不要把当前体素占用查询直接当作通行判断。

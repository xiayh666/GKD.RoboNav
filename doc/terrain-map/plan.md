# 骨架执行计划

| id | title | owner | files | status | depends_on | verify |
|---|---|---|---|---|---|---|
| T1 | 创建独立 Rust 库并接入构建 | codex | Cargo.toml, Cargo.lock, src/terrain_map, README, doc/terrain-map | completed | — | Cargo release/fmt/clippy通过；ros2容器colcon识别cmake，3 packages finished，terrain_map库构建成功 |

| T2 | 体素核心与异步并行运行层 | codex | src/terrain_map, Cargo.lock, 文档 | completed | T1 | 13项测试宿主与容器均通过；fmt/clippy通过；3包colcon构建通过；1/2/4线程耗时留档validation.md |

T2范围：同步体素核心、帧内Rayon并行、Tokio有界输入/命令/快照/关闭。几何、隧道及通行性在后续阶段实现，不以占位结论替代分析。

| T3 | 接通Point-LIO观测体素增量 | codex | src/map_updates, src/point_lio, src/terrain_map, 文档 | completed | T2 | 48项相关回归通过；4场景回放字节一致；容器27项协议/地形回归及3包构建通过；证据observation-interface.md |

| T4 | 完整链路延迟、资源与持续压力验收 | codex | examples/chain_stress.rs, tools/terrain_map_stress, results及文档 | pending | T3 | ros2容器Release运行1k/10k/20k点10Hz、持续旋转、主观测路径分段与超速背压；记录分位延迟、CPU/RSS、积压、TTL/窗口/容量事务断言 |

T4已按用户要求停止：已完成的链路/持续/隔离/背压/窗口数据保存在results/chain-stress；默认20万格容量例程尚未执行，完整压力验收未完成。不要自动继续。

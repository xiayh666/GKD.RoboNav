# 执行计划

| id | title | owner | files | status | depends_on | verify |
|---|---|---|---|---|---|---|
| T1 | 官方方程与接口设计 | codex | doc/point-lio-rust/* | completed | — | 对照 Estimator.cpp/common_lib.h |
| T2 | Rust 滤波、地图和融合接口 | codex | src/point_lio/src/* | completed | T1 | cargo test --workspace --release：11 项通过 |
| T3 | 回放示例和验证说明 | codex | tests, examples, README | completed | T2 | cargo fmt --all -- --check; cargo clippy --workspace --all-targets -- -D warnings; cargo test --workspace --all-targets --release; cargo run --release -p point-lio --example replay，全部通过 |
| T4 | 使用场景与耗时验证 | codex | tests/usability.rs, doc/point-lio-rust/validation.md | completed | T3 | 已执行 5 项场景检查与 1 项手动耗时测量，保留 2 项失败断言；完成的是验证工作，算法未通过可用性验收 |
| T5 | 官方完整节点同输入对照 | codex | tools/point_lio_comparison/*, doc/point-lio-rust/official-comparison.md, results/* | completed | T4 | 完整官方 ROS 1 节点已执行 5 个精度场景与 3 档点数回放；共享文件校验值、参数、轨迹和日志已保存，差距与计时边界已记录 |
| T6 | 按官方模型修正 Rust 传播、观测与地图 | codex | filter.rs, pipeline.rs, map.rs, types.rs, tests, 文档 | completed | T5 | 核心修正与复验完成：9 单元+11 融合回归通过；官方 C++ 数值基准最大差 1.3e-14；旋转及默认场景通过；300 体素预算压力仍失败，不标记整套可用性验收通过 |


| T7 | 拆分地图筛选与 iVox 索引尺度 | codex | types.rs, map.rs, pipeline.rs, tests, 文档 | completed | T6 | 10 单元+11 融合测试通过；新增配置校验复验通过；fmt/clippy 通过 |

| T8 | 官方小容量同数据对照 | codex | tools, results-capacity, 文档 | completed | T7 | 官方 100000/300/301、Rust 100000/300 回放完成；599 共同帧；大容量通过，小容量均未通过位姿阈值；证据 results-capacity |

| T9 | 移除官方源码并统一 Rust 目录 | codex | Cargo.toml, src/point_lio, README, tools, 文档 | completed | T8 | fmt/clippy、21 核心/融合测试通过；场景 4通过/1已知失败/1忽略；目录、Python及shell语法检查通过 |

| T10 | 通过普通 CMake 接入 colcon 构建 | codex | src/point_lio/CMakeLists.txt, README, 文档 | completed | T9 | colcon识别cmake，Release库构建成功，增量Cargo 0.01s，Cargo失败返回非零 |

| T11 | 在实际 ros2 容器验证统一构建 | codex | CMakeLists.txt, README, 文档 | completed | T10 | ros2容器内首次及增量全工程构建均成功，2 packages finished；Rust 1.99.1 |

| T12 | 导出筛点前的体素观测协议 | codex | pipeline.rs, types.rs, map_updates, 连接回归 | completed | T11 | 21核心/融合回归通过；4场景CSV字节一致；filter/map未改，见doc/terrain-map/observation-interface.md |

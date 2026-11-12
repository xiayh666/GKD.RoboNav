# 骨架执行计划

| id | title | owner | files | status | depends_on | verify |
|---|---|---|---|---|---|---|
| T1 | 创建独立 Rust 库并接入构建 | codex | Cargo.toml, Cargo.lock, src/terrain_map, README, doc/terrain-map | completed | — | Cargo release/fmt/clippy通过；ros2容器colcon识别cmake，3 packages finished，terrain_map库构建成功 |

| T2 | 体素核心与异步并行运行层 | codex | src/terrain_map, Cargo.lock, 文档 | completed | T1 | 13项测试宿主与容器均通过；fmt/clippy通过；3包colcon构建通过；1/2/4线程耗时留档validation.md |

T2范围：同步体素核心、帧内Rayon并行、Tokio有界输入/命令/快照/关闭。几何、隧道及通行性在后续阶段实现，不以占位结论替代分析。

# 执行计划

| id | title | owner | files | status | depends_on | verify |
|---|---|---|---|---|---|---|
| L1 | Rust C ABI与语义测试 | codex | src/point_lio_bridge Rust及头文件 | completed | — | Rust测试、fmt/clippy、既有21核心测试 |
| L2 | ROS消息同步与输出 | codex | bridge C++/CMake、bringup配置与launch | completed | L1 | colcon构建、合成ROS端到端/故障检查 |
| L3 | MID-360实机位姿验证 | codex | 工具/文档/结果 | completed | L2 | 实机运动输出、TF、轨迹与诊断留档；用户确认显示效果；完整静态精度待验 |

L3仅完成实机接入及可视化验收。静态漂移尝试包含实际运动，未完成30秒纯静态精度验收。

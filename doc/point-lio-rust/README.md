# Point-LIO 验证资料

当前 Rust 实现位于 [src/point_lio](../../src/point_lio/README.md)。官方 C++ 源码已从项目移除；源码出处与许可证保留在 Rust 库 NOTICE、LICENSE 中。

- [实现设计](design.md)、[目标](goal.md)、[执行记录](plan.md)、[开发日志](log.md)。
- [核心修正报告](correction.md)：滤波数值基准与轨迹对照。
- [最新容量对照](results-capacity/README.md)：共享输入下官方与 Rust 的小容量测试。
- [最新耗时复测](results-corrected/timing-retest.log)、[最新场景复测](results-corrected/pressure-retest.log)。
- [早期官方对照](official-comparison.md)、[早期场景验证](validation.md)：历史结果，不能作为当前版本指标。
- `results/` 保存早期基线；`results-corrected/` 保存修正后记录；`results-capacity/` 保存容量补测证据。原始结果保持不变。

工程统一编译使用 `colcon build`；`point_lio` 为普通 CMake 项目，实际调用 Cargo，详见库 README。

## 运行检查

```sh
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace --release --lib --test fusion
cargo test --release -p point-lio --test usability -- --nocapture --test-threads=1
cargo test --release -p point-lio --test usability processing_time_by_scan_size -- --ignored --nocapture --test-threads=1
```

完整场景测试仍有 300 体素预算失败，保留原断言。2 万点耗时仍无法稳定达到 10Hz。
官方对照工具保留在 [tools/point_lio_comparison](../../tools/point_lio_comparison/README.md)，重跑官方或重新生成 C++ 数值基准需在项目外另行检出官方指定提交；日常 Rust 构建和基准回归不需要这些依赖。

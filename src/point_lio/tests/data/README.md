# 官方 C++ 滤波基准

`official_filter.csv` 由作者提交
`4b86a469eb5572e70ed575af25b5f15dd06e8e3c` 的原始 ESEKF 头文件和
`Estimator.cpp` 中 output-state 动力学、雅可比、过程噪声、IMU 观测函数直接编译生成。
没有重新实现这些 C++ 函数。独立驱动提供两个解析平面观测，避开地图匹配，
以隔离检验滤波代数与时间调度。

```sh
python3 tools/point_lio_comparison/filter_oracle.py /tmp/pointlio-oracle/reference.csv --upstream /tmp/Point-LIO-reference
cmp /tmp/pointlio-oracle/reference.csv src/point_lio/tests/data/official_filter.csv
cargo test --release -p point-lio output_filter_matches_official_cpp_oracle -- --nocapture
```

官方源码已从项目移除；重新生成时，请将上述官方提交单独检出到 `/tmp/Point-LIO-reference`。
需本机 C++ 编译器、Eigen、Boost 头文件；Rust 使用该已生成文件测试时不需要 C++。
每行：时间、32 个名义状态标量（位置 xyz、姿态 xyzw、外参旋转 xyzw、
外参平移、速度、角速度、比力、重力、陀螺零偏、加计零偏），最后是
30×30 协方差，按列存储。固定外参的初始协方差与 Rust 相同，置零。

100 次 200Hz IMU，区间内插入名义状态传播，每四次 IMU 后增加同时间
两个平面观测，第 60～79 次屏蔽饱和 z 陀螺轴，输出七个检查点。
该基准覆盖滤波核心，不覆盖地图、ROS 调度、初始化及大于等于 30 行的
激光观测分支；后者另有解析 Kalman 后验测试。

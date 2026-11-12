# T3：Point-LIO 观测增量连接

## 实现

新增纯标准库 `map-updates` 协议，Point-LIO每次成功扫描按自身voxel_size分组距离过滤后的全部回波，生成Arc观测批次，携带源会话、递增序号、扫描时间、帧末IMU位姿，以及逐点LiDAR世界系原点/终点。生成发生在匹配更新之后、地图插点筛选之前；没有iVox淘汰或环境删除事件。原点包含外参，使用与终点相同的采样时刻后验状态。

`terrain_map`主入口改为MapUpdate，在SurfaceSnapshot中直接使用上游体素键和分辨率，独立维护局部表面观测。无需重分组的首批样本共享Arc；后续仅合并被观测格的有限样本历史。局部窗口、老化和样本限额独立于iVox。`live_samples()`排除逐样本过期信息。可选射线占据积分默认关闭，原Frame接口保留为独立低层工具，接入观测会话后禁止混用。

Reset显式建立/重同步会话，必须先于其指向的下一观测批次传递。同会话重同步保留人工区域，从未来观测重新填充未知地图；不声称重建丢失历史。更换source/epoch清空旧地图及旧坐标系人工标注。每个Mapper最多记忆1024个源编号及其最大epoch，迟到退役源Reset不能回滚当前会话。

序号缺失会清空可用几何/占据视图，标记NeedsReset并发布完整失效快照，即使请求回执返回错误。之后必须显式重同步；重复/迟到批次只拒绝，不改变健康视图。非法批次和容量拒绝不推进序号，可在后续批次之前修正重试。

异步标注命令携带会话，执行时校验，防止排在Reset之后的旧会话命令落到新地图。默认接口捕获实际提交时的最新会话；UI应使用upsert_annotation_in_session/remove_annotation_in_session，传入实际展示快照的session，不能假设最新状态必然等于UI展示状态。

## 验证

宿主：Point-LIO 21项核心/融合、map-updates 2项协议、terrain_map 25项测试通过，共48项相关回归。fmt、workspace clippy all-targets -D warnings通过。Point-LIO既有300体素容量压力失败未修复，本轮未重跑该测试，不宣称全套测试通过。

实际ros2容器：统一colcon构建3包成功；map-updates与terrain_map共27项测试通过；端到端示例在current_thread Tokio runtime上用spawn_blocking顺序运行LIO，再经有界队列传递Arc批次。

关键案例：

- iVox限制5格时，地形视图积累49→91→133格，缓存淘汰未触发环境删除。
- iVox不增长的重复扫描仍输出49个回波，序号继续推进。
- 旋转且具有非零外参平移时，不同采样时刻原点不同，并区别于帧末原点。
- 缺失序号、重同步、换坐标系、过期、容量拒绝、迟到旧源Reset及排队标注均有回归覆盖。
- 首批几何样本Arc与生产者数组指针相同；射线积分关闭时没有额外占据块。

重复扫描测试使用非格边界采样点，避免把原iVox严格开区间插点规则误当成不增长保证；没有修改原规则以迎合测试。

## 里程计数值回归

同CSV、--ros-f32、Release，静止600帧，以及正常旋转、饱和旋转、雷达空帧恢复场景，改造后输出均与已提交results-corrected基线逐字节一致。静止场景还在改造前另行运行，前后SHA256均为a8aadbd5104a960930d2799f0a4174ed785013f77b08506b7b8207696d4ed274。

输入文件由tools/point_lio_comparison/export_data.py生成，逐场景基线路径与哈希见results/observations-replay.json。iVox map.rs和滤波filter.rs无修改。该证明限于已回放路径，不代表整套算法完成实车验收；本轮未测新观测导出带来的性能/内存增量。

## 证据及复现

- results/observations-colcon.log：实际容器统一构建。
- results/observations-tests.log：容器协议与连接回归。
- results/observations-example.log：端到端合成连接输出。
- results/observations-replay.json：既有轨迹字节一致检查。

```sh
cargo test --release --locked -p point-lio --lib --test fusion
cargo test --release --locked -p terrain-map -p map-updates
cargo run --release --locked -p terrain-map --example lio_connection
```

Point-LIO仍为同步计算核心，示例演示调用方Tokio调度；本轮没有新增完整在线IMU/扫描队列服务、ROS传感器桥接、地形特征、自动隧道或通行性算法。表面视图当前不根据可选空闲证据自动移除旧表面样本，几何层尚待融合这两类证据。

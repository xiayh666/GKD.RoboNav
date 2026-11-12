# map_updates

纯标准库的观测数据协议，供 Point-LIO 与 terrain_map 共享，不依赖 ROS、Tokio、nalgebra 或任一算法库。作为 Cargo path 依赖编译，无独立 colcon 构建任务。

- `SessionId { source, epoch }`：source 由调用方为不同估计器分配，Point-LIO reset 推进 epoch。默认构造器 source=0 适合单估计器；多估计器使用 new_with_source。
- `ObservationReset { session, next_sequence }`：显式建立接收会话，或同会话从指定未来序号重新开始未知地图。它不是完整历史快照。
- `ObservationBatch`：严格递增序号、扫描时间范围、帧末 IMU 位姿、上游体素分辨率、按键排序的观测体素。
- `ObservedVoxel`：体素键及 Arc 观测数组；每条观测带采样时间、世界系 LiDAR 原点和回波终点。
- `MapUpdate`：Reset 或 Arc 观测批次，允许异步通道转交而不复制整个点云。

这是“这一批看见了什么”的更新，不是“iVox 缓存发生了什么”的更新。没有点不代表空闲，批次中没有删除操作。Point-LIO 在插点筛选前导出全部距离过滤后回波，即使 iVox 未插入也继续输出。

在同一个生产者顺序中，先取 observation_reset 并发送，之后再 process_scan 并发送各 ScanResult.observations。不能先处理一帧、再取 reset，然后把已处理帧发给它——reset 的 next_sequence 指向下一帧。

接收端不得跳过批次后继续声称几何视图完整；丢序应显式失败并重同步。晚加入或无法重放历史时，在暂停/协调生产者的边界取新的 reset，接收端丢弃旧几何，从后续观测重新填充；不能把定位缓存快照冒充完整地形历史。

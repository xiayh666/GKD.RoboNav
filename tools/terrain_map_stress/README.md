# 主观测链路压力测试

在项目挂载于 `/workspace/GKD.RoboNav` 的 `ros2` 容器内执行：

```sh
source /home/ubuntu/.cargo/env
cargo build --locked --release -p terrain-map --example chain_stress --example capacity_stress --target-dir /tmp/terrain-map-stress-target
python3 tools/terrain_map_stress/suite.py
```

矩阵顺序运行，不相互争抢CPU。约需10分钟，具体取决于机器速度；结果写入 `doc/terrain-map/results/chain-stress/`。外部业务进程仍可能造成竞争，不能视为独占机器。

- `chain N frames Hz threads [moving]`：200Hz IMU、10Hz扫描；真实PointLio.process_scan及Tokio地形服务，默认关闭可选射线积分。三个互相垂直的平面、不同逐点时间、确定性激光与IMU噪声；moving为有解析真值的连续往复偏航。逐帧检查观测点数、状态/协方差有限性、IMU缓冲、顺序及Ready，定期检查每格样本上限。
- `direct`：同一个ObservationBatch::from_points分组验证函数和同步Mapper；分开测构造观测批与地图提交。这里的分组耗时是独立诊断，不能从另一个工作负载的LIO耗时中相减得出严格新增开销。主链路中的导出已经包含在lio_ms内。
- `burst`：无节流生产观测；try_submit_update返回Busy时携原输入等待重试。验证无静默丢序和排空。pending是未收到回执的请求数，包含正在执行和已完成但尚未处理回执的请求，不等同于内部mpsc队列长度；配置队列容量仍为2。
- `capacity_stress`：填满默认20万格表面地图，再测100次重观测提交，验证超限不提交、同序号重试、TTL清空和保留快照内容不变。它刻意使用稠密三维格点，是容量边界压力，不代表真实平面场景。
- `lifecycle`：连续平移局部窗口150m，验证窗口裁剪、样本上限、时间推进后30s TTL清空，以及旧快照内容不变。

时钟口径：

- `lio_ms`：push_imu及process_scan，含观测导出，不含输入点云生成。
- `export_rebuild_ms`：仅在隔离模式非零，包含分组、校验和样本构造。
- `admission_ms`：提交等待，有界队列背压包含在内。
- `ack_ms`：提交成功到回执，包括排队、spawn_blocking调度和地形计算；direct模式则为同步Mapper提交耗时。
- `e2e_ms`：开始处理该帧到地图完成回执，包含合成输入生成及线程调度，不包含提前输入的节流等待。
- `scheduled_ms`：从该帧应当到达的固定节拍到回执，包含上游处理不及导致的累计滞后。Hz=0模式不用于判断10Hz时限。

合成扫描模拟完整一帧已经可供处理；100ms扫描采集时间不包含在处理延迟中。固定节拍的延迟到来数据不丢弃，输入积压以scheduled_ms体现，未分配无限输入队列。每100帧回收已完成任务，可能引入轻微批边界调度效应。

每个文件组：逐帧CSV、stderr日志、0.2秒采样/proc资源CSV及JSON摘要；分位数排除前min(50,frames/10)帧，nearest-rank分位数。完整性检查覆盖所有帧且不能重号。CPU百分比100代表占用一个逻辑核，包含全部工作线程，平均值包含节流空闲；峰值RSS/HWM来自采样，可能漏掉退出前最后0.2秒。CPU累计时间也可能少最后0.2秒，短时用例资源值仅供参考。RSS包含分配器保留内存，不能只凭RSS变化判断泄漏。

测试的是合成几何输入和当前地图维护功能。隧道、坡度、台阶及通行性算法未实现，没有相关准确率；未覆盖实机传感器驱动、ROS收发、真实运动遮挡及规划控制。

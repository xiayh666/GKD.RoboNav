# 开发记录

## 2026-11-12 / 骨架
- Scope：先搭建库和构建入口，再讨论算法；不加入未经讨论的地图或通行性实现。

- Done：创建terrain-map空库、普通CMake构建入口，加入Cargo工作区及README；不引入算法、ROS封装或Point-LIO依赖。
- Verify：宿主Cargo release、fmt和workspace clippy通过；实际ros2容器colcon全工程构建成功，3 packages finished，terrain_map编译0.50s。
- Requirements：全向轮底盘，需考虑车体净空和台阶；隧道明确禁止通行，独立于几何通过能力。具体识别来源与机器人参数待讨论。
- Next：骨架构建完成；讨论三维地图表达、通行性分层与隧道禁行规则，不擅自确定识别算法。

## 2026-11-12 / T2 体素与异步并行底座
- Done：同步Mapper、8³稀疏块、逐射线DDA、未知/自由/占用证据与老化/局部窗口；帧内私有Rayon池并行、块位图合并、命中优先的确定性提交；COW快照。
- Done：调用方Tokio runtime、有界mpsc、非阻塞拒绝返还原帧、回执、watch完整版本快照、人工提示/强制禁行框管理、关闭排空。最多一个blocking计算在途，内部CPU工作并行，不创建全局runtime或全局Rayon池。
- Fix：吞吐场景发现精确负方向格边界端点可能被DDA跨越，增加归一化t=1终止并补充回归。
- Verify：9地图+4运行测试在宿主及实际ros2容器均通过；fmt/workspace clippy -D warnings通过；online示例及colcon3包构建通过。无修改Point-LIO算法。
- Performance：容器独立Release测量，5个稳态样本：10000射线1/2/4线程中位38.986/22.287/15.322ms；20000射线75.109/41.132/27.208ms。完整测试范围和限制见validation.md，原始记录在results/。
- Decision：Tokio负责调度，Rayon负责帧内计算，提交保持串行。当前Tokio最低Rust要求1.85，更新terrain-map元数据，容器1.99.1满足。队列不静默丢帧，超限整帧回滚；人工提示不是自动隧道判断。
- Next：本阶段底座完成；后续扩展表面几何统计、地形与隧道识别，以及Point-LIO射线原点适配。尚未输出自动通行结论，未完成整个地形分析功能。

## 2026-11-12 / T3 Point-LIO观测增量接入
- Done：新增纯std map-updates协议；Point-LIO按源体素分组全部距离接受回波，在插点筛选前导出逐点世界原点/终点、帧末位姿、会话及序号。失败不消费序号，reset推进epoch。
- Done：terrain_map主入口MapUpdate，直接复用源键和Arc样本，独立表面几何视图；射线占据积分默认关闭。丢序显式失效并要求Reset；旧源回滚与排队旧会话人工标注受保护。同会话重同步保留标注，新坐标系清空旧标注。
- Verify：宿主48项相关回归、fmt/clippy通过；实际ros2容器27项协议/地形测试及colcon3包构建通过；5格iVox示例地形视图49/91/133格。静止/旋转/饱和旋转/恢复4场景回放与既有基线字节一致，filter.rs/map.rs未改。详见observation-interface.md与results/observations-*。
- Decision：上游导出观测，不把LRU淘汰作为环境删除。保留legacy world_points及低层射线接口，不假称整个生产流程零拷贝或已经完成隧道分析。
- Next：接口阶段完成，后续实现表面几何、地形与隧道识别；真实传感器接入及本次额外导出开销仍需验证。


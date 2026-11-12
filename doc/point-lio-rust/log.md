# 开发记录

## 2026-11-11
- 用户将需求从 C++ 移植改为 Rust 重写。
- 已核对官方 30 维状态、动力学雅可比、IMU 饱和轴判断、点到平面观测和滤波实现。
- 采用 Rust 原生线性代数；仅保留官方源码作为参考，不链接其实现。
- 当前开始 T2，待独立编译和合成测试。

## T2 checkpoint
- 完成 30 维 output-state EKF、SO(3) 传播/重置、饱和轴屏蔽、外参雅可比、
  五近邻 PCA、容量有界地图与 IMU 队列、事务式逐点融合接口。
- `cargo test --workspace --release`：4 项单元测试和 7 项集成测试通过。
- `cargo clippy --workspace --all-targets -- -D warnings` 通过。
- T3 已补 CSV 回放和说明，继续验证激光抑制漂移以及队列回压行为。

## T3 checkpoint
- 完成 README、原作者 BSD 许可证/NOTICE、库接口说明与 CSV 回放程序。
- 作者 C++ 原始包添加 `COLCON_IGNORE`，避免 ROS 2 工作区误构建；Rust 不链接它。
- `cargo fmt --all -- --check`：通过。
- `cargo clippy --workspace --all-targets -- -D warnings`：通过，无警告。
- `cargo test --workspace --all-targets --release`：13 项测试全部通过。
  带外参合成轨迹验证平移误差 < 8cm、姿态误差 < 0.04rad；
  加计误差对比实验验证纯惯性漂移 > 10cm，激光融合误差 < 纯惯性的 25%。
- `cargo run --release -p point-lio --example replay`：三帧演示成功，后两帧各匹配 169 点。
- CSV 文件入口使用 `/tmp/gkd-point-lio-replay.csv` 验证两帧回放，第二帧匹配 169 点。
- 本次 Rust 核心实现范围内验收完成。保留的明确差异：单一 30 维 output-state
  模型、体素哈希/PCA、标量顺序更新、Joseph/SO(3) 重置、逐事件协方差调度。
- 后续实机工作需要传感器型号、标定、统一时间与真实记录；尚未完成真实数据对比、
  24 维可选分支、ROS/Nav2 桥接或论文性能复现。未将这些标记为已完成。

## T4 场景验证与 T5 官方对照准备
- 用户要求先验证，不急于修改；只新增 tests/usability.rs 和回放参数，不修改核心算法。
- 已执行 5 项场景检查：默认地图含噪静止、退化刻画、空回波恢复通过；
  300 体素容量静止和旋转正常量程对照失败。保留失败断言，不放宽门限。
- 默认容量静止复测 60s 最大位置误差 0.020259m；300 容量为 0.363425m。
- 手动独立耗时测量 1000/10000/20000 点中位耗时 26.351/258.096/509.290ms。
- 详细输入、阈值和限制已记录 validation.md；不再将原 13 项通过作为完整可用性结论。
- 用户随后要求相同数据运行官方实现。已下载 ROS Noetic 镜像和 Livox 官方消息，
  构建隔离环境，准备完整 ROS 节点回放；未修改官方滤波和地图实现。
- 共享 CSV 由同场景公式导出，两侧均实际读取该文件；Rust 可选转换到官方 float32 点精度。

## T5 完整官方程序同数据对照完成
- 在隔离 ROS Noetic / Ubuntu20.04 / PCL1.10 / GCC9.4 容器编译完整官方节点。
  Livox 仅生成官方消息定义；作者 src/include/CMakeLists.txt 与检出逐文件核对一致。
- 官方完整回放完成：静止 600 帧、旋转正常/裁剪各 30 帧、地面 30 帧、
  恢复 25 帧、1000/10000/20000 点各 6 帧。启动首帧及空扫描输出行为差异已说明。
- 使用同一 CSV。Rust 可选 float32 输入精度对齐，物理参数与点筛选对齐；
  地图、初始化与滤波调度保持各自实现。输入 SHA256 与所有参数已保存。
- 正常旋转最大姿态误差：官方 0.010936rad / Rust 0.101525rad；
  裁剪旋转官方 0.013476rad / Rust 0.108242rad。
- 静止最大位置误差：官方 0.025459m / Rust 0.020259m；
  未宣称所有指标官方占优。
- 1 万点处理参考中位耗时：官方内部 55.426ms / Rust 完整 258.096ms。
  计时边界不同，未宣称严格加速比；2 万点两者均超过100ms。
- 数据、轨迹、原始官方日志、版本与参数见 results/*；完整解释见 official-comparison.md。
- Rust 核心算法和官方算法均未修改。原 13 项回归仍通过；Clippy、格式检查、
  Python 语法及 shell 语法检查通过；新增场景测试的两项失败保持可复现。
- 结论：当前 Rust 重写不能视为官方等价实现，也未通过工程可用性验收。
  本轮用户要求的测试与官方对照工作完成；后续算法修改等待用户指示。

## 2026-11-12 / T6 按原理与源码修正
- 已获用户明确授权修正 Rust 实现；以官方源码逐项恢复传播/更新机制。
- 分离逐点名义状态时钟和 IMU 协方差时钟，消除按点密度切碎 Q*dt² 的错误。
  IMU 六轴和同时间激光点联合更新；恢复 P-KHP，单点秩一更新避免 Joseph 稠密计算。
- 恢复多点体素、26 邻域、按写入维护 LRU、缓存邻居后的帧末占用判定、列主元 QR。
  初始点跨帧累计到默认10个后建立地图，修复稀疏首帧锁死同体素增量的问题。
- 直接编译作者 ESEKF 头文件和原始动力学/IMU 函数生成数值基准；7 检查点完整
  名义状态与900项P共6524数值，最大绝对差1.3243746377344934e-14（阈值1e-9）。
- 增加点时间细分不改变过程噪声、批量解析后验、同时间输入顺序不影响后验、
  多点体素/LRU/稀疏启动等回归；9单元+11基础融合回归通过，Clippy和格式检查通过。
- 所有共享CSV的SHA256与历史官方输入一致；重放当前Rust并按共同输出帧比较。
  正常旋转最大姿态误差0.008048rad（此前0.101525，官方0.010936）；
  裁剪旋转0.011530rad（此前0.108242，官方0.013476）。
  恢复共同帧最大位置误差0.000400m，官方0.000319m。
- 默认容量60s静止最大位置0.026380m、姿态0.009103rad，通过原阈值；
  300体素压力测试仍失败，位置1.642475m，未删除、忽略或放宽阈值。
  完整测试结果：9单元通过、11融合通过、场景4通过1失败1手动计时忽略。
- 原floor fixture重复同一坐标三次，导致五邻居可能不足以定义平面；
  保留历史CSV对照，退化回归改用唯一二维地面采样，保持原断言。
  QR秩亏基本解仍按官方方式处理，不把其较小漂移当作可观性证明。
- 耗时中位数1000/10000/20000点为4.661/45.500/107.182ms；2万点仍不满足此10Hz检查。
- 大于等于30行激光观测采用一致方差解；官方大小批量权重不一致未复制。
  初始化、精度、秩亏QR及容量边界仍有差异，未宣称整个程序等价或实车已验收。
- 原始基线不覆盖，最新证据见correction.md及results-corrected/。

## 2026-11-12 / T7 独立地图尺度
- Done：新增 Config.filter_size_map，独立于索引分辨率 voxel_size；MapIncremental 按筛选尺寸计算中心和占用边界，索引查询仍使用 voxel_size。两者默认 0.3m，默认行为不变。
- Verify：10 单元、11 融合测试通过；追加无效筛选尺寸（0、负数、NaN、无穷）校验后定向复验通过；fmt 和 clippy --all-targets -D warnings 通过。新增回归覆盖筛选格大于/小于索引格及正负坐标。
- Decision：保留现有 voxel_size 字段名作为索引尺寸，新增独立字段；初始地图直插及缓存近邻筛选语义不变。
- Next：本问题已完成；未重跑整套压力场景，已有 300 体素预算失败仍未解决，不宣称全套验收通过。

## 2026-11-12 / 拆分体素尺度后压力复测
- Release 构建，测试单线程串行执行；保留默认 voxel_size=filter_size_map=0.3m，无算法或测试阈值修改。
- 耗时：每档排除首帧建图，统计后续 5 帧完整 process_scan。1000 点中位 4.363ms（4.297～4.441），10000 点 46.867ms（44.377～50.225），20000 点 106.777ms（91.833～114.009）。所有点数均逐帧全部匹配。相比前次 4.661/45.500/107.182ms，没有稳定提速证据；2 万点仍无法保证 10Hz。
- 可用性压力：4 通过、1 失败、1 耗时用例默认忽略（已单独显式运行）。默认 100000 体素含噪静止 60s 最大位置 0.026380m、姿态 0.009103rad；300 体素仍为 1.642475m、0.305982rad，原断言失败。
- 饱和旋转最大角误差 0.011530rad；断点恢复最终位置误差 0.000418m；单平面退化检查通过。
- 证据：results-corrected/timing-retest.log、pressure-retest.log。统计仅 5 个稳态帧/档，不能当作长期尾延迟保证。本轮只复测，没有进行新优化。

## 2026-11-12 / T8 官方容量压力补测
- Done：在临时官方源码副本仅增加 comparison_capacity 参数入口，完成官方 100000/300/301 与 Rust 100000/300 同 CSV 回放。正式官方仓库算法文件不变；副本补丁、轨迹、参数、节点日志及报告保存 results-capacity。
- Verify：三档官方均完成 600 帧输入、599 帧输出，Rust 600 帧输出；共同 599 帧统计。官方 301（实际最多300格）最大位置 2.196736m、角度0.372025rad；Rust300 1.572311m、0.298405rad，均未通过原位姿阈值。大容量官方0.025459m、Rust0.026380m，均通过。
- Decision：保留官方 >=capacity 淘汰语义，同时测试300/301；比较统一 float32 点输入，Rust先前原生 f64 压力误差不同。没有用本结果声称整体等效或精度领先。
- Next：容量对照已完成；小地图稳定性问题仍存在于双方本场景。

## 2026-11-12 / T9 目录整理
- Done：按用户要求删除项目内官方 C++ src/point_lio（含嵌套 .git），将原 Rust 目录更名为 src/point_lio；Cargo 工作区与有效文档路径同步。Cargo 包名保持 point-lio。
- Done：增加验证资料总索引，保留历史轨迹、容量对照、许可证及官方数值基准。官方基准生成器改为要求 --upstream 外部源码路径，更新复现说明；清理工具和验证目录的 Python 缓存。
- Verify：fmt、clippy --all-targets -D warnings、10 单元+11 融合测试通过；Python AST、shell语法、生成器help、目录和旧目录名引用检查通过。场景复验4通过、1已知失败、1耗时用例忽略；300格误差仍1.642475m/0.305982rad，无算法或阈值修改。
- Decision：保留已知失败断言与所有原始证据，目录整理不宣称算法验收通过。后续工作以验证资料索引为入口。
- Next：用户要求的目录整理已完成，停止本轮算法迭代。

## 2026-11-12 / T10 普通 CMake 接入工程编译
- Done：src/point_lio/CMakeLists.txt 作为无编程语言的普通 CMake 项目，ALL 目标调用 Cargo --lib --locked，默认 release，可配置 profile/target-dir。不引入 package.xml 或 ament；安装阶段只安装说明及许可证。
- Verify：colcon list 识别 point_lio 为 cmake；实际 colcon build --packages-select point_lio 成功生成 build/point_lio/cargo/release/libpoint_lio.rlib。重复构建 Cargo 0.01s，整个包0.11s。临时独立配置 CARGO_EXECUTABLE=/usr/bin/false 验证编译失败正确传递至 CMake（返回2）。
- Environment：宿主Python已为3.14，colcon模块仍安装在/usr/lib/python3.13/site-packages；本次临时设置 PYTHONPATH 指向该目录运行，未修改系统安装。宿主无/opt/ros，未验证ROS bringup整体构建。
- Decision：不安装孤立rlib作为C++库，不新增FFI；Rust消费者继续Cargo path依赖。默认统一构建入口为colcon build，在有效ROS2/colcon环境中同时调度bringup及Rust库。
- Next：本次构建接入完成。

## 2026-11-12 / T11 实际 ros2 容器验证
- Context：用户明确实际colcon在ros2容器内执行；宿主Python问题不属于实际工程环境。
- Done：启动原ros2容器，在ubuntu用户目录用官方rustup安装minimal stable（rustc1.99.1），未修改shell启动文件。CMake增加CARGO_HOME/HOME下cargo搜索提示，避免依赖shell PATH。Cargo调用清除无法继承描述符的MAKEFLAGS/MFLAGS，消除jobserver警告。
- Verify：在/workspace/GKD.RoboNav source /opt/ros/jazzy/setup.bash，实际执行colcon --log-base log/ros2 build --build-base build/ros2 --install-base install/ros2；首次2包构建成功12.5s（含Cargo依赖下载），增量2包成功0.27s，Cargo0.01s。gkd_nav_bringup与point_lio均完成。
- Decision：宿主与容器缓存隔离到build/ros2、install/ros2，避免CMake绝对路径冲突；现有用户缓存不删除。工具链位于当前容器用户目录，重建容器时需重新安装；工作区不捆绑Rust二进制。
- Next：实际工程环境构建接入验证完成。

## 2026-11-12 / 版本管理整理
- Done：补全.gitignore，忽略Rust/colcon/CMake/Python产物、本地凭据及原始传感器录制；保留Cargo.lock、数值fixture与整理后的验证证据。
- Done：仓库此前无提交，按忽略规则、导航基础包、Point-LIO、terrain_map、对照工具、设计验证资料分类建立中文提交。Point-LIO初始提交使用单成员工作区，后续提交扩展terrain_map，保持代码提交可独立构建。
- Verify：忽略规则正反例通过，格式检查通过；Point-LIO 21项核心/融合回归与terrain_map 13项回归通过。未重跑完整可用性压力测试，已有300体素失败记录原样保留。
- Scope：本轮仅整理版本；Point-LIO到terrain_map的观测增量接口尚未实施。

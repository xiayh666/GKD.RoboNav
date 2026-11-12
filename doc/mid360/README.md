# MID-360 官方驱动与可视化

使用官方仓库：

- ROS驱动：https://github.com/Livox-SDK/livox_ros_driver2 ，子模块固定 `21445540f0d100dc86a7e6df312dd70bbdb4afdf`，程序版本1.2.8。
- 底层SDK：https://github.com/Livox-SDK/Livox-SDK2 ，构建脚本固定 `c0796f04c143143899c87a773d9f6b7136453c0b`。

官方ROS2例程在 `src/livox_ros_driver2/launch_ROS2/`：

| 例程 | 用途 |
|---|---|
| msg_MID360_launch.py | CustomMsg逐点时间点云与Imu |
| rviz_MID360_launch.py | PointCloud2及官方RViz显示配置 |

这些是设备接收/显示例程；本次未发现专门的自动硬件验收测试目录，官方CMake测试入口为ament lint。项目另外提供只读、限时的 `tools/mid360_probe.py` 检查接收数据。

## 当前网络

主机网卡enp5s0 / RTL8125，100Mbps全双工，雷达192.168.12.3，电脑192.168.12.100/24。配置文件为 `src/gkd_nav_bringup/config/MID360_config.json`。电脑地址目前是运行时配置，原NetworkManager配置仍是SharedToEthernet；重连/重启后应重新检查地址，必要时执行 `nmcli device modify enp5s0 +ipv4.addresses 192.168.12.100/24`。Wi-Fi维持原连接。

## 构建

已有ros2容器为Jazzy，项目挂载 `/workspace/GKD.RoboNav`，网络模式host。首次需要容器内PCL、pcl_conversions、ament_cmake_auto和libapr1-dev；本机此次仅缺libapr1-dev，已安装。新容器可先执行：

```sh
docker exec -u 0 ros2 bash -lc 'apt-get update && apt-get install -y libpcl-dev libapr1-dev ros-jazzy-pcl-conversions ros-jazzy-ament-cmake-auto'
bash tools/build_livox.sh
```

脚本初始化官方子模块、生成上游忽略的ROS2 package.xml，将SDK构建产物安装到容器/usr/local，使用现有build和install统一构建驱动、bringup和两个Rust库。上游源码保持干净。驱动的Jazzy参数写在根目录colcon.meta，后续常规colcon也能编译；SDK安装及ROS2 package.xml的首次准备仍需脚本。未运行会清空整个工作区build/install的官方build.sh。

独立Rust Point-LIO及terrain_map不受驱动PCL依赖影响。全工程统一colcon构建实测4个包完成，见results/workspace-build.log。

## 打开官方可视化

从宿主图形桌面的项目目录执行：

```sh
bash tools/view_mid360.sh
```

此入口启动项目的 `ros2 launch gkd_nav_bringup mid360_rviz.launch.py`，内部使用官方驱动节点和原版RViz显示配置，自动选择PointCloud2。项目JSON直接传入节点，不覆盖官方源码或install目录中的配置文件。脚本将当前会话XAUTHORITY副本刷新到容器用户标准位置/home/ubuntu/.Xauthority，容器内直接启动也可认证，不放开整个X服务器访问权限。避免同时启动多个驱动争抢UDP端口。此前已直接运行官方rviz_MID360_launch.py完成例程验证，记录见results/official-rviz.log。

不打开界面、使用项目独立配置：

```sh
docker exec -it -w /workspace/GKD.RoboNav ros2 bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch gkd_nav_bringup mid360.launch.py
# PointCloud2及同一官方显示配置：
# ros2 launch gkd_nav_bringup mid360_rviz.launch.py
```

另一个已source的容器终端可执行 `python3 tools/mid360_probe.py --seconds 10`；PointCloud2模式加 `--format pointcloud2`。

## 实机验证：2026-11-12

| 模式 | 点云接收Hz | IMU接收Hz | 每帧点数 | 检查结果 |
|---|---:|---:|---:|---|
| CustomMsg，约10秒 | 10.002 | 199.941 | 19872–20064 | 时间单调、点数一致、header=timebase、点/IMU有限值通过 |
| PointCloud2，约10秒 | 9.908 | 200.189 | 19776–20064 | 时间单调、数据尺寸及x/y/z/timestamp字段检查通过 |

CustomMsg逐点offset跨度0.0991–0.1005秒；IMU最大相邻时间差分别6.16/6.35ms。探针使用best-effort、深度256，频率为首末消息接收时间估计，短窗口含发现/调度影响，不代表长期无丢包证明。最初深度5探针观测到约175Hz，增大探针队列后复测接近200Hz；不据此前者判定雷达频率异常。

官方rviz_MID360_launch.py已实际启动验证，窗口创建、OpenGL初始化及/livox/lidar订阅成功；工程集成后使用mid360_rviz.launch.py管理同一官方驱动和显示配置。最初X11 cookie无效导致RViz启动失败，复制当前桌面XAUTHORITY后恢复；不是驱动或雷达连接故障。原始日志和JSON在results/。

这版官方驱动的Imu加速度原样发布为g，实测模长约0.995（源码comm/lidar_imu_data_queue.h及lddc.cpp）；后续Rust适配需转换m/s²。CustomMsg逐点offset_time为纳秒，需转换秒。当前仅完成驱动和显示，尚未接入实机Point-LIO融合，也未测几何精度或完成外参标定。

## 工程集成验收（M2）

- 默认mid360.launch.py：`/livox/lidar`为CustomMsg，`/livox/imu`为Imu，10Hz点云；xfer_format只允许0或1。
- 参数：config、publish_freq、lidar_topic、imu_topic；使用固定livox_frame保持点云与官方硬编码的IMU frame一致。真实外参仍待后续适配，不发布虚假的TF。
- mid360_rviz.launch.py固定选择PointCloud2，RViz订阅随lidar_topic重映射。关闭RViz窗口不等于关闭驱动，在启动终端Ctrl+C停止整组进程。
- 实机话题重映射8秒：点云10.004Hz、IMU199.997Hz，80帧点云、1594条IMU，消息检查通过，证据results/project-remap-receive.json。
- 工程可视化入口实际启动后再次检查PointCloud2及IMU接收，见results/project-rviz-receive.json与project-rviz.log。
- tools/build_livox.sh已重复执行，四包构建通过，colcon.meta按包注入Jazzy参数；没有修改SDK或驱动算法。本阶段按用户确认仅做驱动工程集成，尚未连接Rust Point-LIO。

话题重映射示例：

```sh
ros2 launch gkd_nav_bringup mid360.launch.py lidar_topic:=/mid360/lidar imu_topic:=/mid360/imu
```

## 标准构建目录（M4，替代M3的子目录隔离）

容器内直接 `colcon build`，使用标准build/、install/、log/；环境入口为 `source install/setup.bash`。旧宿主绝对路径缓存和此前ros2子目录产物已移到临时备份/tmp/gkd-colcon-old.FxJ9hR。标准目录全量重建四包通过（21.2秒），source install/setup.bash后两个launch均可解析，证据为results/standard-colcon-build.log和standard-launch-arguments.log。切换宿主/容器构建环境时必须重新生成CMake缓存，不能沿用另一环境的绝对路径。colcon_defaults.yaml已移除，colcon.meta仅保留官方驱动的Jazzy构建参数。

如果重新登录宿主桌面后出现MIT-MAGIC-COOKIE错误，在宿主运行tools/view_mid360.sh会刷新认证。当前容器中可直接ros2 launch，不需要临时设置XAUTHORITY。

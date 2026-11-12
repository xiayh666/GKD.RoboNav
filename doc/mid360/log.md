# 接入记录

## 2026-11-12 / M1
- Done：官方driver2子模块、固定SDK构建脚本、项目网络JSON与launch、Jazzy colcon.meta；宿主GUI启动脚本及限时接收探针。上游源码未改。
- Verify：SDK和驱动构建通过；脚本重复执行通过；4包colcon统一构建通过；10秒CustomMsg与PointCloud2实机接收检查通过，点云约10Hz、IMU约200Hz。详见README及results。
- Verify：官方rviz_MID360_launch.py已实际打开1853x1025窗口并订阅点云；当前保持运行。修复容器X11 cookie认证，不修改官方显示配置。
- Decision：从官方build.sh提取必要ROS2准备，避免清除已有工作区产物；设备IP由独立配置维护。官方Imu加速度单位g，后续适配必须转换。
- Next：用户查看官方点云；后续再接Rust Point-LIO，当前不启动融合。

## 2026-11-12 / M2 工程集成
- Scope：用户确认先完成驱动工程集成，不在本轮连接Rust Point-LIO。
- Done：统一构建脚本改为编译四包；独立驱动入口默认CustomMsg，支持网络JSON/频率/话题参数及格式约束；独立mid360_rviz入口固定PointCloud2并使用官方RViz配置，支持点云话题重映射。宿主查看脚本不再覆盖install配置。
- Verify：脚本四包构建通过；launch参数解析通过；8秒自定义话题实机接收约10.004Hz点云和199.997Hz IMU，结构及时间检查通过；工程RViz入口窗口创建且PointCloud2及IMU接收检查通过，结果见project-*.json/log。shell语法、Python编译及git diff --check通过；官方子模块干净。
- Decision：固定livox_frame，避免暴露仅能改变点云而不能改变上游硬编码IMU帧名的参数；不修改官方驱动。
- State：当前运行工程mid360_rviz.launch.py及其官方驱动节点，供用户查看；新的headless实例需先停止该组进程，以免UDP端口冲突。

## 2026-11-12 / M3 直接构建缓存冲突
- Cause：此前仅脚本及显式命令选择build/ros2，裸colcon build仍复用了宿主build/point_lio绝对路径缓存。
- Done：添加工作区colcon_defaults.yaml，自动隔离容器build/install/log；test同样选择容器build目录。保留旧缓存，更新环境加载及宿主覆盖目录说明。
- Verify：容器项目根目录仅source ROS环境后直接colcon build，四包成功，2.96秒；CMAKE_HOME_DIRECTORY为/workspace/GKD.RoboNav/src/point_lio，日志位于log/ros2；git diff --check通过。

## 2026-11-12 / M4 恢复标准目录
- Requirement：用户要求不再使用ros2子目录隔离，恢复标准colcon布局。
- Done：移除colcon_defaults.yaml，构建及查看脚本改为标准目录，更新当前使用说明。停止旧目录启动的驱动/RViz后，将旧包缓存、安装树及log/ros2移到/tmp/gkd-colcon-old.FxJ9hR可恢复备份，保留SDK源码和SDK构建缓存。
- Verify：清理构建环境后容器直接colcon build，四包21.2秒通过；source install/setup.bash后包前缀正确为/workspace/GKD.RoboNav/install/gkd_nav_bringup，两个launch参数解析通过；setup.bash父链只含/opt/ros/jazzy，无旧工作区；build/ros2、install/ros2、log/ros2均不存在，shell语法及git diff --check通过。
- State：已使用标准install下的工程入口重新启动官方驱动及RViz窗口。历史M3记录保留为问题过程，当前操作以M4和README为准。

## 2026-11-12 / M5 容器直接启动RViz认证
- Cause：图形脚本先前仅用临时XAUTHORITY环境变量授权它启动的进程，容器用户默认.Xauthority过期，直接ros2 launch触发MIT-MAGIC-COOKIE错误及Qt abort(-6)。
- Done：保留旧.Xauthority备份，复制当前宿主会话认证到容器/home/ubuntu/.Xauthority标准位置；更新宿主查看脚本每次刷新该文件，不需要修改用户Bash/fish配置或放开X服务器。清理遗留launch以免多驱动抢端口。
- Verify：不传XAUTHORITY的容器Bash直接启动mid360_rviz.launch.py，OpenGL4.6、点云订阅成功；5秒限时测试随后主动SIGINT，两个子进程干净退出，timeout返回124符合预期。证据results/direct-rviz-auth.log。shell语法及diff检查通过。

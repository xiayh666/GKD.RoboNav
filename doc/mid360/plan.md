# MID-360 接入计划

| id | title | owner | files | status | depends_on | verify |
|---|---|---|---|---|---|---|
| M1 | 固定官方驱动、SDK及本机配置，验证接收例程 | codex | .gitmodules, src/livox_ros_driver2, bringup/config及launch, tools, doc/mid360 | completed | 网口链路已恢复 | ros2容器colcon构建；官方节点实际连接MID360并收到自定义点云与IMU；记录频率/时间字段和复现命令 |

| M2 | 整理工程驱动入口与可配置话题 | codex | bringup launch/config, tools/build_livox.sh, README及文档 | completed | M1 | 容器构建通过；launch参数可查看；项目入口发布点云/IMU并通过限时接收检查；官方源码干净 |

| M3 | 修复直接colcon build复用宿主缓存 | codex | colcon_defaults.yaml, README及接入文档 | completed | M2 | 容器从项目根目录直接执行colcon build，四包通过并写入build/ros2及install/ros2 |

| M4 | 恢复标准colcon目录布局 | codex | 构建默认配置、脚本、README及旧缓存 | completed | M3 | 容器裸colcon build四包通过；source install/setup.bash后两个launch可解析；无活动ros2子目录路径 |

| M5 | 修复容器终端直接启动RViz的X11认证 | codex | tools/view_mid360.sh, 容器Xauthority及文档 | completed | M4 | 不传XAUTHORITY的容器Bash直接启动工程RViz，OpenGL/订阅成功，限时SIGINT后干净退出 |

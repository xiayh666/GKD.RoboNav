# MID-360 接入目标

- 使用Livox-SDK官方livox_ros_driver2和Livox-SDK2，不选第三方fork。
- 使用当前ros2/Jazzy容器，通过有线网口接收MID-360点云及内置IMU。
- 验证官方接收例程并提供RViz例程入口；本阶段不自动启动Point-LIO、不改变其算法，不宣称完成实机里程计融合。

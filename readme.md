本仓库记录在光交澳实习的过程中在树莓派4B上部署的代码
Requirements:
Ubuntu mate 20.04
Python3
Pyserial
ros neotic
rplidar_ros
hector_slam

运行代码前需先运行两个ROS节点：
roslaunch rplidar_ros rplidar_u1.launch        启动思岚u1水下激光雷达的雷达驱动
roslaunch hector_mapping mapping_default.launch

详见：https://iib0j7117e.feishu.cn/wiki/LuDkwCMGtirwFWkfEw1cwUO0nqg?from=from_copylink
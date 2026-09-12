#!/usr/bin/env bash
# 清理 mid360 链路残留进程。
#
# 背景:livox_ros_driver2_node 在 launch 被 Ctrl-C 后会残留(C-livox SDK
# 忽略 SIGTERM,launch 的进程号转发管不到它)。残留实例占着雷达 UDP 连接,
# 新起的驱动全部 bind failed → point_lio 拿不到点云 → 不发布 tf
# (camera_init→body) → rviz 报 "Frame [camera_init] does not exist"、
# octomap_server 卡在等 transform。
#
# 报错时先跑这个,再重新启动:
#   bash src/mid360_bringup/scripts/cleanup.sh
#   ros2 launch mid360_bringup mid360.launch.py
#
# 只杀本链路相关进程,不影响机器上其他 ROS 栈。
# 用 [x] 括号写法避免 pkill 匹配到本脚本自己的命令行。

pkill -9 -f 'livox_ros_driver2_no[d]e' 2>/dev/null   # livox 驱动
pkill -9 -f 'pointlio_mappi[n]g'       2>/dev/null   # point_lio 建图
pkill -9 -f 'octomap_server_no[d]e'    2>/dev/null   # octomap_server
pkill -9 -x rviz2                      2>/dev/null   # rviz2

sleep 1
left=$(ps -eo cmd | grep -cE 'livox_ros_driver2_no[d]e|pointlio_mappi[n]g|octomap_server_no[d]e' || true)
if [ "$left" -eq 0 ]; then
    echo "清理完成,重新启动: ros2 launch mid360_bringup mid360.launch.py"
else
    echo "还有 $left 个残留进程,请人工核对: ps -eo pid,cmd | grep -E 'livox|pointlio|octomap'"
fi
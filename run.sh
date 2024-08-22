#!/bin/bash

gnome-terminal -x bash -c "
source /opt/ros/noetic/setup.bash
roscore
" & sleep 1

# ====================设备端运行====================

gnome-terminal -x bash -c "
source /home/han/workspace/FastLio2B_ws/devel/setup.bash
roslaunch fast_lio mapping_mid360.launch
" & sleep 1

gnome-terminal -x bash -c "
source /home/han/data/workspace2/OccupiedMap_ws/devel/setup.bash
roslaunch OccupiedMap lidar_log.launch
" & sleep 1

gnome-terminal -x bash -c "
source /home/han/Project/QT_display/yjzb_qt_yt/devel/setup.bash
/home/han/Project/QT_display/yjzb_qt_yt/devel/lib/mapping/mapping
" & sleep 1

# ----------------------编码----------------------
gnome-terminal -x bash -c "
python3 Encode/SaveAndEncode.py
" & sleep 1


# =====================传输====================

# gnome-terminal -x bash -c "
# python3 4G/4G_receive.py
# " & sleep 1

# gnome-terminal -x bash -c "
# python3 4G/4G_send.py
# " & sleep 1


# ====================服务器端运行==================== 

gnome-terminal -x bash -c "
source /home/han/Project/pcd3map/devel/setup.bash
roslaunch pcd3map pcd3map.launch
" & sleep 1

# --------------------解码----------------------
gnome-terminal -x bash -c "
python3 Encode/Decode.py
" & sleep 1

gnome-terminal -x bash -c "
python3 Encode/DataPublisher.py
" & sleep 1
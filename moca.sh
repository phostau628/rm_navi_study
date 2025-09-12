#!/bin/bash
source /home/auto/rmnavi_1/install/setup.bash
source_common="source /opt/ros/humble/setup.bash;source /home/auto/rmnavi_1/install/setup.bash;export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"
source ~/.bashrc
source /opt/ros/humble/setup.bash

gnome-terminal --window --title=""\
--tab --title="livox_ros_driver2" -e "bash -c '$source_common;ros2 launch livox_ros_driver2 msg_MID360_launch.py; exec bash'" \
--tab --title="point_lio_cxr" -e "bash -c '$source_common;ros2 launch point_lio_cxr RMUC.launch.py; exec bash'" \
--tab --title="rviz" -e "bash -c '$source_common;rviz2'" \
--tab --title="remove_pointcloud" -e "bash -c '$source_common;ros2 launch remove_pointcloud remove_pointcloud.launch.py; exec bash'" \
--tab --title="navi_cal" -e "bash -c '$source_common;ros2 run navi_cal navi_cal_node; exec bash'" \
--tab --title="para" -e "bash -c '$source_common;ros2 param set /Navi_cal pit_bias 0.20; exec bash'" \
--tab --title="serial_driver" -e "bash -c '$source_common;ros2 run serial_driver serial_driver_node; exec bash'" \
# --tab --title="point_lio_cxr" -e "bash -c '$source_common;sleep 3; ros2 launch point_lio_cxr RMUC_left.launch.py; exec bash'" \
# --tab --title="pointcloud_cough" -e "bash -c '$source_common;ros2 launch pointcloud_cough pointcloud_cough.launch.py'" \
# --tab --title="rm_navigation" -e "bash -c '$source_common;ros2 launch rm_navigation RMUC.launch.py; exec bash'" \
# --tab --title="livox_ros_driver2" -e "bash -c '$source_common;ros2 launch livox_ros_driver2 msg_MID360_launch.py; exec bash'" \
# --tab --title="pointcloud_to_laserscan" -e "bash -c '$source_common;ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py;exec bash'" \
# --tab --title="remove_pointcloud" -e "bash -c '$source_common;ros2 launch remove_pointcloud remove_pointcloud.launch.py; exec bash'" \
# --tab --title="send_goal_" -e "bash -c '$source_common;ros2 launch send_goal_ send_goal.launch.py; exec bash'" \
# --tab --title="pid_follow" -e "bash -c '$source_common;ros2 launch pid_follow pid_follow.launch.py; exec bash'" \
# --tab --title="rm_vision_bringup" -e "bash -c 'source ~/github_code/rmvision_ws/install/setup.bash;ros2 launch rm_vision_bringup test.launch.py ; exec bash'" \
# --tab --title="omnidirectional_perception" -e "bash -c '$source_common;ros2 launch omnidirectional_perception omnidirectional_perception.launch.py; exec bash'" \
#--tab --title="auto_decision" -e "bash -c '$source_common;ros2 run auto_decision_RMUC navigation_check_node; exec bash'" \
#--tab --title="rm_vision_bringup" -e "bash -c 'source ~/github_code/rmvision_ws/install/setup.bash;ros2 launch rm_vision_bringup three_camera.launch.py ; exec bash'" \
--hide-menubar

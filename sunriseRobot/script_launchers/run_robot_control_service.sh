#! /bin/bash

###############################################################################
# Entry point of robot_control.service (see sunriseRobot/systemd/).
# Sets up the ROS2 environment, then replaces itself with the main app (exec),
# so systemd supervises the Python process directly: signals, exit code and
# restarts all apply to it.
# Not meant to be run by hand while the service is active (two instances would
# fight over the serial port and GPIO): use `systemctl restart robot_control`.
###############################################################################

source /opt/ros/foxy/setup.bash
source /root/marco_ros2_ws/install/local_setup.bash
source /root/software/library_ws/install/local_setup.bash

# systemd does not read ~/.bashrc. Same domain as the interactive shells (see
# info/connection_info.txt), so `ros2 topic list` from an SSH session sees the
# robot's nodes. ROS2 processes started by the app inherit it.
export ROS_DOMAIN_ID=32

exec python3 /root/GIT/yahboom_rdk_x3_robot/sunriseRobot/app_SunriseRobot/main_thread.py

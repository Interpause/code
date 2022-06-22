#! /bin/sh
git config --global --add safe.directory /code
. /opt/ros/$ROS_DISTRO/setup.sh
rosdep install -i --from-path /code/src --rosdistro humble -y
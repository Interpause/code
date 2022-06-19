. /opt/ros/$ROS_DISTRO/setup.sh
rosdep install -i --from-path /code/src --rosdistro humble -y
colcon build --symlink-install
echo "source /opt/ros/$ROS_DISTRO/setup.bash\nsource /code/install/local_setup.bash" | tee -a ~/.bashrc > /dev/null
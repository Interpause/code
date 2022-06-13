# pin to ROS2 humble & Ubuntu jammy for no reason
FROM ros:humble-ros-core-jammy
RUN apt update && apt install -y ros-humble-turtlesim
RUN apt install -y ros-humble-demo-nodes-cpp
RUN apt install -y ~nros-humble-rqt*
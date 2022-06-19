# see https://blog.mikesir87.io/2018/07/leveraging-multi-stage-builds-single-dockerfile-dev-prod/

# pin to ROS2 humble & Ubuntu jammy for no reason
FROM ros:humble-ros-base-jammy AS dev
WORKDIR /code
RUN apt update && apt install -y ~nros-humble-rqt*
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

FROM ros:humble-ros-core-jammy as build
WORKDIR /code
COPY . .
RUN rosdep install -i --from-path src --rosdistro humble -y && colcon build

# TODO: is there a way to start from a fresh image? dev stage has bloat in it
FROM build as prod
WORKDIR /code
RUN . install/setup.sh
CMD ["ros2","run","turtlesim","turtle_teleop"]
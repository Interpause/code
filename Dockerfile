# see https://blog.mikesir87.io/2018/07/leveraging-multi-stage-builds-single-dockerfile-dev-prod/

# pin to ROS2 humble & Ubuntu jammy for no reason
FROM ros:humble-ros-base-jammy AS dev
WORKDIR /code

# copy over package.xml per package as needed to prevent rebuilding container
RUN mkdir src
# COPY src/ros_tutorials/turtlesim/package.xml src/turtlesim

RUN apt update && \
  rosdep install -i --from-path src --rosdistro humble -y && \
  apt install -y ~nros-humble-rqt*

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

FROM dev as build
WORKDIR /app
COPY --from=dev /code .
RUN colcon build

# TODO: is there a way to start from a fresh image? dev stage has bloat in it
FROM dev as prod
WORKDIR /app
RUN rm -rf /code
COPY --from=build /app .
RUN . install/setup.sh
CMD ["ros2","run","turtlesim","turtle_teleop"]

# apparently a new terminal has to be used to prevent some issue?

# FROM ros:humble-ros-core-jammy AS base2
# WORKDIR /code
# COPY --from=base1 /code/ ./
# RUN source install/setup.bash


# RUN apt update && apt install -y ros-humble-turtlesim
# RUN apt install -y ros-humble-demo-nodes-cpp
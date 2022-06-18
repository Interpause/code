# see https://blog.mikesir87.io/2018/07/leveraging-multi-stage-builds-single-dockerfile-dev-prod/

# pin to ROS2 humble & Ubuntu jammy for no reason
FROM ros:humble-ros-base-jammy AS base1
# RUN apt update && apt install -y python3-rosdep && rosdep init && rosdep update
WORKDIR /code
# copy over package.xml per package as needed
COPY src/ros_tutorials/turtlesim/package.xml src/turtlesim
RUN apt update && \
  rosdep install -i --from-path src --rosdistro humble -y

# allows sourcing any number of scripts at runtime via the $BEFORE_SHELL env var
RUN echo 'a=(${BEFORE_SHELL//:/ }); for x in ${a[@]}; do source $x; done' >> ~/.bashrc
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# apparently a new terminal has to be used to prevent some issue?

# FROM ros:humble-ros-core-jammy AS base2
# WORKDIR /code
# COPY --from=base1 /code/ ./
# RUN source install/setup.bash


# RUN apt update && apt install -y ros-humble-turtlesim
# RUN apt install -y ros-humble-demo-nodes-cpp
# RUN apt install -y ~nros-humble-rqt*
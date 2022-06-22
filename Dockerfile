# see https://blog.mikesir87.io/2018/07/leveraging-multi-stage-builds-single-dockerfile-dev-prod/

# pin to ROS2 humble & Ubuntu jammy for no reason
FROM ros:humble-ros-base-jammy AS dev
WORKDIR /code
RUN apt update && apt install -y ~nros-humble-rqt*
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# TODO: research these images more https://hub.docker.com/r/osrf/ros/tags?page=1&name=humble
# does desktop-full contains simulation? what is simulation? desktop-full vs desktop?
# TODO: download these at home, not now...
# this stage should ideally need not ever change (its heavy)
# FROM osrf/ros:humble-desktop-full-jammy AS devbase
# WORKDIR /code
# # unnecessary?
# RUN apt update && apt install -y ~nros-humble-rqt*
# RUN apt install -y black
# # TODO: install CUDA

# add dependencies being tested & other experiments in this stage
# FROM devbase as dev
# Should we copy over workspace & preinstall dependencies?
# Can't prebuild (cause we mounting over the folder)
# Should we echo setup.sh into .bashrc, although local_setup.sh not built yet?

# TODO: split this into another Dockerfile. No reason to run dev stages.
# NOTE: Need to build Dockerfiles resilient to rebuilding offline or with private registry
FROM ros:humble-ros-core-jammy as build
WORKDIR /code
COPY . .
RUN . /opt/ros/humble/setup.sh
RUN rosdep install -i --from-path src --rosdistro humble -y && colcon build

# TODO: is there a way to start from a fresh image? dev stage has bloat in it
FROM build as prod
WORKDIR /code
RUN . install/setup.sh
CMD ["ros2","run","turtlesim","turtle_teleop"]
FROM --platform=linux/arm64/v8 ubuntu:focal


RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata wget curl vim dirmngr gnupg2 build-essential git ffmpeg libsm6 libxext6

#####
# ROS2 Foxy

RUN echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update -y && \
    apt-get install -y --no-install-recommends python3.8 python3-pip python3-colcon-common-extensions \
		     python3-colcon-mixin python3-rosdep python3-vcstool

RUN pip3 install opencv-python

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO foxy

RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-ros-base=0.9.2-1* libroslib-dev ffmpeg libsm6 libxext6

RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc

# CMD ["source", "/opt/ros/foxy/setup.bash"]

# END: ROS2 Foxy
#####

#####
# SSH

RUN apt-get update && apt-get install -y --no-install-recommends openssh-server

RUN mkdir /var/run/sshd

RUN sed -i '/^#/!s/PermitRootLogin .*/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config

RUN echo 'root:raspberry' | chpasswd

# RUN service ssh start

EXPOSE 22

# END: SSH
#####

#####
# Build Robot Core

ADD --chown=root:root core /home/core

WORKDIR /home/core

SHELL ["/bin/bash", "-c"] 

RUN rosdep install --from-paths src --ignore-src -y && \
    source /opt/ros/foxy/setup.bash && colcon build && bash install/setup.bash

# END: Build Robot Core
#####

CMD ["/usr/sbin/sshd", "-D"]

# ENTRYPOINT service ssh restart && bash


#####
# Tf2

RUN pip3 install tensorflow -f https://tf.kmtea.eu/whl/stable.html

# END: Tf2
#####



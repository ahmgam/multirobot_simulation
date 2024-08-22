FROM ubuntu:focal

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends \
    tzdata  \
    nodejs \
    git \
    libnode64 \
    && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

# setup sources.list
RUN . /etc/os-release \
    && echo "deb http://packages.osrfoundation.org/gazebo/$ID-stable `lsb_release -sc` main" > /etc/apt/sources.list.d/gazebo-latest.list


# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    gazebo11=11.12.0-1* \
    libgazebo11-dev=11.12.0-1* \
    build-essential \
    cmake \
    imagemagick \
    libboost-all-dev \
    libgts-dev \
    libjansson-dev \
    libtinyxml-dev \
    mercurial \
    npm \
    pkg-config \
    psmisc \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

# install gazebo packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgazebo11-dev=11.12.0-1* \
    && rm -rf /var/lib/apt/lists/*

# clone gzweb
ENV GZWEB_WS /root/gzweb
RUN git clone https://github.com/osrf/gzweb.git $GZWEB_WS
WORKDIR $GZWEB_WS

# build gzweb
RUN git checkout gzweb_1.4

RUN npm run deploy --- -m


#install ros


# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO noetic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-core=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO


# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install packages dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-dynamixel-sdk \
    libgflags-dev \
    libgoogle-glog-dev \
    protobuf-compiler libprotobuf-dev \
    ros-noetic-map-server \
    && rm -rf /var/lib/apt/lists/*


ENV TURTLEBOT3_MODEL=waffle               

RUN mkdir -p /root/robot_ws/src

RUN git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git /root/robot_ws/src/turtlebot3 

RUN git clone -b noetic-devel  https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git /root/robot_ws/src/turtlebot3_msgs

RUN git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git /root/robot_ws/src/turtlebot3_simulations

RUN git clone  https://github.com/ethz-asl/rotors_simulator.git /root/robot_ws/src/rotor_simulations

RUN git clone  https://github.com/OctoMap/octomap_msgs.git /root/robot_ws/src/octomap_msgs

RUN git clone  https://github.com/ethz-asl/mav_comm.git /root/robot_ws/src/mav_comm

RUN git clone  https://github.com/OctoMap/octomap_ros.git /root/robot_ws/src/octomap_ros

COPY ./multirobot_sim /root/robot_ws/src/multirobot_sim

RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /root/robot_ws; catkin_make'

RUN cp -R /usr/share/gazebo-11/media /root/gzweb/http/client/assets/

EXPOSE 8080
EXPOSE 7681
EXPOSE 11234
EXPOSE 11311
EXPOSE 11345

COPY ./entrypoint.sh /

RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

#CMD ["bash"]
FROM ros:noetic-ros-core-focal

RUN mkdir -p robot_ws/src

RUN mkdir -p robot_ws/devel

WORKDIR /robot_ws

COPY ./src/multirobot_sim/scripts/roschain/requirements.txt ./requirements.txt

RUN apt-get update && apt-get install -y python3-pip python-is-python3

RUN pip3 install -r requirements.txt

COPY ./src ./src

RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd robot_ws; catkin_make'
  
COPY ./entrypoint.sh ./entrypoint.sh

RUN chmod +x ./entrypoint.sh

ENTRYPOINT ["/robot_ws/entrypoint.sh"]

#CMD ["bash"]
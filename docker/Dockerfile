FROM ros:kinetic-robot

RUN apt update && apt install -y g++ git autogen autoconf build-essential cmake

RUN apt install -y ros-kinetic-dynamixel-motor

RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
RUN bin/bash -c "source /opt/ros/kinetic/setup.bash && \
                    mkdir -p /hans_ws/src && \
                    cd /hans_ws/src && \
                    catkin_init_workspace && \
                    cd /hans_ws/ && \
                    catkin_make && \
                    cd src/ && \
                    git clone -b kinetic-devel https://github.com/ros-controls/ros_control && \
                    cd /hans_ws/ && \
                    catkin_make"

RUN bin/bash -c "source /opt/ros/kinetic/setup.bash && \
                    cd /hans_ws/src && \
                    git clone https://timyanmuk@bitbucket.org/timym/cyton-robot-ros.git && \
                    cd /hans_ws/ && \
                    catkin_make"

CMD [ "bash", "-c", "tail -f /dev/null" ] 
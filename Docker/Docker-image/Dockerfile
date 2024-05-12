FROM ros:noetic

RUN apt update && apt install -y software-properties-common

# Add Qt PPA repository
RUN apt update && apt install -y dirmngr
RUN add-apt-repository ppa:beineri/opt-qt-5.15.2-focal

#### ROS pkg
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y \
	ros-${ROS_DISTRO}-visp-bridge  \
	ros-${ROS_DISTRO}-rqt-common-plugins  \
    ros-${ROS_DISTRO}-rqt-image-view  \
	v4l-utils  \
   	libev4 \
  	libev-dev \
    python3-pip \
	libpq-dev \
	python-dev \
 	ros-${ROS_DISTRO}-web-video-server \
 	liborocos-kdl-dev \
 	libpcl-dev \
	net-tools \
	netcat \
	wget \
    iputils-ping \ 
	wireless-tools  \
	libopencv-dev  \ 
    python3-opencv \
    ros-${ROS_DISTRO}-rosbridge-server \
	ros-${ROS_DISTRO}-teleop-twist-keyboard \
	ros-${ROS_DISTRO}-amcl \
	ros-${ROS_DISTRO}-map-server \
	ros-${ROS_DISTRO}-move-base \
	ros-${ROS_DISTRO}-urdf \
	ros-${ROS_DISTRO}-xacro \
	ros-${ROS_DISTRO}-rqt* \
	ros-${ROS_DISTRO}-rviz \
	ros-${ROS_DISTRO}-gmapping \
	ros-${ROS_DISTRO}-navigation \
	ros-${ROS_DISTRO}-turtlebot3-msgs \
	ros-${ROS_DISTRO}-dynamixel-sdk \
	ros-${ROS_DISTRO}-turtlebot3 \
	git 



#### QT library
RUN DEBIAN_FRONTEND=noninteractive apt install -y \
    qt515base \
    qt515script

RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y 


RUN apt install -y --no-install-recommends gdb 

## install python packages
RUN pip3 install psycopg2 
		


# Create catkin workspace
RUN mkdir -p /root/catkin_ws/src 
WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/RobotWebTools/tf2_web_republisher.git && \
    git clone -b noetic-devel https://github.com/ros/robot_state_publisher.git

WORKDIR /root/catkin_ws

# Build ROS packages
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    catkin_make"

# Add ROS setup to bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Start bash when container starts
CMD ["/bin/bash"]
RUN echo "All Done "


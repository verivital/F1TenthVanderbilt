#pull from the osrf full kinetic build
FROM nvidia/cudagl:9.0-base-ubuntu16.04
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES},display

#install ros
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y ros-kinetic-desktop-full && rosdep update && apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential


#install some ros packages we are gonna need
RUN apt-get update && apt-get install -y ros-kinetic-driver-base  ros-kinetic-navigation ros-kinetic-ros-control ros-kinetic-teb-local-planner ros-kinetic-move-base ros-kinetic-ros-controllers ros-kinetic-gazebo-ros-control ros-kinetic-ackermann-msgs ros-kinetic-joy
#navigate to home directory 
WORKDIR home

#install range_libc
COPY range_libc range_libc
WORKDIR range_libc/pywrapper
#compile it without cuda for now
RUN apt-get install -y python-pip && apt-get install -y python3-pip && pip install cython && python setup.py install
WORKDIR ..
WORKDIR ..



RUN mkdir -p catkin_ws/src 
WORKDIR catkin_ws
#intialize the workspace
RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash && catkin_make'
#navigate into the src directory
WORKDIR src
#COPY all of the relevant code files
COPY a_stars_pure_pursuit a_stars_pure_pursuit 
COPY f110-fall2018-skeletons f110-fall2018-skeletons 
COPY hector_slam hector_slam 
COPY particle_filter particle_filter
COPY range_libc range_libc
#build the workspace
WORKDIR ..
RUN /bin/bash -c 'apt-get install -y ros-kinetic-map-server && source /opt/ros/kinetic/setup.bash && catkin_make'

CMD /bin/bash -c 'source devel/setup.bash && roslaunch race f1_tenth.launch'

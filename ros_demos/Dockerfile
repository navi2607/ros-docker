FROM ubuntu:bionic

# Install python 3.8
RUN apt-get update && apt-get install -y software-properties-common
RUN add-apt-repository ppa:deadsnakes/ppa
RUN apt-get install -y \
    python3.8 \
    python3.8-dev \
    python3-distutils \
    curl \
    build-essential \
    cmake

# Install pip for Python 3.8
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
RUN python3.8 get-pip.py

# Disable interactive mod for apt
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Install all deps for ROS build
RUN python3.8 -m pip install \
    rosdep \
    rospkg \
    rosinstall_generator \
    rosinstall \
    wstool \
    vcstools \
    catkin_tools \
    catkin_pkg \
    empy

# Init ROS
RUN rosdep init
RUN rosdep update
RUN mkdir ~/ros_catkin_ws
RUN cd ~/ros_catkin_ws

# Prepare for ROS build
RUN mkdir ./src
RUN rosinstall_generator ros_comm --rosdistro melodic --deps --tar > melodic-ros_comm.rosinstall
RUN wstool init -j8 src melodic-ros_comm.rosinstall

# Install additional libs for ROS build
RUN apt-get install -y \
    libboost-all-dev \
    libconsole-bridge-dev \
    libpoco-dev \
    libgtest-dev \
    libtinyxml-dev \
    libtinyxml2-dev \
    liblz4-dev \
    liblz4-tool \
    liblz4-1 \
    liblog4cxx-dev \
    libbz2-dev \
    libgpgme-dev

# Configure CMake variables for ROS build
RUN catkin config \
    -DPYTHON_EXECUTABLE=/usr/bin/python3.8 \
    -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 \
    -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8.so
RUN catkin config --install
RUN catkin build

# PYTHON DEPS AFTER BUILD
RUN python3.8 -m pip install \
    defusedxml \
    netifaces

COPY ros_entrypoint.sh /usr/local/bin/
RUN  chmod 755 /usr/local/bin/ros_entrypoint.sh
ENTRYPOINT ["/usr/local/bin/ros_entrypoint.sh"]

RUN echo "source /usr/local/bin/ros_entrypoint.sh" >> ~/.bashrc

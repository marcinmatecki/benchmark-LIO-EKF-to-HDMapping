FROM ubuntu:20.04

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release software-properties-common \
    build-essential git cmake \
    python3-pip \
    libceres-dev libeigen3-dev \
    libpcl-dev \
    nlohmann-json3-dev \
    tmux \
    libflann-dev \
    libvtk7-dev \
    wget \
    libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros1.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt

RUN wget https://github.com/Kitware/CMake/releases/download/v3.24.0/cmake-3.24.0.tar.gz && \
    tar -xzf cmake-3.24.0.tar.gz && \
    cd cmake-3.24.0 && \
    ./bootstrap --prefix=/opt/cmake-3.24 && \
    make -j$(nproc) && \
    make install

RUN git clone https://gitlab.com/libeigen/eigen.git && \
    cd eigen && git checkout 3.4.0 && \
    mkdir build && cd build && \
    /opt/cmake-3.24/bin/cmake .. -DCMAKE_INSTALL_PREFIX=/opt/eigen-3.4 && \
    make -j$(nproc) && \
    make install

RUN git clone https://ceres-solver.googlesource.com/ceres-solver && \
    cd ceres-solver && git fetch --all --tags && git checkout tags/2.1.0 && \
    mkdir build && cd build && \
    /opt/cmake-3.24/bin/cmake .. -DEigen3_DIR=/opt/eigen-3.4/share/eigen3/cmake && \
    make -j$(nproc) && make install

RUN git clone https://github.com/strasdat/Sophus && \
    cd Sophus && mkdir build && cd build && \
    /opt/cmake-3.24/bin/cmake .. -DEigen3_DIR=/opt/eigen-3.4/share/eigen3/cmake && \
    make -j$(nproc) && make install


RUN git clone https://github.com/Livox-SDK/Livox-SDK.git && \
    cd Livox-SDK && \
    rm -rf build && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

WORKDIR /ros_ws

COPY ./src ./src

RUN sed -i \
  -e 's|lidar_topic:[[:space:]]*"velodyne_points"|lidar_topic:  "/livox/pointcloud"|' \
  -e 's|imu_topic:[[:space:]]*"imu/data"|imu_topic:  "/livox/imu"|' \
   src/LIO-EKF/config/ntu_viral.yaml

RUN sed -i '/include(kiss-icp.cmake)/d' src/LIO-EKF/CMakeLists.txt

RUN source /opt/ros/noetic/setup.bash && \
    catkin_make
    
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID ros && \
    useradd -m -u $UID -g $GID -s /bin/bash ros
WORKDIR /ros_ws

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /ros_ws/devel/setup.bash" >> ~/.bashrc

CMD ["bash"]

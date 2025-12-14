# LIO-EKF converter

## Example Dataset: 

Download the dataset from [Bunker DVI Dataset](https://charleshamesse.github.io/bunker-dvi-dataset/)  

## Intended use 

This small toolset allows to integrate SLAM solution provided by [lio-ekf](https://github.com/YibinWu/LIO-EKF) with [HDMapping](https://github.com/MapsHD/HDMapping).
This repository contains ROS 1 workspace that :
  - submodule to tested revision of LIO-EKF and KISS-ICP(for building LIO-EKF)
  - a converter that listens to topics advertised from odometry node and save data in format compatible with HDMapping.

## Dependencies

```shell
sudo apt install -y nlohmann-json3-dev
sudo apt install libgflags-dev
sudo apt install libgoogle-glog-dev
```

## Dependecies for Sophus and Ceres
```shell
cd ~
wget https://github.com/Kitware/CMake/releases/download/v3.24.0/cmake-3.24.0.tar.gz
tar -xzf cmake-3.24.0.tar.gz
sudo mv cmake-3.24.0 /opt
cd /opt/cmake-3.24.0
./bootstrap --prefix=/opt/cmake-3.24
make -j$(nproc)
sudo make install

cd ~
git clone https://gitlab.com/libeigen/eigen.git
sudo mv eigen /opt
cd /opt/eigen
git checkout 3.4.0
mkdir build && cd build
/opt/cmake-3.24/bin/cmake .. -DCMAKE_INSTALL_PREFIX=/opt/eigen-3.4
make -j$(nproc)
sudo make install
```

## Ceres and Sophus
```shell
cd ~
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver && git fetch --all --tags
git checkout tags/2.1.0
mkdir build && cd build
/opt/cmake-3.24/bin/cmake .. \
  -DEigen3_DIR=/opt/eigen-3.4/share/eigen3/cmake
make -j$(nproc)
sudo make install

cd ~
git clone https://github.com/strasdat/Sophus
cd Sophus
mkdir build && cd build
/opt/cmake-3.24/bin/cmake .. \
  -DEigen3_DIR=/opt/eigen-3.4/share/eigen3/cmake
make -j$(nproc)
sudo make install
```

## Building

Clone the repo
```shell
mkdir -p /test_ws/src
cd /test_ws/src
git clone https://github.com/marcinmatecki/LIO-EKF-to-HDMapping.git --recursive
cd ..
catkin_make --cmake-args -DEigen3_DIR=/opt/eigen-3.4/share/eigen3/cmake
```

## Usage - data SLAM:

Prepare recorded bag with estimated odometry:

In first terminal record bag:
```shell
rosbag record /local_map /odometry
```

 start odometry:
```shell 
cd /test_ws/
source ./devel/setup.sh # adjust to used shell
roslaunch lio_ekf eee_01.launch 
rosbag play {path_for_bag}
```

## Usage - conversion:

```shell
cd /test_ws/
source ./devel/setup.sh # adjust to used shell
rosrun lio-ekf-to-hdmapping listener <recorded_bag> <output_dir>
```

## Record the bag file:

```shell
rosbag record /local_map /odometry -O {your_directory_for_the_recorded_bag}
```

## LIO-EKF Launch:

```shell
cd /test_ws/
source ./devel/setup.sh # adjust to used shell
roslaunch lio_ekf eee_01.launch 
rosbag play {path_for_bag}
```

## During the record (if you want to stop recording earlier) / after finishing the bag:

```shell
In the terminal where the ros record is, interrupt the recording by CTRL+C
Do it also in ros launch terminal by CTRL+C.
```

## Usage - Conversion (ROS bag to HDMapping, after recording stops):

```shell
cd /test_ws/
source ./devel/setup.sh # adjust to used shell
rosrun lio-ekf-to-hdmapping listener <recorded_bag> <output_dir>
```

#!/bin/bash

# install everything with apt
sudo apt-get install $(grep -vE "^\s*#" ./package-list.txt | tr "\n" " ")

# clone repo
git clone --single-branch --depth 1 https://github.com/joeoakes/CMPSC488FA24Sec2Team1.git
cd ./CMPSC488FA24Sec2Team1
cmpscRepo=$PWD

# run rosdep
sudo rosdep init
rosdep init

# install freenect
git clone --single-branch --depth 1 https://github.com/OpenKinect/libfreenect.git
cd libfreenect
echo 'SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")' >> ./CMakeLists.txt
mkdir build && cd build
cmake .. -DBUILD_PYTHON3=ON
sudo make install

#make ros workspace and install dependencies
mkdir ~/ros2_ws
cd ~/ros2_ws
ln -sf $cmpscRepo/ros src
rosdep install --from-paths src --ignore-src -r -y
colcon build

# run checks

cd $cmpscRepo
./pi-check/pi-check.sh

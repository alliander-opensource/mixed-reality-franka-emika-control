#!/bin/bash

#echo $1

cd $1/src

## Download ROS-TCP-Endpoint package: https://github.com/Unity-Technologies/ROS-TCP-Endpoint 
git clone git@github.com:Unity-Technologies/ROS-TCP-Endpoint.git

## Download panda_moveit_config package: https://github.com/ros-planning/panda_moveit_config
git clone --branch noetic-devel git@github.com:ros-planning/panda_moveit_config.git

## Download franka_ros package: https://github.com/frankaemika/franka_ros
cd $1
git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros

## Install libfranka for the Franka Research 3: https://frankaemika.github.io/docs/installation_linux.html
cd
if [ -d "libfranka" ];
then
	echo "libfranka directory already exists"
else
	cd
	sudo apt remove "*libfranka*"
	sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
	git clone --recursive https://github.com/frankaemika/libfranka --branch 0.10.0 
	cd libfranka
	mkdir build
	cd build
	cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
	cmake --build .
fi
cd $1

## Ensure rosdep is installed and activated
sudo apt-get update
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update

## Install all required package dependencies
rosdep install --from-paths src --ignore-src -r -y

## Install catkin
sudo apt-get install python3-catkin-tools

## Configure the workspace
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build

## Start building the workspace
catkin build

## Source the package
source devel/setup.bash


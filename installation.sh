#!/bin/bash

# ------ install ROS (Kinetic)-------
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install -y ros-kinetic-desktop
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc


# ----------- make a new catkin workspace at the location of this file -----------
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $DIR
mkdir -p ./catkin_ws_clean/src
cd ./catkin_ws_clean/src
catkin_init_workspace
cd ..
catkin_make
source devel/setup.bash
cd src

# ----get dependencies-----
sudo apt-get update
sudo apt-get install -y ros-kinetic-ackermann-msgs
sudo apt-get install -y ros-kinetic-joy

#------clone needed repositories-----

git clone https://github.com/ArticulatedControl17/truck_master
git clone https://github.com/ArticulatedControl17/gulliview_server
git clone https://github.com/ArticulatedControl17/truck_automatic_ctrl
git clone https://github.com/ArticulatedControl17/truck_manual_ctrl
git clone https://github.com/ArticulatedControl17/path_planning
git clone https://github.com/ArticulatedControl17/custom_msgs
git clone https://github.com/ArticulatedControl17/truck_map
git clone https://github.com/ArticulatedControl17/truck_visualization

#--- build workspace---
cd ..
catkin_make -j4

# -- add setup.bash to bashrc --
echo "source $DIR/catkin_ws_clean/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

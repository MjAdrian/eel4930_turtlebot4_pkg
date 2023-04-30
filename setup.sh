#! /usr/bin/env bash

##################################3
# Install script for EEL4930 group
##################################3
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install -y \
    python3 \
    ros-galactic-desktop \
    ros-galactic-navigation2 \
    ros-galactic-nav2-simple-commander \
    ros-galactic-tf-transformations \
    "ros-galactic-turtlebot3*" \
    "ros-galactic-gazebo*"

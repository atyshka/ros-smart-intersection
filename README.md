ROS Smart Intersection
===
This repository contains files from the final project of ECE 5532 (Intro to Autonomous Vehicles). In this project, we designed and implemented a 
smart intersection that facilitates the passing of vehicle through safely without needing to stop. 

Prerequisites
---
* ROS Melodic and Gazebo
* [Audibot](https://github.com/robustify/audibot)
* OU UGV Libraries

Installation
---
To download the repository, go to your catkin workspace:

  `cd ~/catkin_ws/src`

Then clone the repository:

  `git clone https://github.com/rocketmax/ros-smart-intersection/new/main`
  
And build:

  `cd ..`

  `catkin_make`

Running
---
The main launch file that will run the complete project demonstration is:

  `roslaunch smart_intersection_launch smart_intersection.launch`
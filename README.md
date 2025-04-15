# ENPME661_Project03_Phase02

# Team Members

Chris Collins 
  - UID: 110697305
  - Directory ID: ccollin5@umd.edu

Kyle Demmerle 
  - UID: 121383341
  - Directory ID: kdemmerl@umd.edu

Dan Zinobile 
  - UID: 121354464
  -  Directory ID: zinobile@umd.edu

# Clone the repository in home folder

git clone [git@github.com:kademmerle/ENPME661_Project03_Phase02.git](https://github.com/kademmerle/ENPME661_Project03_Phase02.git)

# Dependencies
import pygame

import pygame.gfxdraw

import time

import math

import heapq

import numpy as np

import csv

# Part 1 -- Run Path Planner Independently
## Instructions for Running
1) Extract proj3p2_chris_daniel_kyle.zip to home directory
2) Navigate to Part01 folder:
   
      cd ~/proj3p2_chris_daniel_kyle/Part01
3) Run the path planning script with:
   
      python3 path_planning.py
4) Pygame will open a window and search for the solution path. Game will plot the path in the window, and then pause.
5) Close window to end script once game completes. Solution path is published to the file "part01_waypoints.csv" in the same directory the script resides.

## Example of Inputs (fucntional)
- start_node x: 0
- start_node y: 0
- start_node theta: 45
- goal_node x: 539
- goal_node y: 0
- RPM1: 50
- RPM2: 100
- clearance: 1

# Part 2 -- Launch the ROS Node
## Source ROS
source /opt/ros/humble/setup.bash

source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash


## Build the workspace
cd /Part02

colcon build

source install/setup.bash

export TURTLEBOT3_MODEL=waffle

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:turtlebot3_project3/share/turtlebot3_project3/models/

## Launch the path planner and wait for it to finish
ros2 launch turtlebot3_project3 competition_world.launch.py

## Close out of the display once path planning is finished

## Wait for gazebo to load

## Open a separate terminal and run controller script
cd /Project02

source install/setup.bash

ros2 run turtlebot3_project3 controller.py


# Video Link: Falcon_Sim
https://www.dropbox.com/scl/fi/fmmgjbitul9dvippasjqh/Falcon_Sim.mp4?rlkey=a4j3prztos9l6u7d74xrtd7xm&e=1&st=0y13g4je&dl=0

# Video Link: Gazebo
https://drive.google.com/file/d/1GRBmHJSjgqCbgyjLd2lvX1ygbl5Fs3w5/view?usp=drive_link


# ENPME661_Project03_Phase02

########################### Team Members ###########################

Chris Collins UID: 110697305

Kyle Demmerle UID: 121383341

Dan Zinobile UID: 121354464

#Clone the repository in home folder
cd
git clone git@github.com:kademmerle/ENPME661_Project03_Phase02.git

# Create a workspace in home folder
cd
mkdir -p project3_ws/src
cd

#Copy package to workspace src folder
cp -r ENPME661_Project03_Phase02/Part02/turtlebot3_project3 project3_ws/src

#Source ROS
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash


#Build the workspace
cd ~/project3_ws
colcon build
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:turtlebot3_project3/share/turtlebot3_project3/models/

#Launch the path planner and wait for it to finish
ros2 launch turtlebot3_project3 competition_world.launch.py

#Close out of the display once path planning is finished

#Wait for gazebo to load

#Open a separate terminal and run controller script
cd ~/project3_ws
source install/setup.bash
ros2 run turtlebot3_project3 controller.py

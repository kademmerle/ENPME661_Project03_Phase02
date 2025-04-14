import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/cgcollins91/Projects/ENPME661_Project03_Phase02/falcon_sim/ROS2/falcon_turtlebot3_project_ws/src/install/astar_falcon_planner'

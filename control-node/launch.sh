#!/bin/bash  
cd /home/cms-eam/Desktop/cms-inspection-robot-2024/control-node/
source ros2_dashing_py36/bin/activate
source ros2/dashing/setup.bash 
nohup python serial_control_node.py &
nohup python flask_control_node.py & 


#!/bin/bash
cd /home/cms-eam/Desktop/cms-inspection-robot-2024/camera-node/
source env/bin/activate
nohup python flask_camera_node.py &


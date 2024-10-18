#!/bin/bash
cd /home/cms-eam/Desktop/cms-inspection-robot-2024/sensors-node
source env/bin/activate
nohup python flask_sensors_node.py &
nohup python serial_sensors_node.py &
echo "cmseam" | sudo -S systemctl start grafana-server.service



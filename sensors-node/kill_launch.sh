#!/bin/bash
echo "cmseam" | sudo -S pkill -f serial_sensors_node.py
echo "cmseam" | sudo -S pkill -f flask_sensors_node.py
echo "cmseam" | sudo -S systemctl stop grafana-server.service

#!/bin/bash
echo "cmseam" | sudo -S pkill -f flask_control_node.py
echo "cmseam" | sudo -S pkill -f serial_control_node.py

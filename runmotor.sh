#!/bin/bash
sudo ip link set can0 up type can bitrate 500000
sudo ip link set up can0

cd /home/next/motortest_ws
sleep 0.1
source deval/setup.bash
sleep 0.1
roslaunch kinco_control drive_kinco.launch

echo "Run Success!!"

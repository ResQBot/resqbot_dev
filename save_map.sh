#!/bin/bash
source /opt/ros/humble/setup.bash &&
source /etc/ros/env.sh &&
ros2 service call /map_save std_srvs/srv/Trigger {} &&
scp resqbots@192.168.0.43:/home/resqbots/maps/last_map.pcd /home/paul/maps/last_raspi_map.pcd 

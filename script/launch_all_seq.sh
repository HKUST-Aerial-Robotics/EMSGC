#!/bin/bash
#cd /home/zhouyi/catkin_ws/src/EMSGC/launch/DistSurf
#roslaunch car.launch
#roslaunch hand.launch
#
#cd /home/zhouyi/catkin_ws/src/EMSGC/launch/EED
#roslaunch fast_drone.launch
#roslaunch light_variations.launch
#roslaunch occlusions.launch
#roslaunch what_is_background.launch

cd /home/zhouyi/catkin_ws/src/EMSGC/launch/EV-IMO
#roslaunch box_seq00.launch
roslaunch table_seq01.launch

cd /home/zhouyi/catkin_ws/src/EMSGC/launch/hkust_EMS
#roslaunch cast.launch
roslaunch corridor.launch
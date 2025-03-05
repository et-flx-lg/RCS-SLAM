#!/bin/sh
cd 

cd .ros/ 

mv odom.bag ~/catkin_ws/src/uri_soft_ah/ajh_swarm_slam/optimization/Data/Current/

mv commgraph.bag ~/catkin_ws/src/uri_soft_ah/ajh_swarm_slam/optimization/Data/Current/

mv noisy_odom.bag ~/catkin_ws/src/uri_soft_ah/ajh_swarm_slam/optimization/Data/Current/

mv centralgraph.bag ~/catkin_ws/src/uri_soft_ah/ajh_swarm_slam/optimization/Data/Current/

mv slam.bag ~/catkin_ws/src/uri_soft_ah/ajh_swarm_slam/optimization/Data/Current/

mv cmd_vel.bag ~/catkin_ws/src/uri_soft_ah/ajh_swarm_slam/optimization/Data/Current/

cd ../catkin_ws/src/uri_soft_ah/ajh_swarm_slam/optimization

python3 BagPostProc.py

python3 Stats_Eval.py



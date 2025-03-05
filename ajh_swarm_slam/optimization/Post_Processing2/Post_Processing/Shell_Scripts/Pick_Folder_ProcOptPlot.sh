#!/bin/sh
cd 

cd .ros/ 

mv odom.bag ~/catkin_ws/src/uri_soft_ah/ajh_swarm_slam/optimization/Data/$1/

mv commgraph.bag ~/catkin_ws/src/uri_soft_ah/ajh_swarm_slam/optimization/Data/$1/

mv noisy_odom.bag ~/catkin_ws/src/uri_soft_ah/ajh_swarm_slam/optimization/Data/$1/

mv centralgraph.bag ~/catkin_ws/src/uri_soft_ah/ajh_swarm_slam/optimization/Data/$1/

mv slam.bag ~/catkin_ws/src/uri_soft_ah/ajh_swarm_slam/optimization/Data/$1/

mv cmd_vel.bag ~/catkin_ws/src/uri_soft_ah/ajh_swarm_slam/optimization/Data/$1/

cd ../catkin_ws/src/uri_soft_ah/ajh_swarm_slam/optimization

python3 BagPostProc.py -f $1

python3 Stats_Eval.py -f $1

cd ~/catkin_ws/devel/lib/ajh_swarm_slam

./RCS_PGO20 $1

./DR_PGO20 $1

cd ~/catkin_ws/src/uri_soft_ah/ajh_swarm_slam/optimization

python3 RCS_Plotter.py -f $1

python3 DR_Plotter.py -f $1



#!/bin/sh

python3 BagPostProc.py

python3 Stats_Eval.py

cd ~/catkin_ws/devel/lib/ajh_swarm_slam

./RCS_PGO20 Current

./DR_PGO20 Current

cd ~/catkin_ws/src/uri_soft_ah/ajh_swarm_slam/optimization/Post_Processing


python3 RCS_Plotter.py

python3 DR_Plotter.py



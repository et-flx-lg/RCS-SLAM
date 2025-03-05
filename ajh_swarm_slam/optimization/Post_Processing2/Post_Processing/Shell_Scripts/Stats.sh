#!/bin/sh

#cd ~/catkin_ws/devel/lib/ajh_swarm_slam

#./RCS_PGO20 Current

#./DR_PGO20 Current

#cd ~/catkin_ws/src/uri_soft_ah/ajh_swarm_slam/optimization

############### RANDOM #######################
echo Random LN
python3 Stats_Eval.py -f 20min_House_Random_LN

echo 
echo Random MN
python3 Stats_Eval.py -f 20min_House_Random_MN

echo 
echo Random HN
python3 Stats_Eval.py -f 20min_House_Random_HN

############### Leader #######################
echo 
echo Leader LN
python3 Stats_Eval.py -f 20min_House_Leader_LN

echo 
echo Leader MN
python3 Stats_Eval.py -f 20min_House_Leader_MN

echo 
echo Leader HN
python3 Stats_Eval.py -f 20min_House_Leader_HN

############### Joy #######################
echo 
echo Joy LN
python3 Stats_Eval.py -f 20min_House_Joy_LN

echo 
echo Joy MN
python3 Stats_Eval.py -f 20min_House_Joy_MN

echo 
echo Joy HN
python3 Stats_Eval.py -f 20min_House_Joy_HN


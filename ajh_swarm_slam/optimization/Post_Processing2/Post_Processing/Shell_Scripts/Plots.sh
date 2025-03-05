#!/bin/sh

#cd ~/catkin_ws/devel/lib/ajh_swarm_slam

#./RCS_PGO20 Current

#./DR_PGO20 Current

#cd ~/catkin_ws/src/uri_soft_ah/ajh_swarm_slam/optimization

############### RANDOM #######################
python3 RCS_Plotter.py -f 20min_House_Random_LN

python3 DR_Plotter.py -f 20min_House_Random_LN


python3 RCS_Plotter.py -f 20min_House_Random_MN

python3 DR_Plotter.py -f 20min_House_Random_MN


python3 RCS_Plotter.py -f 20min_House_Random_HN

python3 DR_Plotter.py -f 20min_House_Random_HN

############### Leader #######################
python3 RCS_Plotter.py -f 20min_House_Leader_LN

python3 DR_Plotter.py -f 20min_House_Leader_LN


python3 RCS_Plotter.py -f 20min_House_Leader_MN

python3 DR_Plotter.py -f 20min_House_Leader_MN


python3 RCS_Plotter.py -f 20min_House_Leader_HN

python3 DR_Plotter.py -f 20min_House_Leader_HN

############### Joy #######################
python3 RCS_Plotter.py -f 20min_House_Joy_LN

python3 DR_Plotter.py -f 20min_House_Joy_LN


python3 RCS_Plotter.py -f 20min_House_Joy_MN

python3 DR_Plotter.py -f 20min_House_Joy_MN


python3 RCS_Plotter.py -f 20min_House_Joy_HN

python3 DR_Plotter.py -f 20min_House_Joy_HN


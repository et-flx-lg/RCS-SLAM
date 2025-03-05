#!/bin/sh



############### RANDOM #######################
python3 Plot_RCS_DR_Comp.py -c Random -n LN
python3 Plot_RCS_DR_Comp.py -c Random -n MN
python3 Plot_RCS_DR_Comp.py -c Random -n HN

############### Leader #######################
python3 Plot_RCS_DR_Comp.py -c Leader -n LN
python3 Plot_RCS_DR_Comp.py -c Leader -n MN
python3 Plot_RCS_DR_Comp.py -c Leader -n HN


############### Joy #######################
python3 Plot_RCS_DR_Comp.py -c Joy -n LN
python3 Plot_RCS_DR_Comp.py -c Joy -n MN
python3 Plot_RCS_DR_Comp.py -c Joy -n HN


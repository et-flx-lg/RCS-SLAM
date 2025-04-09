#!/bin/bash

# Wechsle in das Data-Verzeichnis und führe merge.py aus
cd ./Data
python3 merge.py

# Zurück zum vorherigen Verzeichnis und führe postLC.py aus
cd ..
python3 postLC.py

# Führe postGraph.py und postComm.py aus
python3 postGraph.py
python3 postComm.py

# Wechsle in das Verzeichnis catkin_ws/devel/lib/ajh_swarm_slam und führe DR_EXP und RCS_EXP aus
cd ~/catkin_ws/devel/lib/ajh_swarm_slam/
./DR_EXP
./RCS_EXP

# Wechsle zum Verzeichnis für die Optimierung und führe DR_PlotterEXP.py und RCS_PlotterEXP.py aus
cd ~/catkin_ws/src/ajh_swarm_slam/optimization
python3 DR_PlotterEXP.py
python3 RCS_PlotterEXP.py


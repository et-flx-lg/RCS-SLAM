import pandas as pd
import numpy as np
import os
import scipy.io
import csv
import sys

diameter = 0.34
internodes = 5


# Datei einlesen und Odometry-Daten extrahieren
def extract_odometry(input_file, bumps_input, output_mat_file, output_csv_file, robot_id_prefix="10"):
    if not os.path.exists(input_file):
        print(f"Fehler: Datei {input_file} nicht gefunden.")
        return
    
    if not os.path.exists(bumps_input):
        print(f"Fehler: Datei {bumps_input} nicht gefunden.")
        return
        
    odometry_data = []
    x_prev = 0.0
    y_prev = 0.0
    theta_prev = 0.0
    init = True
    zaehlung = 0
    
    with open(input_file, "r") as file:
        for line in file:
            parts = line.strip().split()
            if parts[0] == "VERTEX_SE2" and parts[1].startswith(robot_id_prefix) and ((len(parts[1]) == 4 and len(robot_id_prefix) == 1) or (len(parts[1]) == 5 and len(robot_id_prefix) == 2)):
                zaehlung = zaehlung + 1
                node_id = int(parts[1])
                x, y, theta = map(float, parts[2:])
                if init:
                	x_prev, y_prev, theta_prev = map(float, parts[2:])
                	init = False
                else:
                	for i in range(internodes):
                		i = i+1
                		factor = i / (internodes + 1)
                		x_int = x_prev + (x - x_prev) * factor
                		y_int = y_prev + (y - y_prev) * factor
                		theta_int = theta_prev
                		odometry_data.append([x_int, y_int, theta_int])
                	x_prev, y_prev, theta_prev = map(float, parts[2:])
                odometry_data.append([x, y, theta])
    
    #print(zaehlung)
    bump_data = []
    
    with open(bumps_input, "r") as file:
        reader = csv.DictReader(file)
        for line in reader:
            data = line["data"].split()
            data[0] = (int(data[0]) % 1000) * (internodes + 1)
            bump_data.append([data])            
                  
    # In NumPy-Array umwandeln
    state = np.array(odometry_data).T  # Transponieren für richtiges Format (3 x N)
    
    # Beispielhafte Sensordaten: 360 Laserstrahlen pro Zeitschritt
    N = state.shape[1]  # Anzahl der Zeitschritte aus den Odometry-Daten
    M1 = 360  # Anzahl der Laserstrahlen pro Zeitschritt
    meas = np.zeros((M1, 2, N))
    
    for i in range(N):
    	for h in range(M1):
        	meas[h, 0, i] = diameter
    	meas[:, 1, i] = np.linspace(-np.pi, np.pi, M1)
    
    M2 = 180 #roughly the range of the bumper
    object_det = np.zeros((M2, 2, N))
    
    for i in range(N):
    	object_det[:, 1, i] = np.linspace(-np.pi/2, np.pi/2, M2)
    	for b in range(len(bump_data)):
    		if i == bump_data[b][0][0]:
    			if bump_data[b][0][1]=='right':
    				object_det[:80, 0, i] = diameter
    				
    			elif bump_data[b][0][1]=='left':
    				object_det[100:, 0, i] = diameter
    				
    			elif bump_data[b][0][1]=='middle':
    				object_det[80:100, 0, i] = diameter
    				
    				
    
    # Speichern als MATLAB .mat-Datei
    scipy.io.savemat(output_mat_file, {"X": state, "z": meas, "b": object_det})
    
    # Sensordaten für einen Zeitschritt als CSV speichern
    df = pd.DataFrame(object_det[:, :, 7], columns=["Distance", "Angle"])
    df.to_csv(output_csv_file, index=False)
    print(f"Data has been saved!")

# Funktionsaufruf
if len(sys.argv) != 2:
        print("Usage: python3 OGMcrData.py <robot_id>")
        sys.exit(1)
        
robot_id = sys.argv[1]
print(f"Mapdata of Robot {robot_id} is getting created")

extract_odometry("./Data/Current/DR_LC_IneqOut.csv", f"./Data/Current/bumps/Robot{robot_id}.csv", "state_meas_data.mat", "sensor_data.csv", robot_id) #DR_LC_IneqOut, cppgraph


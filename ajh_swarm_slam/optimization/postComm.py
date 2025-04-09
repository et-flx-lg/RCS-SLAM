import pandas as pd
import numpy as np
import itertools
import csv
import os

directory = "./Data/Current/bumps"
teamsize = len([f for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f))])
print(f"Teamsize: {teamsize}")
robots = {}

for i in range(1, teamsize + 1):
    file_path = f"./Data/Current/groundtruth/Robot{i}_filtered.csv"
    df = pd.read_csv(file_path)[["pose.position.x", "pose.position.y"]]
    df.columns = [f"x{i}", f"y{i}"]
    robots[i] = df

min_rows = min(len(df) for df in robots.values())
robots = {i: df.iloc[:min_rows] for i, df in robots.items()}

data = pd.concat(robots.values(), axis=1)

# Liste für gefilterte Paare
filtered_pairs = []

# Alle möglichen Roboterpaare durchgehen
for r1, r2 in itertools.combinations(range(1, teamsize + 1), 2):
    # Abstand berechnen
    distance = np.sqrt((data[f"x{r1}"] - data[f"x{r2}"])**2 + (data[f"y{r1}"] - data[f"y{r2}"])**2)

    
    # Indizes generieren
    robot1_index = (r1 * 1000) + data.index
    robot2_index = (r2 * 1000) + data.index

    # Nur Paare mit Abstand < 1 speichern
    mask = distance < 1
    filtered_pairs.append(pd.DataFrame({
        "robot1_index": robot1_index[mask],
        "robot2_index": robot2_index[mask],
        "distance": distance[mask]
    }))

# Gesamtdaten in eine Datei speichern
if filtered_pairs:
    result = pd.concat(filtered_pairs)
    result.to_csv("./Data/Current/cppcomm.csv", index=False, header=False, sep=' ')
else:
    print("Keine Paare mit Abstand < 1 gefunden.")
    

output_hop = "./Data/Current/hopgraph.csv"

with open(output_hop, "w", newline="") as f:
	

    writer = csv.writer(f, delimiter=" ")
    
    for r1, r2, r3 in itertools.combinations(range(1, teamsize + 1), 3):
    	i = 0
    	for row in data.itertuples(index=False):
    		x1 = getattr(row, f"x{r1}")
    		y1 = getattr(row, f"y{r1}")
    		x2 = getattr(row, f"x{r2}")
    		y2 = getattr(row, f"y{r2}")
    		x3 = getattr(row, f"x{r3}")
    		y3 = getattr(row, f"y{r3}")
    	
    		distance12 = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    		distance13 = np.sqrt((x1 - x3)**2 + (y1 - y3)**2)  
    		distance23 = np.sqrt((x3 - x2)**2 + (y3 - y2)**2)
    		
    		r1_idx = (r1 * 1000) + i
    		r2_idx = (r2 * 1000) + i
    		r3_idx = (r3 * 1000) + i
    		
    	
    		if distance12 < 1 and distance13 < 1 and distance23 > 1:
    			writer.writerow((r2_idx, r3_idx))
    	
    		elif distance13 < 1 and distance23 < 1 and distance12 > 1: 
    			writer.writerow((r1_idx, r2_idx))
    		
    		elif distance12 < 1 and distance23 < 1 and distance13 > 1:
    			writer.writerow((r1_idx, r3_idx))
    		
    		i = i + 1
    		
    	 

"""# Dateien einlesen
robot1 = pd.read_csv("./Data/Current/groundtruth/Robot1_filtered.csv")
robot2 = pd.read_csv("./Data/Current/groundtruth/Robot2_filtered.csv")

# Relevante Spalten extrahieren
robot1 = robot1[["pose.position.x", "pose.position.y"]]
robot2 = robot2[["pose.position.x", "pose.position.y"]]

# Mergen der Daten basierend auf Index (falls gleiche Anzahl Zeilen angenommen wird)
data = pd.concat([robot1, robot2], axis=1)
data.columns = ["x1", "y1", "x2", "y2"]

# Abstand berechnen
data["distance"] = np.sqrt((data["x1"] - data["x2"])**2 + (data["y1"] - data["y2"])**2)

# Indizes für die Roboter erstellen, bevor gefiltert wird
data.insert(0, "robot1_index", 1000 + data.index)
data.insert(1, "robot2_index", 2000 + data.index)

# Nur Werte mit Abstand < 1 filtern
filtered_data = data[data["distance"] < 1]

# Speichern in neuer CSV ohne Komma
filtered_data[["robot1_index", "robot2_index", "distance"]].to_csv("./Data/Current/cppcomm.csv", index=False, header=False, sep=' ')
"""

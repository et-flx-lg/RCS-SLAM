import pandas as pd
import numpy as np

# Dateien einlesen
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

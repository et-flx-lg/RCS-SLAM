import pandas as pd
import os

# Definiere die Teamgröße
directory = "./Data/Current/bumps"
teamsize = len([f for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f))])
timeinterval = 3

# Definiere den Pfad zu den Daten
base_path = "./Data/Current/groundtruth/"

# Schleife über alle Roboter
for i in range(1, teamsize + 1):
    # Erstelle den Dateipfad für den aktuellen Roboter
    file_path = os.path.join(base_path, f"Robot{i}.csv")
    
    # Lese die Datei ein
    df = pd.read_csv(file_path)
    
    # Konvertiere die Zeitspalte in Sekunden (Ganzzahl)
    df['Time_Seconds'] = df['Time'].astype(float).astype(int)
    
    # Filtere Zeilen, sodass nur ein Eintrag pro 3-Sekunden-Intervall bleibt
    filtered_df = df.groupby(df['Time_Seconds'] // timeinterval).first()
    
    # Entferne die Hilfsspalte
    filtered_df = filtered_df.drop(columns=['Time_Seconds'])
    
    # Entferne den allerersten Eintrag
    filtered_df = filtered_df.iloc[1:].reset_index(drop=True)
    
    # Speichere die gefilterten Daten in eine neue Datei
    filtered_file_path = os.path.join(base_path, f"Robot{i}_filtered.csv")
    filtered_df.to_csv(filtered_file_path, index=False)
    
    print(f"Gefilterte Datei für Robot{i} wurde gespeichert als {filtered_file_path}")

print("Alle gefilterten Groundtruth Dateien wurden gespeichert.")


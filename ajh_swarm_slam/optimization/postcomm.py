import pandas as pd

# Datei einlesen
df = pd.read_csv("./Data/Current/groundtruth/Robot1.csv")
df2 = pd.read_csv("./Data/Current/groundtruth/Robot2.csv")

# Konvertiere die Zeitspalte in Sekunden (Ganzzahl)
df['Time_Seconds'] = df['Time'].astype(float).astype(int)
df2['Time_Seconds'] = df2['Time'].astype(float).astype(int)

# Filtere Zeilen, sodass nur ein Eintrag pro 3-Sekunden-Intervall bleibt
filtered_df = df.groupby(df['Time_Seconds'] // 3).first()
filtered_df2 = df2.groupby(df2['Time_Seconds'] // 3).first()

# Entferne die Hilfsspalte
filtered_df = filtered_df.drop(columns=['Time_Seconds'])
filtered_df2 = filtered_df2.drop(columns=['Time_Seconds'])

# Speichere die gefilterten Daten in eine neue Datei
filtered_df.to_csv("./Data/Current/groundtruth/Robot1_filtered.csv", index=False)
filtered_df2.to_csv("./Data/Current/groundtruth/Robot2_filtered.csv", index=False)

print("Gefilterte Datei wurde gespeichert als groundtruth/Robot1_filtered.csv")

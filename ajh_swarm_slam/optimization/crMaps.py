import os
import sys
import subprocess

def main():
    if len(sys.argv) < 2:
        print("❗ Bitte gib die Versuchsnummer an, z. B.: python3 run_all.py 2")
        sys.exit(1)

    versuch = sys.argv[1]

    bumps_dir = './Data/Current/bumps/'

    if not os.path.exists(bumps_dir):
        print(f"❗ Verzeichnis {bumps_dir} existiert nicht.")
        sys.exit(1)

    # Alle Dateien im Ordner zählen (z. B. robot1.csv, robot2.csv, ...)
    files = [f for f in os.listdir(bumps_dir) if os.path.isfile(os.path.join(bumps_dir, f))]
    anzahl_roboter = len(files)

    print(f"🔍 {anzahl_roboter} Roboter erkannt.")

    for roboter_id in range(1, anzahl_roboter + 1):
        print(f"▶️ Roboter {roboter_id}: führe aus...")

        # 1. Befehl
        subprocess.run(['python3', 'OGMcrData.py', str(roboter_id)])

        # 2. Befehl mit Versuch und Roboter-ID
        subprocess.run(['python3', 'ogm.py', str(versuch), str(roboter_id)])

    print("✅ Alle Roboter wurden verarbeitet.")

if __name__ == '__main__':
    main()


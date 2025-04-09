import os
import shutil

def copy_and_rename_files(versuch, menge):
    target_dir = "./Current/odom"
    target_gt = "./Current/groundtruth"
    target_b = "./Current/bumps"
    os.makedirs(target_dir, exist_ok=True)
    
    for i in range(runs, 0, -1):  # Startet mit dem höchsten Ordner
        source_dir = f"./V{versuch}.{i}/odom"
        source_gt = f"./V{versuch}.{i}/groundtruth"
        source_b = f"./V{versuch}.{i}/bumps"
        
        if not os.path.exists(source_dir):
            print(f"Warnung: {source_dir} existiert nicht und wird übersprungen.")
            continue
            
        if not os.path.exists(source_gt):
            print(f"Warnung: {source_gt} existiert nicht und wird übersprungen.")
            continue
            
        if not os.path.exists(source_gt):
            print(f"Warnung: {source_b} existiert nicht und wird übersprungen.")
            continue
            
        for j in range(1, menge + 1):  # Kopiert RobotX_filtered Dateien
            source_file = os.path.join(source_dir, f"Robot{j}_filtered.csv")
            target_file = os.path.join(target_dir, f"Robot{j + 3 * (i-1)}_filtered.csv")
            
            if os.path.exists(source_file):
                shutil.copy2(source_file, target_file)
                print(f"Kopiert: {source_file} -> {target_file}")
            else:
                print(f"Fehlend: {source_file} wird übersprungen.")

        for j in range(1, menge + 1):  # Kopiert RobotX_filtered Dateien
            source_file = os.path.join(source_dir, f"Robot{j}-odom.csv")
            target_file = os.path.join(target_dir, f"Robot{j + 3 * (i-1)}-odom.csv")
            
            if os.path.exists(source_file):
                shutil.copy2(source_file, target_file)
                print(f"Kopiert: {source_file} -> {target_file}")
            else:
                print(f"Fehlend: {source_file} wird übersprungen.")
                
        for j in range(1, menge + 1):  # Kopiert RobotX_filtered Dateien
            source_file = os.path.join(source_gt, f"Robot{j}.csv")
            target_file = os.path.join(target_gt, f"Robot{j + 3 * (i-1)}.csv")
            
            if os.path.exists(source_file):
                shutil.copy2(source_file, target_file)
                print(f"Kopiert: {source_file} -> {target_file}")
            else:
                print(f"Fehlend: {source_file} wird übersprungen.")
                
        for j in range(1, menge + 1):  # Kopiert RobotX_filtered Dateien
            source_file = os.path.join(source_gt, f"Robot{j}_filtered.csv")
            target_file = os.path.join(target_gt, f"Robot{j + 3 * (i-1)}_filtered.csv")
            
            if os.path.exists(source_file):
                shutil.copy2(source_file, target_file)
                print(f"Kopiert: {source_file} -> {target_file}")
            else:
                print(f"Fehlend: {source_file} wird übersprungen.")
                
        for j in range(1, menge + 1):  # Kopiert RobotX_filtered Dateien
            source_file = os.path.join(source_b, f"Robot{j}.csv")
            target_file = os.path.join(target_b, f"Robot{j + 3 * (i-1)}.csv")
            
            if os.path.exists(source_file):
                shutil.copy2(source_file, target_file)
                print(f"Kopiert: {source_file} -> {target_file}")
            else:
                print(f"Fehlend: {source_file} wird übersprungen.")

if __name__ == "__main__":
    versuch = 2
    menge = 3
    runs = 5
    copy_and_rename_files(versuch, menge)



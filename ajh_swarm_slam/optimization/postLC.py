import pandas as pd
import os
import math

directory = "./Data/Current/bumps"

# Define the team size
teamsize = len([f for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f))])

print(f"Teamsize: {teamsize}")

# Define the path to the data
base_path = "./Data/Current/groundtruth/"

# Output file path
output_file = "./Data/Current/loopclosure.csv"

# Create a list to store the results
results = []

# Loop over all robots
for i in range(1, teamsize + 1):
    # Create the file path for the current robot
    file_path = os.path.join(base_path, f"Robot{i}_filtered.csv")
    
    # Read the filtered file
    df = pd.read_csv(file_path)
    
    # Loop through all rows of the DataFrame and calculate the distance to the origin
    for index, row in df.iterrows():
        # Extract the x and y positions of the robot
        x = row['pose.position.x']  # Use the name of the column for x (if different)
        y = row['pose.position.y']  # Use the name of the column for y (if different)
        
        # Calculate the Euclidean distance to the origin
        distance = math.sqrt(x**2 + y**2)
        
        # Check if the distance is less than or equal to 1 meter
        if distance <= 1.0:  # Direct comparison with <=
            # Format the index as a 3-digit number
            formatted_index = f"{index:03d}"
            
            # Create the first entry (robot number + timestamp)
            first_entry = f"{i}{formatted_index}"
            
            # Create the second entry (21 + timestamp)
            second_entry = f"21{formatted_index}"
            
            # Add the result to the list
            results.append(f"{first_entry} {second_entry} {distance}")

# Save results to a file without commas and headers
with open(output_file, "w") as f:
    f.write("\n".join(results))

print("The data for LoopClosures has been saved.")


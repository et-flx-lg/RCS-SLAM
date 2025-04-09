import pandas as pd
import os
import math
import numpy as np

# Function to convert quaternion to yaw (theta)
def quaternion_to_yaw(x, y, z, w):
    """Convert a quaternion to a yaw angle in radians (0 to 2π)."""
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    theta = math.atan2(siny_cosp, cosy_cosp)
    return theta if theta >= 0 else theta + 2 * math.pi  # Ensure theta is always positive

# Function to normalize angle between -pi and pi
def normalize_angle(angle):
    """Ensure the angle stays within the range [-π, π]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

# Define the team size
directory = "./Data/Current/bumps"
teamsize = len([f for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f))])

# Define the path to the odometry data
base_path = "./Data/Current/odom/"
final_path = "./Data/Current/"

# Output file path
output_file = os.path.join(final_path, "cppgraph.csv")

# Open file for writing
with open(output_file, "w") as f:
    max_entries = 999999999999  # Track max number of entries
    
    for i in range(1, teamsize + 1):
        file_path = os.path.join(base_path, f"Robot{i}_filtered.csv")

        # Read the odometry CSV file
        df = pd.read_csv(file_path)
        if len(df) < max_entries:
        	max_entries = len(df)
        	
    # Process each robot
    for i in range(1, teamsize + 1):
        file_path = os.path.join(base_path, f"Robot{i}_filtered.csv")

        # Read the odometry CSV file
        df = pd.read_csv(file_path)

        # Ensure the required columns exist
        required_columns = {'pose.x', 'pose.y', 'orientation.x', 'orientation.y', 'orientation.z', 'orientation.w'}
        if not required_columns.issubset(df.columns):
            print(f"Error: Missing columns in {file_path}")
            continue

        # Iterate over the rows
        prev_id, prev_x, prev_y, prev_theta = None, None, None, None
        g = 0
        for index, row in df.iterrows():
        
            if g < max_entries:
            	g = g + 1
            	# Extract position and orientation
            	x = row['pose.x']
            	y = row['pose.y']
            	theta = quaternion_to_yaw(row['orientation.x'], row['orientation.y'], row['orientation.z'], row['orientation.w'])

            	# Create the unique vertex ID
            	vertex_id = f"{i}{index:03d}"
            	f.write(f"VERTEX_SE2 {vertex_id} {x} {y} {theta}\n")

            	# If there was a previous point, create an edge
            	if prev_id is not None:
            
                	R1 = np.array([[math.cos(prev_theta), -math.sin(prev_theta)],[math.sin(prev_theta),math.cos(prev_theta)]])
                	R2 = np.array([[math.cos(theta), -math.sin(theta)],[math.sin(theta),math.cos(theta)]])
                	P1 = np.array([[prev_x],[prev_y]])
                	P2 = np.array([[x],[y]])
                
                	edgeR = np.matmul(np.transpose(R1),R2)
                	edgeTh = math.atan2(edgeR[1,0],edgeR[0,0])
                	edgeP = np.matmul(np.transpose(R1), (P2-P1))

                	# Write EDGE_SE2 line
                	f.write(f"EDGE_SE2 {prev_id} {vertex_id} {edgeP[0,0]} {edgeP[1,0]} {edgeTh} 3185.4347253710557 0 0 772532.0257356245 0 4423.392741146374\n")

            	# Update previous values
            	prev_id, prev_x, prev_y, prev_theta = vertex_id, x, y, theta

    # Generate data for static robot 21
    prev_id = None
    for index in range(max_entries):
        vertex_id = f"21{index:03d}"
        f.write(f"VERTEX_SE2 {vertex_id} 0.0 0.0 0.0\n")

        # Create edges between consecutive entries
        if prev_id is not None:
            f.write(f"EDGE_SE2 {prev_id} {vertex_id} 0.0 0.0 0.0 3185.4347253710557 0 0 772532.0257356245 0 4423.392741146374\n")

        prev_id = vertex_id  # Update previous ID for the next loop

print(f"Graph CSV file saved as {output_file}")


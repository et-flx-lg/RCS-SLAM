import scipy.io
import scipy.stats
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import sys

class Map():
    def __init__(self, xsize, ysize, grid_size):
        self.xsize = xsize # keep this as is
        self.ysize = ysize # keep this as is
        self.grid_size = grid_size # save this off for future use

        self.origin_x = xsize // 2  # Set the origin in the center for negative coordinates
        self.origin_y = ysize // 2
        self.log_prob_map = np.zeros((self.xsize, self.ysize)) # set all to zero

        self.alpha = 0.1 # The assumed thickness of obstacles
        self.beta = 2 * np.pi/180.0 # The assumed width of the laser beam
        self.z_max = 200 # The max reading from the laser

        # Pre-allocate the x and y positions of all grid positions into a 3D tensor
        # (pre-allocation = faster)
        self.grid_position_m = np.array([np.tile(np.arange(0, self.xsize*self.grid_size, self.grid_size)[:,None], (1, self.ysize)),
                                         np.tile(np.arange(0, self.ysize*self.grid_size, self.grid_size)[:,None].T, (self.xsize, 1))])

        # Log-Probabilities to add or remove from the map 
        self.l_occ = np.log(0.65/0.35)
        self.l_free = np.log(0.35/0.65)
        
        self.theta = 0.0
        self.thetam1 = 0.0
        self.pose_2m1 = []

    def set_map_boundary_occupied(self):
    	# Randzellen auf besetzt setzen
    	self.log_prob_map[0, :] = self.l_occ  # obere Zeile
    	self.log_prob_map[-1, :] = self.l_occ  # untere Zeile
    	self.log_prob_map[:, 0] = self.l_occ  # linke Spalte
    	self.log_prob_map[:, -1] = self.l_occ  # rechte Spalte
    
    def update_map(self, pose, z, bumps, posem1):

        dx = self.grid_position_m.copy() # A tensor of coordinates of all cells
        dx[1, :, :] -= (pose[0]/self.grid_size + self.origin_x) * self.grid_size # A matrix of all the x coordinates of the cell
        dx[0, :, :] -= (pose[1]/self.grid_size + self.origin_y) * self.grid_size # A matrix of all the y coordinates of the cell
        #theta_to_grid = np.arctan2(dx[1, :, :], dx[0, :, :]) - pose[2] # matrix of all bearings from robot to cell
        	

        theta_to_grid = np.arctan2(dx[1, :, :], dx[0, :, :]) - pose[2]
        
        # Wrap to +pi / - pi
        theta_to_grid[theta_to_grid > np.pi] -= 2. * np.pi
        theta_to_grid[theta_to_grid < -np.pi] += 2. * np.pi

        dist_to_grid = scipy.linalg.norm(dx, axis=0) # matrix of L2 distance to all cells from robot

        # For each laser beam
        for z_i in z:
            r = z_i[0] # range measured
            b = z_i[1] # bearing measured
            
            # Calculate which cells are measured free or occupied, so we know which cells to update
            # Doing it this way is like a billion times faster than looping through each cell (because vectorized numpy is the only way to numpy)
            free_mask = (np.abs(theta_to_grid - b) <= self.beta/2.0) & (dist_to_grid < (r - self.alpha/2.0))
                   
            # Adjust the cells appropriately
            self.log_prob_map[free_mask] += self.l_free
            
        # For each bump
        pos_x = int((pose[0]/map.grid_size)+map.origin_x)
        pos_y = int((pose[1]/map.grid_size)+map.origin_y)
    
        for b_i in bumps:
            r = b_i[0] # range measured
            b = b_i[1] # bearing measured
            
            
            if r > 0.0:
            	# Calculate which cells are measured free or occupied, so we know which cells to update
            	# Doing it this way is like a billion times faster than looping through each cell (because vectorized numpy is the only way to numpy)
            	#pos_xray = int(pos_x - r * np.sin(orient + b - np.pi/2)/map.grid_size)
            	#pos_yray = int(pos_y - r * np.cos(orient + b - np.pi/2)/map.grid_size)
            	theta = posem1[2] + b
            	arrow = (pose[0:2] / map.grid_size + np.array([0.3/map.grid_size, 0]).dot(np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])))
            	#pos_xray = int(pos_x - r*np.sin(orient)/map.grid_size)
            	#pos_yray = int(pos_y - r*np.cos(orient)/map.grid_size)
            	pos_xray = int(arrow[0] + self.origin_x)
            	pos_yray = int(arrow[1] + self.origin_y)
            	
            	occ_mask = np.zeros_like(theta_to_grid, dtype=bool)
            	occ_mask[(pos_yray-2): (pos_yray+2),(pos_xray-2):(pos_xray+2)] = True
            	#occ_mask = (np.abs(theta_to_grid - b) <= self.beta/2.0) & (np.abs(dist_to_grid - r) <= self.alpha/2.0)
                   
            	# Adjust the cells appropriately
            	self.log_prob_map[occ_mask] += self.l_occ
            	
            

if __name__ == '__main__':

    if len(sys.argv) != 3:
        print("Usage: python3 ogm.py <trial> <robot_id>")
        sys.exit(1)
    
    trial = sys.argv[1]
    robotid = sys.argv[2]
    
    data = scipy.io.loadmat('state_meas_data.mat')
    state = data['X']
    meas = data['z']
    bumps = data['b']

    # Define the parameters for the map.  (This is a 15x15m map with grid size 0.05x0.05m)
    grid_size = 0.03
    map = Map(int(15/grid_size), int(15/grid_size), grid_size)  

    plt.ion() # enable real-time plotting
    plt.figure(1) # create a plot
    z_test = np.array([[2.0, 0.0]])  # 2m nach vorne, 0° Winkel
    
    map.set_map_boundary_occupied()
    
    for i in tqdm(range(len(state.T))):
    	if i > 0:
        	map.thetam1 = map.theta
        	map.theta = state[2, i]
    
        	map.update_map(state[:,i], meas[:,:,i], bumps[:, :, i], state[:,i - 1]) # update the map
        
    	
        	# Real-Time Plotting 
        	# (comment out these next lines to make it run super fast, matplotlib is painfully slow)
        	plt.clf()
        	pose = state[:,i]
        	circle = plt.Circle(((pose[0]/map.grid_size)+map.origin_x, (pose[1]/map.grid_size)+map.origin_y), radius=0.34/map.grid_size, fc='y')
        	#plt.gca().add_patch(circle)
        	arrow = (pose[0:2] / map.grid_size + np.array([0.3/map.grid_size, 0]).dot(np.array([[np.cos(pose[2]), np.sin(pose[2])], [-np.sin(pose[2]), np.cos(pose[2])]]))) 
        	#plt.plot([pose[0]/map.grid_size + map.origin_x, arrow[0] + map.origin_x], [pose[1]/map.grid_size + map.origin_y , arrow[1] + map.origin_y])
        	plt.imshow(1.0 - 1./(1.+np.exp(map.log_prob_map)), 'Greys')
        	plt.gca().invert_yaxis()
        	plt.xticks(ticks=np.linspace(0, map.log_prob_map.shape[1], num=5), labels=[str(-map.origin_x * map.grid_size), str(-map.origin_x * map.grid_size/2), "0.0", str(map.origin_x * map.grid_size/2), str(map.origin_x * map.grid_size)])
        	plt.yticks(ticks=np.linspace(0, map.log_prob_map.shape[0], num=5), labels=[str(-map.origin_y * map.grid_size), str(-map.origin_y * map.grid_size/2), "0.0", str(map.origin_y * map.grid_size/2), str(map.origin_y * map.grid_size)])
        	plt.xlabel("x [m]")
        	plt.ylabel("y [m]")
        	
        	plt.pause(0.001)
        
        


    filepath = f"/home/robolab/figures/V{trial}/R{robotid}_DR_V{trial}.png"
    plt.savefig(filepath, format="png")
    
    print("Image saved")
    
    #plt.pause(999999)
    # Final Plotting
    #plt.ioff()
    #plt.clf()
    #plt.imshow(1.0 - 1./(1.+np.exp(map.log_prob_map)), 'Greys') # This is probability
    #plt.imshow(map.log_prob_map, 'Greys') # log probabilities (looks really cool)
    #plt.show()


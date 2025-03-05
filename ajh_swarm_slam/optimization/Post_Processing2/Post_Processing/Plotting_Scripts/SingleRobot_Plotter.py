
# pylint: disable=invalid-name, E1101

from __future__ import print_function

import argparse

import gtsam
import matplotlib.pyplot as plt
from gtsam.utils import plot
from bagconverter import bc
from bagpy import bagreader
import time
import numpy as np
import math
from metrics import ATE

def SlamProc (slamcsv, optcsv):

	opt = np.loadtxt(optcsv,delimiter=' ', dtype=str, usecols=(0,1,2,3,4))
	slam = np.loadtxt(slamcsv, delimiter=' ', dtype=str)
	
	slamnodes = np.zeros([len(slam),2])
	det_range = 1.0
	for i in range(len(slam)):
		n = int(slam[i,0])
		for k in range(len(opt)):
			o = int(opt[k,1])
			if (opt[k,0]=='VERTEX_SE2') and (o==n):
				
				
				ray = -.698 + int(slam[i, 1])*.199
				heading = float(opt[k,4])
				if heading < 0.0:
					heading = heading + 2*math.pi
				bearing = heading + ray
								
				slamnodes[i,0] = float(opt[k,2]) + det_range*math.cos(bearing)
				slamnodes[i,1] = float(opt[k,3]) + det_range*math.sin(bearing)
				
				break
				
	return slamnodes


def main():
    """Main runner."""

    parser = argparse.ArgumentParser(
        description="Plot initial and optimizied graphs.")
    parser.add_argument('-t', '--nodetime', type=float, help='Node Interval')
    parser.add_argument('-r', '--robnum', type=int, help="Rob to Plot.")
    parser.add_argument('-f', '--folder', type=str,help='Current Data File.')


    args = parser.parse_args()
    
    folder = 'Current' if args.folder is None\
    	else args.folder

	
    nodetime = 3 if args.nodetime is None\
       else args.nodetime
       
    robnum = 1 if args.robnum is None\
       else args.robnum
       
       
    """
    Convert from rosbag files to csv then use comm graph to create edges during communications
    """ 
        
    
    optimized = 'Data/' + folder + '/RCS_LC_IneqOut.csv'
    slamcsv = 'Data/' + folder + '/slam.csv'
    groundtruth = 'Data/' + folder + '/odom.bag'
    initial = 'Data/' + folder + '/cppgraph.csv'  

    init = open(initial, 'r+')
    gt = open(groundtruth, 'r+')               
    opt_graph, initial = gtsam.readG2o(optimized, False)
      
    resultPoses = gtsam.utilities.extractPose2(initial)
    
    #print(len(resultPoses))
        
    bag = bagreader('Data/' + folder + '/odom.bag')
    odom = bag.odometry_data()
    print(len(odom))
    
    #nodes = int(len(resultPoses)/20)
    nodes = 400
    #print(nodes)
    
    rob = np.zeros((nodes,2));
    slamnodes = SlamProc(slamcsv, optimized)

    j = 0;
    #x1, y1 = [-2.5,-2.5], [-1.5, 9.5]
    #x2, y2 = [-2.5, 8.5], [9.5, 9.5]
    #x3, y3 = [8.5, 8.5], [9.5, -1.5]
    #x4, y4 = [8.5, -2.5], [-1.5, -1.5]

    x1, y1 = [-10,-10], [-10, 10]
    x2, y2 = [-10, 10], [10, 10]
    x3, y3 = [10, 10], [10, -10]
    x4, y4 = [10, -10], [-10, -10]
             
    for i in range(((robnum-1)*nodes),(robnum*nodes)):
      rob[j,0] = resultPoses[i,0];
      rob[j,1] = resultPoses[i,1];
      j = j+1;

        
    odomfile = 'Data/' + folder + '/odom/Robot' + str((robnum)) + '-odom.csv'
          
    odomdata = np.loadtxt(odomfile,delimiter=',', skiprows=1, dtype=str)
    
    x = odomdata[:,4]
    y = odomdata[:,5]
      
    gt_x = x.astype(float)
    gt_y = y.astype(float)
      
    label = "Ground Truth " 
            
    bag = bagreader('Data/' + folder + '/noisy_odom.bag')
    odom = bag.odometry_data()
      
    noisyfile = 'Data/' + folder + '/noisy_odom/Robot' + str((robnum)) + '-noisy_odom.csv'
    noisydata = np.loadtxt(noisyfile,delimiter=',', skiprows=1, dtype=str)
    
       
    x = noisydata[:,4]
    y = noisydata[:,5]
      
    noisy_x = x.astype(float)
    noisy_y = y.astype(float)
    
    ATE1,ATE2 = ATE(odomdata, noisydata, rob, nodes, nodetime)  
    
    Odom_Output = 'Average Odometry ATE: ' + str(np.mean(ATE1))
    Opt_Output = 'Average Optimized ATE: ' + str(np.mean(ATE2))
    
    print(Odom_Output)
    print(Opt_Output)
      
    label = "Odometry "
    plt.plot(rob[:,0],rob[:,1], 'g', label="Optimized")
    plt.plot(gt_x,gt_y, 'b', label=label)  
    plt.plot(noisy_x,noisy_y, 'r', label=label)
    plt.plot(x1, y1, x2, y2, x3, y3, x4, y4, marker='o')
        
        
    plt.plot(rob[:,0],rob[:,1], 'g', label="Optimized")
    plt.scatter(slamnodes[:,0], slamnodes[:,1])

    plt.legend()
    plt.title("Robot 8 with Loop Closure")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.show()


         
    
if __name__ == "__main__":
    main()

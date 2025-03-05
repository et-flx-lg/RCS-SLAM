"""
 ----------------------------------------------------------------------------

 * File: Stats_Eval.py
 * Author: Adam Hoburg
 * Date: 25 March 2024
 
 * Purpose: Performs statistical evaluation of noisy odometry to determine actual
 noise characteristics of the uncertain estimate. Compares ground truth trajectory
 with estimated trajectory and averages errors across all trajectories to get 
 observed noise characteristics. These values are then used to update the input pose
 graphs prior to optimization.

 * -------------------------------------------------------------------------- */
"""

from __future__ import print_function

import argparse

"""import gtsam"""
import matplotlib.pyplot as plt
"""from gtsam.utils import plot"""
from bagconverter import bc
from bagpy import bagreader
import time
import numpy as np
import math
from metrics import ATE
import csv
from getTheta import getTheta


		
	

def main():

    parser = argparse.ArgumentParser(
        description="Plot initial and optimizied graphs.")
    parser.add_argument('-f', '--folder', type=str,help='Desired data folder.')
    parser.add_argument('-t', '--nodetime', type=float, help='Node Interval')


    args = parser.parse_args()
    
    folder = 'Current' if args.folder is None\
    	else args.folder  
        
    	
    nodetime = 3 if args.nodetime is None\
       else args.nodetime
       
    teamsize = 20
       
    """
    Convert from rosbag files to csv then use comm graph to create edges during communications
    """ 
    pre_stats_centgraph = 'Data/' + folder + '/pre_stats_cppgraph.csv'
    centgraph = 'Data/' + folder + '/cppgraph.csv'
    groundtruth = 'Data/' + folder + '/odom.bag'        
    bag = bagreader('Data/' + folder + '/odom.bag')
    #### Commenting this out because it takes place in BagPostProc.py###
    #optimized = 'Data/' + folder + '/IneqOut.csv'
    #odom = bag.odometry_data()
    
    noisy_bag = bagreader('Data/' + folder + '/noisy_odom.bag')
    noisy_odom = noisy_bag.odometry_data()
    
    nodes = 400
    
    print("Nodes Per Robot: "+ str(nodes))
    x_error = np.zeros((1,teamsize))
    y_error = np.zeros((1,teamsize))
    th_error = np.zeros((1,teamsize))
    
    e_x = np.zeros((1,nodes-1))
    e_y = np.zeros((1,nodes-1))
    e_th = np.zeros((1,nodes-1))
    
    gt_n = np.zeros((nodes,3))
    n_odom = np.zeros((nodes,3))
    dist_total = np.zeros((1,teamsize))
    rot_total = np.zeros((1,teamsize))
    noisy_dist_total = np.zeros((1,teamsize))

    k = 0         
    for i in range(0,teamsize):
              
      odomfile = 'Data/' + folder + '/odom/Robot' + str((i+1)) + '-odom.csv'
      noisyfile = 'Data/' + folder + '/noisy_odom/Robot' + str((i+1)) + '-noisy_odom.csv'
      
      gt = np.loadtxt(odomfile,delimiter=',', skiprows=1, dtype=str)      
      noisy = np.loadtxt(noisyfile,delimiter=',', skiprows=1, dtype=str)
      
      step1 = int(len(gt)/nodes)
      step2 = int(len(noisy)/nodes)
      
      gt_dist = 0
      n_dist = 0
      r_sum = 0


      for k in range(0,nodes):
      	gt_n[k,0] = gt[(k*step1),4]
      	gt_n[k,1] = gt[(k*step1),5]
      	gt_n[k,2] = getTheta(gt[(k*step1),7], gt[(k*step1),8], gt[(k*step1),9], gt[(k*step1),10])
      	
      	n_odom[k,0] = noisy[(k*step2),4] 
      	n_odom[k,1] = noisy[(k*step2),5]
      	n_odom[k,2] = getTheta(noisy[(k*step2),7], noisy[(k*step2),8], noisy[(k*step2),9], noisy[(k*step2),10])
      	#print(n_odom[k,2])
      
      for j in range(0,(nodes-1)):
      ######## Ground Truth ###########################################
      	gt_R1 = np.array([[math.cos(gt_n[j,2]), -math.sin(gt_n[j,2])],[math.sin(gt_n[j,2]),math.cos(gt_n[j,2])]])
      	gt_R2 = np.array([[math.cos(gt_n[j+1,2]), -math.sin(gt_n[j+1,2])],[math.sin(gt_n[j+1,2]),math.cos(gt_n[j+1,2])]])
      	gt_P1 = np.array([[gt_n[j,0]],[gt_n[j,1]]])
      	gt_P2 = np.array([[gt_n[j+1,0]],[gt_n[j+1,1]]])

      	gt_edgeR = np.matmul(np.transpose(gt_R1),gt_R2)
      	gt_edgeTh = math.atan2(gt_edgeR[1,0],gt_edgeR[0,0])
      	gt_edgeP = np.matmul(np.transpose(gt_R1), (gt_P2-gt_P1))
      	
      	######## Noisy Odom ###########################################
      	n_R1 = np.array([[math.cos(n_odom[j,2]), -math.sin(n_odom[j,2])],[math.sin(n_odom[j,2]),math.cos(n_odom[j,2])]])
      	n_R2 = np.array([[math.cos(n_odom[j+1,2]), -math.sin(n_odom[j+1,2])],[math.sin(n_odom[j+1,2]),math.cos(n_odom[j+1,2])]])
      	n_P1 = np.array([[n_odom[j,0]],[n_odom[j,1]]])
      	n_P2 = np.array([[n_odom[j+1,0]],[n_odom[j+1,1]]])
      	
      	n_edgeR = np.matmul(np.transpose(n_R1),n_R2)
      	n_edgeTh = math.atan2(n_edgeR[1,0],n_edgeR[0,0])
      	n_edgeP = np.matmul(np.transpose(n_R1), (n_P2-n_P1))
      	   
      	error_x = gt_edgeP[0,0] - n_edgeP[0,0]
      	error_y = gt_edgeP[1,0] - n_edgeP[1,0]
      	error_th = gt_edgeTh - n_edgeTh
      	      	
      	e_x[0,j] = abs(error_x)
      	e_y[0,j] = abs(error_y)
      	e_th[0,j] = abs(error_th)
      	
      	d_gt = math.sqrt(math.pow(gt_edgeP[0,0],2) + math.pow(gt_edgeP[1,0],2))
      	d_n = math.sqrt(math.pow(n_edgeP[0,0],2) + math.pow(n_edgeP[1,0],2))      	
      	gt_dist += d_gt
      	n_dist += d_n
      	r_sum += abs(gt_edgeTh)
      
      dist_total[0,i] = gt_dist
      rot_total[0,i] = r_sum
      noisy_dist_total[0,i] = n_dist
      x_error[0,i] = np.mean(e_x)
      y_error[0,i] = np.mean(e_y)
      th_error[0,i] = np.mean(e_th)
    
        
    X_Output = 'Average X Error: ' + str(np.mean(x_error))
    Y_Output = 'Average Y Error: ' + str(np.mean(y_error))
    Th_Output = 'Average Th Error: ' + str(np.mean(th_error))
    gt_Dist_Traveled = 'GT Average Total Distance Traveled: ' + str(np.mean(dist_total))
    n_Dist_Traveled = 'Noisy Average Total Distance Traveled: ' + str(np.mean(noisy_dist_total))
    Dist_Per_Node = 'Average Distance Per Node: ' + str(np.mean(dist_total)/nodes)
    Rot_Per_Node = 'Average Rotation Per Node: ' + str(np.mean(rot_total)/nodes)
    X_NoisePer = 'X Noise Percentage: ' + str(np.mean(x_error)/(np.mean(dist_total)/nodes))
    Y_NoisePer = 'Y Noise Percentage: ' + str(np.mean(y_error)/(np.mean(dist_total)/nodes))
    TH_NoisePer = 'Th Noise Percentage: ' + str(np.mean(th_error)/(np.mean(rot_total)/nodes))
    
    Stats_File = 'Data/' + folder + '/STATS.csv'
    stats = open(Stats_File, "w+")
    stats.write(X_Output + '\n')
    stats.write(Y_Output + '\n')
    stats.write(Th_Output + '\n')
    stats.write(gt_Dist_Traveled + '\n')
    stats.write(n_Dist_Traveled + '\n')
    stats.write(Dist_Per_Node + '\n')
    stats.write(Rot_Per_Node + '\n')
    stats.write(X_NoisePer + '\n')
    stats.write(Y_NoisePer + '\n')
    stats.write(TH_NoisePer + '\n')
    
    stats.close()

    
    print(X_Output)
    print(Y_Output)
    print(Th_Output)
    print(' ')
    print(gt_Dist_Traveled)
    print(n_Dist_Traveled)
    print(Dist_Per_Node)
    print(Rot_Per_Node)
    print(X_NoisePer)
    print(Y_NoisePer)
    print(TH_NoisePer)
    
    
    # This updates the cppgraph.csv file with the calculated noise characteristics.
    with open(pre_stats_centgraph, 'r') as f:
    	with open(centgraph, 'w') as c:
    		g = csv.reader(f, delimiter=' ')
    		w = csv.writer(c, delimiter=' ')
    		for graph in g:
    		
    			if (graph[0] == 'EDGE_SE2'):
    				graph[6] = str(1/math.pow(np.mean(x_error),2))
    				graph[9] = str(1/math.pow(np.mean(y_error),2))
    				graph[11] = str(1/math.pow(np.mean(th_error),2))
    				    				
    			w.writerow(graph)

         
    
if __name__ == "__main__":
    main()

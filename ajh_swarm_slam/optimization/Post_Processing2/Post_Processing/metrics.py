"""
 ----------------------------------------------------------------------------

 * File: metrics.py
 * Author: Adam Hoburg
 * Date: 25 March 2024
 
 * Purpose: Error evaluations for noisy odometry and optimized estimates. Can 
be used to provide either Relative Position Error (RPE) or Absolute Trajectory Error(ATE). 

 * -------------------------------------------------------------------------- */
"""
import csv
import rosbag
import rospy
import numpy as np
from bagpy import bagreader
import pandas as pd
import math
from getTheta import getTheta


def transform(x1, x2):

	R1 = np.array([[math.cos(x1[2]), -math.sin(x1[2])],[math.sin(x1[2]),math.cos(x1[2])]])
	R2 = np.array([[math.cos(x2[2]), -math.sin(x2[2])],[math.sin(x2[2]),math.cos(x2[2])]])
	P1 = np.array([[x1[0]],[x1[1]]])
	P2 = np.array([[x2[0]],[x2[1]]])
	
	edgeR = np.matmul(np.transpose(R1),R2)
	edgeTh = math.atan2(edgeR[1,0],edgeR[0,0])
	edgeP = np.matmul(np.transpose(R1), (P2-P1))
	
	return edgeP

 
#############################################
# Relative Pose Error
#############################################  


def RPE(gt1_nodes, gt2_nodes, noisy1_nodes, noisy2_nodes, rob1, rob2, nodes):
	noisy_RPE = np.zeros((nodes))
	opt_RPE = np.zeros((nodes))
	
	for j in range(0,nodes):
				
		gt_P = transform(gt1_nodes[j,:], gt2_nodes[j,:])
		n_P = transform(noisy1_nodes[j,:], noisy2_nodes[j,:])
		o_P = transform(rob1[j,:], rob2[j,:])
						
		noisy_error = np.sqrt(np.matmul(np.transpose(np.subtract(gt_P,n_P)),np.subtract(gt_P,n_P)))
		
		opt_error = np.sqrt(np.matmul(np.transpose(np.subtract(gt_P,o_P)),np.subtract(gt_P,o_P)))
		
		noisy_RPE[j] = noisy_RPE[j] + noisy_error
		opt_RPE[j] = opt_RPE[j] + opt_error
	
	return noisy_RPE, opt_RPE

#############################################
# Absolute Trajectory Error
#############################################
 	

def ATE(gt_nodes, noisy_odom_nodes, opt, nodes,nodetime):

 

  #----- Noisy Odometry ATE ----
  
  MSE_x = np.square(np.subtract(gt_nodes[:,0],noisy_odom_nodes[:,0]))
  MSE_y = np.square(np.subtract(gt_nodes[:,1],noisy_odom_nodes[:,1]))
  
  
  
  ATE1 = np.sqrt(np.add(MSE_x,MSE_y))
    
    #----- Optimization ATE ----
  
  MSE_x = np.square(np.subtract(gt_nodes[:,0],opt[:,0]))
  MSE_y = np.square(np.subtract(gt_nodes[:,1],opt[:,1]))

  
  ATE2 = np.sqrt(np.add(MSE_x,MSE_y))
 
  

	
  
  return ATE1, ATE2

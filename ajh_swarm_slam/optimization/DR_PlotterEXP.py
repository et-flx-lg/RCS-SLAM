"""
 ----------------------------------------------------------------------------

 * File: DR_Plotter.py
 * Author: Adam Hoburg
 * Date: 25 March 2024
 
 * Purpose: This code is intended to perform the error evaluation and plotting 
 for the direct-ranging pose graph approach. In the context of this project, this is
 used to compare performance with RCS-SLAM. This scipt reads in data from ground truth, 
 noisy odometry and optimized estimates performs error comparison for both relative
 position error and absolute trajectory error then plots the actual/esimated trajectories
 and the error comparisons for direct ranging with and without loop closure. 
 This code also saves all of the error data into .csv files to make subsequent
 plotting much more efficient. This code also reads in the nodes that are labled
 with object detection and plots the sparse map estimate of the environment.


 * -------------------------------------------------------------------------- */
"""

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
from metrics import ATE, RPE
from getTheta import getTheta


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
	parser.add_argument('-f', '--folder', type=str,help='Current Data File.')
	parser.add_argument('-t', '--nodetime', type=float, help='Node Interval')

	parser.add_argument(
		'-g',
		'--groundtruth', type=str,
		help="ground truth bag file.")

	args = parser.parse_args()
	
	folder = 'Current' if args.folder is None\
		else args.folder

	nodetime = 3 if args.nodetime is None\
		else args.nodetime
				
	optimized_DR = 'Data/' + folder + '/DR_IneqOut.csv'
	optimized_DR_LC = 'Data/' + folder + '/DR_LC_IneqOut.csv'
	
	#groundtruth = 'Data/' + folder + '/odom.bag'
	initial = 'Data/' + folder + '/cppgraph.csv'
	noisy_bag_file = 'Data/' + folder + '/noisy_odom.bag' 

	teamsize = 2

	init = open(initial, 'r+')
	#gt = open(groundtruth, 'r+')
	
	opt_graph, initial_DR = gtsam.readG2o(optimized_DR, False)
	opt_graph, initial_DR_LC = gtsam.readG2o(optimized_DR_LC, False)

	
	resultPoses_DR = gtsam.utilities.extractPose2(initial_DR)
	resultPoses_DR_LC = gtsam.utilities.extractPose2(initial_DR_LC)

	nodes = int(len(resultPoses_DR)/(teamsize+1))
	#nodes = 400 - valid for 3 second node interval of 20min simulation.
	print("Nodes Per Robot: "+ str(nodes))
	
	rob1_DR = np.zeros((nodes,3));
	rob1_DR_LC = np.zeros((nodes,3));

	rob2_DR = np.zeros((nodes,3)); 
	rob2_DR_LC = np.zeros((nodes,3));

	
	#slamnodes_DR = SlamProc(slamcsv, optimized_DR)
	#slamnodes_DR_LC = SlamProc(slamcsv, optimized_DR_LC)

	
	##### This is done in Stats_Eval.py so commenting out because its extremely slow####
	#bag = bagreader(groundtruth)
	#odom = bag.odometry_data()
	
	#noisy_bag = bagreader(noisy_bag_file)
	#noisy_odom = noisy_bag.odometry_data()
	
	interactions = (teamsize/2)*(teamsize-1) # This interactions per node. Equation is sum of decreasing series cause r1 has 19 interactions, r2 has 18, etc.
	
	RPE_odom = np.zeros((int(interactions), nodes))
	RPE_opt_DR = np.zeros((int(interactions), nodes))
	RPE_opt_DR_LC = np.zeros((int(interactions), nodes))
	
	nodes_gt = np.zeros((nodes, teamsize,3))
	nodes_n = np.zeros((nodes, teamsize,3))


	ATE_odom = np.zeros((teamsize, nodes))
	ATE_opt_DR = np.zeros((teamsize, nodes))	
	ATE_opt_DR_LC = np.zeros((teamsize, nodes))	

	x1, y1 = [-5,5], [5, 5]
	x2, y2 = [5, 5], [-5, 5]
	x3, y3 = [-5, -5], [-5, 5]
	x4, y4 = [-5, 5], [-5, -5]


	k = 0
	p = 0

	#fig, ax = plt.subplots(nrows=1, ncols=2, layout='constrained')
	fig, ax = plt.subplots(nrows=1, ncols=2)
	
	for i in range(0,teamsize):
		odomfile = 'Data/' + folder + '/groundtruth/Robot' + str((i+1)) + '.csv'
		gt = np.loadtxt(odomfile,delimiter=',', skiprows=1, dtype=str)
		x = gt[:,5]
		y = gt[:,6]
		
		x = x.astype(float)
		y = y.astype(float) 
	

		label = "Ground Truth " + str(i+1)
		
		ax[0].plot(x,y, 'b', label="Ground Truth")
		
		step = int(len(gt)/nodes)
		#print(gt[0,:])
		for t in range(0,nodes):
			nodes_gt[t,i,0] = gt[(t*step),5] 
			nodes_gt[t,i,1] = gt[(t*step),6] 
			nodes_gt[t,i,2] = getTheta(gt[(t*step),8], gt[(t*step),9], gt[(t*step),10], gt[(t*step),11])
			
		
		
	for i in range(0,teamsize):
		noisyfile = 'Data/' + folder + '/odom/Robot' + str((i+1)) + '-odom.csv'
		#print(noisyfile)
		n = np.loadtxt(noisyfile,delimiter=',', skiprows=1, dtype=str)
		x = n[:,4]
		y = n[:,5]
		
		x = x.astype(float)
		y = y.astype(float)
				
		label = "Odometry " + str(i+1)
		
		ax[1].plot(x,y, 'r', label="Odometry")

		step = int(len(n)/nodes)
		
		for t in range(0,nodes):
			nodes_n[t,i,0] = n[(t*step),4] 
			nodes_n[t,i,1] = n[(t*step),5] 
			nodes_n[t,i,2] = getTheta(n[(t*step),7], n[(t*step),8], n[(t*step),9], n[(t*step),10])	

	print("Node Matricies Created")	
	ax[0].plot(x1, y1, x2, y2, x3, y3, x4, y4, marker='o', color='tab:gray', linewidth=4)
	ax[1].plot(x1, y1, x2, y2, x3, y3, x4, y4, marker='o', color='tab:gray', linewidth=4)


	ax[0].set_title('Ground Truth')
	ax[1].set_title('Odometry')
	
	ax[0].set_xlabel('X-Position (m)')
	ax[0].set_ylabel('Y-Position (m)')

	ax[1].set_xlabel('X-Position (m)')
	ax[1].set_ylabel('Y-Position (m)')
	
	ax[0].set_ylim([-6,6])
	ax[0].set_xlim([-6,6])
	ax[1].set_ylim([-6,6])
	ax[1].set_xlim([-6,6])
	
	plt.savefig('Data/' + folder + '/figures/DR_GT_Noisy.pdf', format="pdf")
	
	print("Calculating Errors")

	#fig, ax = plt.subplots(nrows=1, ncols=2, layout='constrained')
	fig, ax = plt.subplots(nrows=1, ncols=2)
	
	for i in range(0,teamsize):
		for j in range((i*nodes),((i+1)*nodes)):
			rob1_DR[k,0] = resultPoses_DR[j,0];
			rob1_DR[k,1] = resultPoses_DR[j,1];
			rob1_DR[k,2] = resultPoses_DR[j,2];

			rob1_DR_LC[k,0] = resultPoses_DR_LC[j,0];
			rob1_DR_LC[k,1] = resultPoses_DR_LC[j,1];
			rob1_DR_LC[k,2] = resultPoses_DR_LC[j,2];

			k = k+1
			
		ax[0].plot(rob1_DR[:,0],rob1_DR[:,1], 'g', label='DR')
		ax[1].plot(rob1_DR_LC[:,0],rob1_DR_LC[:,1], 'tab:cyan', label='DR-LC')

		k=0
		
		
		gt1 = np.reshape(nodes_gt[:,i,:],(nodes,3))
		noisy1 = np.reshape(nodes_n[:,i,:],(nodes,3))
		
		ATE1, ATE2_DR = ATE(gt1, noisy1, rob1_DR, nodes, nodetime)
		ATE1, ATE2_DR_LC = ATE(gt1, noisy1, rob1_DR_LC, nodes, nodetime)
		
		ATE_odom[i,:] = ATE1
		ATE_opt_DR[i,:] = ATE2_DR
		ATE_opt_DR_LC[i,:] = ATE2_DR_LC
		
		
		for r in range((i+1),teamsize):
		
			m = 0
			for j in range((r*nodes),((r+1)*nodes)):
				rob2_DR[m,0] = resultPoses_DR[j,0];
				rob2_DR[m,1] = resultPoses_DR[j,1];
				rob2_DR[m,2] = resultPoses_DR[j,2];

				rob2_DR_LC[m,0] = resultPoses_DR_LC[j,0];
				rob2_DR_LC[m,1] = resultPoses_DR_LC[j,1];
				rob2_DR_LC[m,2] = resultPoses_DR_LC[j,2];


				m = m+1
				
			gt1 = np.reshape(nodes_gt[:,i,:],(nodes,3))
			noisy1 = np.reshape(nodes_n[:,i,:],(nodes,3))

			gt2 = np.reshape(nodes_gt[:,r,:],(nodes,3))
			noisy2 = np.reshape(nodes_n[:,r,:],(nodes,3))
			
			RPE1, RPE2_DR = RPE(gt1, gt2, noisy1, noisy2, rob1_DR, rob2_DR, nodes)

			RPE1, RPE2_DR_LC = RPE(gt1, gt2, noisy1, noisy2, rob1_DR_LC, rob2_DR_LC, nodes)

			
			RPE_odom[p,:] = RPE1
			RPE_opt_DR[p,:] = RPE2_DR
			RPE_opt_DR_LC[p,:] = RPE2_DR_LC
			p+=1


	ax[0].plot(x1, y1, x2, y2, x3, y3, x4, y4, marker='o', color='tab:gray', linewidth=4)
	ax[1].plot(x1, y1, x2, y2, x3, y3, x4, y4, marker='o', color='tab:gray', linewidth=4)

	ax[0].set_title('DR')
	ax[1].set_title('DR with LC')
	
	ax[0].set_xlabel('X-Position (m)')
	ax[0].set_ylabel('Y-Position (m)')

	ax[1].set_xlabel('X-Position (m)')
	ax[1].set_ylabel('Y-Position (m)')
	
	ax[0].set_ylim([-6,6])
	ax[0].set_xlim([-6,6])
	ax[1].set_ylim([-6,6])
	ax[1].set_xlim([-6,6])
	plt.savefig('Data/' + folder + '/figures/DR_Opt_Nodes.pdf', format="pdf")
			

	
	mean_odom_ATE = np.mean(ATE_odom)
	mean_opt_ATE_DR = np.mean(ATE_opt_DR)
	mean_opt_ATE_DR_LC = np.mean(ATE_opt_DR_LC)


	node_avg_odom_ATE = np.mean(ATE_odom, axis=0)
	node_avg_opt_ATE_DR = np.mean(ATE_opt_DR, axis=0)
	node_avg_opt_ATE_DR_LC = np.mean(ATE_opt_DR_LC, axis=0)


	mean_odom_RPE = np.mean(RPE_odom) 
	mean_opt_RPE_DR = np.mean(RPE_opt_DR)
	mean_opt_RPE_DR_LC = np.mean(RPE_opt_DR_LC)

	
	node_avg_odom_RPE = np.mean(RPE_odom, axis=0)
	node_avg_opt_RPE_DR = np.mean(RPE_opt_DR, axis=0)	
	node_avg_opt_RPE_DR_LC = np.mean(RPE_opt_DR_LC, axis=0)	

	
	ATE_Odom_Output = 'Odometry ATE: ' + str(mean_odom_ATE)
	ATE_Opt_Output_DR = 'DR ATE: ' + str(mean_opt_ATE_DR)
	ATE_Opt_Output_DR_LC = 'DR-LC ATE: ' + str(mean_opt_ATE_DR_LC)

	
	RPE_Odom_Output = 'Odometry RPE: ' + str(mean_odom_RPE)
	RPE_Opt_Output_DR = 'DR RPE: ' + str(mean_opt_RPE_DR)
	RPE_Opt_Output_DR_LC = 'DR-LC RPE: ' + str(mean_opt_RPE_DR_LC)
	
	
	print(ATE_Odom_Output)
	print(ATE_Opt_Output_DR)
	print(ATE_Opt_Output_DR_LC)
	print(RPE_Odom_Output)
	print(RPE_Opt_Output_DR)
	print(RPE_Opt_Output_DR_LC)

	Avg_File = 'Data/' + folder + '/DR_AVERAGES.csv'
	avgs = open(Avg_File, "w+")
	avgs.write(ATE_Odom_Output + '\n')
	avgs.write(ATE_Opt_Output_DR + '\n')
	avgs.write(ATE_Opt_Output_DR_LC + '\n')
	avgs.write(RPE_Odom_Output + '\n')
	avgs.write(RPE_Opt_Output_DR + '\n')
	avgs.write(RPE_Opt_Output_DR_LC + '\n')
	
	avgs.close()
	
			
	#fig, ax = plt.subplots(nrows=1, ncols=2, layout='constrained')
	"""fig, ax = plt.subplots(nrows=1, ncols=2)
	
	ax[0].plot(x1, y1, x2, y2, x3, y3, x4, y4, x5, y5, x6, y6, x7, y7, x8, y8, x9, y9, x10, y10, x11, y11, x12, y12, x13, y13, x14, y14, x15, y15, x16, y16, marker='o', color='tab:gray', linewidth=4)
	ax[1].plot(x1, y1, x2, y2, x3, y3, x4, y4, x5, y5, x6, y6, x7, y7, x8, y8, x9, y9, x10, y10, x11, y11, x12, y12, x13, y13, x14, y14, x15, y15, x16, y16, marker='o', color='tab:gray', linewidth=4)

	ax[0].scatter(slamnodes_DR[:,0], slamnodes_DR[:,1],  c='g')
	ax[0].set_ylim([-18,18])
	ax[0].set_xlim([-18,18])
	ax[0].set_xlabel('X-Position (m)')
	ax[0].set_ylabel('Y-Position (m)')
	ax[0].set_title("Object Detection with DR")

	ax[1].scatter(slamnodes_DR_LC[:,0], slamnodes_DR_LC[:,1],  c='tab:cyan')
	ax[1].set_ylim([-18,18])
	ax[1].set_xlim([-18,18])
	ax[1].set_xlabel('X-Position (m)')
	ax[1].set_ylabel('Y-Position (m)')
	ax[1].set_title("Object Detection with DR-LC")
	plt.savefig('Data/' + folder + '/figures/DR_SLAM_Nodes.pdf', format="pdf")
"""

	y = np.arange(0,nodes, dtype=int)
	odom_RPE_File = 'Data/' + folder + '/odom_RPE.csv'
	opt_DR_RPE_File = 'Data/' + folder + '/opt_DR_RPE.csv'
	opt_DR_LC_RPE_File = 'Data/' + folder + '/opt_DR_LC_RPE.csv'


	odom_ATE_File = 'Data/' + folder + '/odom_ATE.csv'
	opt_DR_ATE_File = 'Data/' + folder + '/opt_DR_ATE.csv'
	opt_DR_LC_ATE_File = 'Data/' + folder + '/opt_DR_LC_ATE.csv'


	
	RPE_File1 = open(odom_RPE_File, "w+") # These make sure the file exists
	RPE_File2 = open(opt_DR_RPE_File, "w+")
	RPE_File3 = open(opt_DR_LC_RPE_File, "w+")

	ATE_File1 = open(odom_ATE_File, "w+") # These make sure the file exists
	ATE_File2 = open(opt_DR_ATE_File, "w+")
	ATE_File3 = open(opt_DR_LC_ATE_File, "w+")


	np.savetxt(odom_RPE_File, RPE_odom, delimiter=' ')
	np.savetxt(opt_DR_RPE_File, RPE_opt_DR, delimiter=' ')
	np.savetxt(opt_DR_LC_RPE_File, RPE_opt_DR_LC, delimiter=' ')
	np.savetxt(odom_ATE_File, ATE_odom, delimiter=' ')
	np.savetxt(opt_DR_ATE_File, ATE_opt_DR, delimiter=' ')
	np.savetxt(opt_DR_LC_ATE_File, ATE_opt_DR_LC, delimiter=' ')


	plt.figure(5)
	
	plt.plot(node_avg_odom_RPE, 'r', label='Noisy Odometry')
	plt.plot(node_avg_opt_RPE_DR, 'g', label='DR')
	plt.plot(node_avg_opt_RPE_DR_LC, 'tab:cyan', label='DR-LC')
	plt.ylabel('Average RPE (m)')
	plt.xlabel('Pose Graph Node')
	plt.title("Relative Position Error (RPE)")
	plt.legend()
	plt.savefig('Data/' + folder + '/figures/DR_RPE.pdf', format="pdf")
	
	plt.figure(6)

	
	plt.plot(node_avg_odom_ATE, 'r', label='Noisy Odometry')
	plt.plot(node_avg_opt_ATE_DR, 'g', label='DR')
	plt.plot(node_avg_opt_ATE_DR_LC, 'tab:cyan', label='DR-LC')
	plt.legend()
	plt.title("Absolute Trajectory Error (ATE)")
	plt.ylabel('Average ATE (m)')
	plt.xlabel('Pose Graph Node')

	plt.savefig('Data/' + folder + '/figures/DR_ATE.pdf', format="pdf")

	plt.title("ATE")
	
	
	#plt.show()

if __name__ == "__main__":
	main()

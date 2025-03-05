"""
 ----------------------------------------------------------------------------

 * File: RCS_Plotter.py
 * Author: Adam Hoburg
 * Date: 25 March 2024
 
 * Purpose: This code is intended to perform the error evaluation and plotting 
 for the RCS-SLAM pose graph approach. This scipt reads in data from ground truth, 
 noisy odometry and optimized estimates performs error comparison for both relative
 position error and absolute trajectory error then plots the actual/esimated trajectories
 and the error comparisons for RCS-SLAM with and without loop closure. 
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
				
	optimized_RCS = 'Data/' + folder + '/RCS_IneqOut.csv'
	optimized_RCS_LC = 'Data/' + folder + '/RCS_LC_IneqOut.csv'
	slamcsv = 'Data/' + folder + '/slam.csv'
	groundtruth = 'Data/' + folder + '/odom.bag'
	initial = 'Data/' + folder + '/cppgraph.csv'
	noisy_bag_file = 'Data/' + folder + '/noisy_odom.bag' 

	teamsize = 20

	init = open(initial, 'r+')
	gt = open(groundtruth, 'r+')
	
	opt_graph, initial_RCS = gtsam.readG2o(optimized_RCS, False)
	opt_graph, initial_RCS_LC = gtsam.readG2o(optimized_RCS_LC, False)

	
	resultPoses_RCS = gtsam.utilities.extractPose2(initial_RCS)
	resultPoses_RCS_LC = gtsam.utilities.extractPose2(initial_RCS_LC)

	
	nodes = int(len(resultPoses_RCS)/teamsize)
	#nodes = 400
	print("Nodes Per Robot: "+ str(nodes))
	
	rob1_RCS = np.zeros((nodes,3));
	rob1_RCS_LC = np.zeros((nodes,3));

	rob2_RCS = np.zeros((nodes,3)); 
	rob2_RCS_LC = np.zeros((nodes,3));

	
	slamnodes_RCS = SlamProc(slamcsv, optimized_RCS)
	slamnodes_RCS_LC = SlamProc(slamcsv, optimized_RCS_LC)

	##### This is done in Stats_Eval.py so commenting out because its extremely slow####
	#bag = bagreader(groundtruth)
	#odom = bag.odometry_data()
	
	#noisy_bag = bagreader(noisy_bag_file)
	#noisy_odom = noisy_bag.odometry_data()
	
	interactions = (teamsize/2)*(teamsize-1) # This interactions per node. Equation is sum of decreasing series cause r1 has 19 interactions, r2 has 18, etc.
	
	RPE_odom = np.zeros((int(interactions), nodes))
	RPE_opt_RCS = np.zeros((int(interactions), nodes))
	RPE_opt_RCS_LC = np.zeros((int(interactions), nodes))
	
	nodes_gt = np.zeros((nodes, teamsize,3))
	nodes_n = np.zeros((nodes, teamsize,3))


	ATE_odom = np.zeros((teamsize, nodes))
	ATE_opt_RCS = np.zeros((teamsize, nodes))	
	ATE_opt_RCS_LC = np.zeros((teamsize, nodes))	

	x1, y1 = [-10,10], [15, 15]
	x2, y2 = [10, 10], [10, 15]
	x3, y3 = [10, 15], [10, 10]
	x4, y4 = [15, 15], [10, -10]
	x5, y5 = [15,10], [-10, -10]
	x6, y6 = [10, 10], [-10, -15]
	x7, y7 = [10, -10], [-15, -15]
	x8, y8 = [-10, -10], [-15, -10]
	x9, y9 = [-10,-15], [-10, -10]
	x10, y10 = [-15, -15], [-10, 10]
	x11, y11 = [-15, -10], [10, 10]
	x12, y12 = [-10, -10], [10, 15]
	x13, y13 = [0,0], [10, 15]
	x14, y14 = [10, 15], [0, 0]
	x15, y15 = [0, 0], [-10, -15]
	x16, y16 = [-10, -15], [0, 0]


	k = 0
	p = 0

	#fig, ax = plt.subplots(nrows=1, ncols=2, layout='constrained')
	fig, ax = plt.subplots(nrows=1, ncols=2)
	
	for i in range(0,teamsize):
		odomfile = 'Data/' + folder + '/odom/Robot' + str((i+1)) + '-odom.csv'
		gt = np.loadtxt(odomfile,delimiter=',', skiprows=1, dtype=str)
		x = gt[:,4]
		y = gt[:,5]
		
		x = x.astype(float)
		y = y.astype(float) 
	

		label = "Ground Truth " + str(i+1)
		
		ax[0].plot(x,y, 'b', label="Ground Truth")
		
		step = int(len(gt)/nodes)
		#print(gt[0,:])
		for t in range(0,nodes):
			nodes_gt[t,i,0] = gt[(t*step),4] 
			nodes_gt[t,i,1] = gt[(t*step),5] 
			nodes_gt[t,i,2] = getTheta(gt[(t*step),7], gt[(t*step),8], gt[(t*step),9], gt[(t*step),10])
			
		
		
	for i in range(0,teamsize):
		noisyfile = 'Data/' + folder + '/noisy_odom/Robot' + str((i+1)) + '-noisy_odom.csv'
		#print(noisyfile)
		n = np.loadtxt(noisyfile,delimiter=',', skiprows=1, dtype=str)
		x = n[:,4]
		y = n[:,5]
		
		x = x.astype(float)
		y = y.astype(float)
				
		label = "Odometry " + str(i+1)
		
		ax[1].plot(x,y, 'r', label="Noisy Odometry")

		step = int(len(n)/nodes)
		
		for t in range(0,nodes):
			nodes_n[t,i,0] = n[(t*step),4] 
			nodes_n[t,i,1] = n[(t*step),5] 
			nodes_n[t,i,2] = getTheta(n[(t*step),7], n[(t*step),8], n[(t*step),9], n[(t*step),10])	

	print("Node Matricies Created")	
	ax[0].plot(x1, y1, x2, y2, x3, y3, x4, y4, x5, y5, x6, y6, x7, y7, x8, y8, x9, y9, x10, y10, x11, y11, x12, y12, x13, y13, x14, y14, x15, y15, x16, y16, marker='o', color='tab:gray', linewidth=4)
	ax[1].plot(x1, y1, x2, y2, x3, y3, x4, y4, x5, y5, x6, y6, x7, y7, x8, y8, x9, y9, x10, y10, x11, y11, x12, y12, x13, y13, x14, y14, x15, y15, x16, y16, marker='o', color='tab:gray', linewidth=4)


	ax[0].set_title('Ground Truth')
	ax[1].set_title('Noisy Odometry')
	
	ax[0].set_xlabel('X-Position (m)')
	ax[0].set_ylabel('Y-Position (m)')

	ax[1].set_xlabel('X-Position (m)')
	ax[1].set_ylabel('Y-Position (m)')
	
	ax[0].set_ylim([-18,18])
	ax[0].set_xlim([-18,18])
	ax[1].set_ylim([-18,18])
	ax[1].set_xlim([-18,18])
	
	plt.savefig('Data/' + folder + '/figures/RCS_GT_Noisy.pdf', format="pdf")
	
	print("Calculating Errors")

	#fig, ax = plt.subplots(nrows=1, ncols=2, layout='constrained')
	fig, ax = plt.subplots(nrows=1, ncols=2)
	
	for i in range(0,teamsize):
		for j in range((i*nodes),((i+1)*nodes)):
			rob1_RCS[k,0] = resultPoses_RCS[j,0];
			rob1_RCS[k,1] = resultPoses_RCS[j,1];
			rob1_RCS[k,2] = resultPoses_RCS[j,2];

			rob1_RCS_LC[k,0] = resultPoses_RCS_LC[j,0];
			rob1_RCS_LC[k,1] = resultPoses_RCS_LC[j,1];
			rob1_RCS_LC[k,2] = resultPoses_RCS_LC[j,2];

			k = k+1
			
		ax[0].plot(rob1_RCS[:,0],rob1_RCS[:,1], 'g', label='RCS')
		ax[1].plot(rob1_RCS_LC[:,0],rob1_RCS_LC[:,1], 'tab:cyan', label='RCS-LC')

		k=0
		
		
		gt1 = np.reshape(nodes_gt[:,i,:],(nodes,3))
		noisy1 = np.reshape(nodes_n[:,i,:],(nodes,3))
		
		ATE1, ATE2_RCS = ATE(gt1, noisy1, rob1_RCS, nodes, nodetime)
		ATE1, ATE2_RCS_LC = ATE(gt1, noisy1, rob1_RCS_LC, nodes, nodetime)
		
		ATE_odom[i,:] = ATE1
		ATE_opt_RCS[i,:] = ATE2_RCS
		ATE_opt_RCS_LC[i,:] = ATE2_RCS_LC
		
		
		for r in range((i+1),teamsize):
		
			m = 0
			for j in range((r*nodes),((r+1)*nodes)):
				rob2_RCS[m,0] = resultPoses_RCS[j,0];
				rob2_RCS[m,1] = resultPoses_RCS[j,1];
				rob2_RCS[m,2] = resultPoses_RCS[j,2];

				rob2_RCS_LC[m,0] = resultPoses_RCS_LC[j,0];
				rob2_RCS_LC[m,1] = resultPoses_RCS_LC[j,1];
				rob2_RCS_LC[m,2] = resultPoses_RCS_LC[j,2];


				m = m+1
				
			gt1 = np.reshape(nodes_gt[:,i,:],(nodes,3))
			noisy1 = np.reshape(nodes_n[:,i,:],(nodes,3))

			gt2 = np.reshape(nodes_gt[:,r,:],(nodes,3))
			noisy2 = np.reshape(nodes_n[:,r,:],(nodes,3))
			
			RPE1, RPE2_RCS = RPE(gt1, gt2, noisy1, noisy2, rob1_RCS, rob2_RCS, nodes)

			RPE1, RPE2_RCS_LC = RPE(gt1, gt2, noisy1, noisy2, rob1_RCS_LC, rob2_RCS_LC, nodes)

			
			RPE_odom[p,:] = RPE1
			RPE_opt_RCS[p,:] = RPE2_RCS
			RPE_opt_RCS_LC[p,:] = RPE2_RCS_LC
			p+=1


	ax[0].plot(x1, y1, x2, y2, x3, y3, x4, y4, x5, y5, x6, y6, x7, y7, x8, y8, x9, y9, x10, y10, x11, y11, x12, y12, x13, y13, x14, y14, x15, y15, x16, y16, marker='o', color='tab:gray', linewidth=4)
	ax[1].plot(x1, y1, x2, y2, x3, y3, x4, y4, x5, y5, x6, y6, x7, y7, x8, y8, x9, y9, x10, y10, x11, y11, x12, y12, x13, y13, x14, y14, x15, y15, x16, y16, marker='o', color='tab:gray', linewidth=4)

	ax[0].set_title('RCS')
	ax[1].set_title('RCS with LC')
	
	ax[0].set_xlabel('X-Position (m)')
	ax[0].set_ylabel('Y-Position (m)')

	ax[1].set_xlabel('X-Position (m)')
	ax[1].set_ylabel('Y-Position (m)')
	
	ax[0].set_ylim([-18,18])
	ax[0].set_xlim([-18,18])
	ax[1].set_ylim([-18,18])
	ax[1].set_xlim([-18,18])
	plt.savefig('Data/' + folder + '/figures/RCS_Opt_Nodes.pdf', format="pdf")
			

	
	mean_odom_ATE = np.mean(ATE_odom)
	mean_opt_ATE_RCS = np.mean(ATE_opt_RCS)
	mean_opt_ATE_RCS_LC = np.mean(ATE_opt_RCS_LC)


	node_avg_odom_ATE = np.mean(ATE_odom, axis=0)
	node_avg_opt_ATE_RCS = np.mean(ATE_opt_RCS, axis=0)
	node_avg_opt_ATE_RCS_LC = np.mean(ATE_opt_RCS_LC, axis=0)


	mean_odom_RPE = np.mean(RPE_odom) 
	mean_opt_RPE_RCS = np.mean(RPE_opt_RCS)
	mean_opt_RPE_RCS_LC = np.mean(RPE_opt_RCS_LC)

	
	node_avg_odom_RPE = np.mean(RPE_odom, axis=0)
	node_avg_opt_RPE_RCS = np.mean(RPE_opt_RCS, axis=0)	
	node_avg_opt_RPE_RCS_LC = np.mean(RPE_opt_RCS_LC, axis=0)	

	
	ATE_Odom_Output = 'Odometry ATE: ' + str(mean_odom_ATE)
	ATE_Opt_Output_RCS = 'RCS ATE: ' + str(mean_opt_ATE_RCS)
	ATE_Opt_Output_RCS_LC = 'RCS-LC ATE: ' + str(mean_opt_ATE_RCS_LC)

	
	RPE_Odom_Output = 'Odometry RPE: ' + str(mean_odom_RPE)
	RPE_Opt_Output_RCS = 'RCS RPE: ' + str(mean_opt_RPE_RCS)
	RPE_Opt_Output_RCS_LC = 'RCS-LC RPE: ' + str(mean_opt_RPE_RCS_LC)
	
	
	print(ATE_Odom_Output)
	print(ATE_Opt_Output_RCS)
	print(ATE_Opt_Output_RCS_LC)
	print(RPE_Odom_Output)
	print(RPE_Opt_Output_RCS)
	print(RPE_Opt_Output_RCS_LC)

	Avg_File = 'Data/' + folder + '/RCS_AVERAGES.csv'
	avgs = open(Avg_File, "w+")
	avgs.write(ATE_Odom_Output + '\n')
	avgs.write(ATE_Opt_Output_RCS + '\n')
	avgs.write(ATE_Opt_Output_RCS_LC + '\n')
	avgs.write(RPE_Odom_Output + '\n')
	avgs.write(RPE_Opt_Output_RCS + '\n')
	avgs.write(RPE_Opt_Output_RCS_LC + '\n')
	
	avgs.close()
	
			
	#fig, ax = plt.subplots(nrows=1, ncols=2, layout='constrained')
	fig, ax = plt.subplots(nrows=1, ncols=2)
	
	ax[0].plot(x1, y1, x2, y2, x3, y3, x4, y4, x5, y5, x6, y6, x7, y7, x8, y8, x9, y9, x10, y10, x11, y11, x12, y12, x13, y13, x14, y14, x15, y15, x16, y16, marker='o', color='tab:gray', linewidth=4)
	ax[1].plot(x1, y1, x2, y2, x3, y3, x4, y4, x5, y5, x6, y6, x7, y7, x8, y8, x9, y9, x10, y10, x11, y11, x12, y12, x13, y13, x14, y14, x15, y15, x16, y16, marker='o', color='tab:gray', linewidth=4)

	ax[0].scatter(slamnodes_RCS[:,0], slamnodes_RCS[:,1],  c='g')
	ax[0].set_ylim([-18,18])
	ax[0].set_xlim([-18,18])
	ax[0].set_xlabel('X-Position (m)')
	ax[0].set_ylabel('Y-Position (m)')
	ax[0].set_title("Object Detection with RCS")

	ax[1].scatter(slamnodes_RCS_LC[:,0], slamnodes_RCS_LC[:,1],  c='tab:cyan')
	ax[1].set_ylim([-18,18])
	ax[1].set_xlim([-18,18])
	ax[1].set_xlabel('X-Position (m)')
	ax[1].set_ylabel('Y-Position (m)')
	ax[1].set_title("Object Detection with RCS-LC")
	plt.savefig('Data/' + folder + '/figures/RCS_SLAM_Nodes.pdf', format="pdf")


	y = np.arange(0,nodes, dtype=int)
	odom_RPE_File = 'Data/' + folder + '/odom_RPE.csv'
	opt_RCS_RPE_File = 'Data/' + folder + '/opt_RCS_RPE.csv'
	opt_RCS_LC_RPE_File = 'Data/' + folder + '/opt_RCS_LC_RPE.csv'


	odom_ATE_File = 'Data/' + folder + '/odom_ATE.csv'
	opt_RCS_ATE_File = 'Data/' + folder + '/opt_RCS_ATE.csv'
	opt_RCS_LC_ATE_File = 'Data/' + folder + '/opt_RCS_LC_ATE.csv'


	
	RPE_File1 = open(odom_RPE_File, "w+") # These make sure the file exists
	RPE_File2 = open(opt_RCS_RPE_File, "w+")
	RPE_File3 = open(opt_RCS_LC_RPE_File, "w+")

	ATE_File1 = open(odom_ATE_File, "w+") # These make sure the file exists
	ATE_File2 = open(opt_RCS_ATE_File, "w+")
	ATE_File3 = open(opt_RCS_LC_ATE_File, "w+")


	np.savetxt(odom_RPE_File, RPE_odom, delimiter=' ')
	np.savetxt(opt_RCS_RPE_File, RPE_opt_RCS, delimiter=' ')
	np.savetxt(opt_RCS_LC_RPE_File, RPE_opt_RCS_LC, delimiter=' ')
	np.savetxt(odom_ATE_File, ATE_odom, delimiter=' ')
	np.savetxt(opt_RCS_ATE_File, ATE_opt_RCS, delimiter=' ')
	np.savetxt(opt_RCS_LC_ATE_File, ATE_opt_RCS_LC, delimiter=' ')


	plt.figure(5)
	'''
	for i in range(0, nodes, 40):

		plt.boxplot(RPE_odom[:,i], positions=[i], widths=[5.0], showfliers=False)
		
	for j in range(0, nodes, 35):
		plt.boxplot(RPE_opt_RCS[:,j], positions=[j], widths=[5.0], showfliers=False)
	'''	
	
	plt.plot(node_avg_odom_RPE, 'r', label='Noisy Odometry')
	plt.plot(node_avg_opt_RPE_RCS, 'g', label='RCS')
	plt.plot(node_avg_opt_RPE_RCS_LC, 'tab:cyan', label='RCS-LC')
	plt.ylabel('Average RPE (m)')
	plt.xlabel('Pose Graph Node')
	plt.title("Relative Position Error (RPE)")
	plt.legend()
	plt.savefig('Data/' + folder + '/figures/RCS_RPE.pdf', format="pdf")
	
	plt.figure(6)
	'''
	for i in range(0, nodes, 40):

		plt.boxplot(ATE_odom[:,i], positions=[i], widths=[5.0], showfliers=False)
		
	for j in range(0,nodes, 35):
		plt.boxplot(ATE_opt_RCS[:,j], positions=[j], widths=[5.0], showfliers=False)
	'''	
	
	plt.plot(node_avg_odom_ATE, 'r', label='Noisy Odometry')
	plt.plot(node_avg_opt_ATE_RCS, 'g', label='RCS')
	plt.plot(node_avg_opt_ATE_RCS_LC, 'tab:cyan', label='RCS-LC')
	plt.legend()
	plt.title("Absolute Trajectory Error (ATE)")
	plt.ylabel('Average ATE (m)')
	plt.xlabel('Pose Graph Node')

	plt.savefig('Data/' + folder + '/figures/RCS_ATE.pdf', format="pdf")

	plt.title("ATE")
	
	
	#plt.show()

if __name__ == "__main__":
	main()

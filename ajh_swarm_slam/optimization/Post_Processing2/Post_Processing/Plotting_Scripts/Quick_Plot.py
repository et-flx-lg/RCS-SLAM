
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
from metrics import ATE, RPE


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
	parser.add_argument('-m', '--map', type=float, help='map')

	parser.add_argument(
		'-g',
		'--groundtruth', type=str,
		help="ground truth bag file.")

	args = parser.parse_args()
	
	folder = 'Current' if args.folder is None\
		else args.folder

	nodetime = 3 if args.nodetime is None\
		else args.nodetime
	
	Mp = 'house' if args.map is None\
		else args.map
	
		
	optimized = 'Data/' + folder + '/IneqOut.csv'
	slamcsv = 'Data/' + folder + '/slam.csv'
	groundtruth = 'Data/' + folder + '/odom.bag'
	initial = 'Data/' + folder + '/cppgraph.csv'
	RPE_odom_file = 'Data/' + folder + '/odom_RPE.csv'
	RPE_opt_file =  'Data/' + folder + '/opt_RPE.csv'
	ATE_odom_file = 'Data/' + folder + '/odom_ATE.csv'
	ATE_opt_file =  'Data/' + folder + '/opt_ATE.csv'


	teamsize = 20

	init = open(initial, 'r+')
	gt = open(groundtruth, 'r+')
	
	opt_graph, initial = gtsam.readG2o(optimized, False)
	
	resultPoses = gtsam.utilities.extractPose2(initial)
	
	#nodes = int(len(resultPoses)/teamsize)
	nodes = 400
	print("Nodes Per Robot: "+ str(nodes))
	
	rob1 = np.zeros((nodes,3));
	rob2 = np.zeros((nodes,3)); 
	
	slamnodes = SlamProc(slamcsv, optimized)
		
	interactions = (teamsize/2)*(teamsize-1) # This interactions per node. Equation is sum of decreasing series cause r1 has 19 interactions, r2 has 18, etc.
	

	k = 0
	t = 0

	fig, ax = plt.subplots(nrows=1, ncols=3, layout='constrained')
	print("Plotting Ground Truth")
	for i in range(0,teamsize):
		odom = 'Data/' + folder + '/odom/Robot' + str(i+1) + '-odom.csv'
		odomdata = np.loadtxt(odom,delimiter=',', skiprows=1, dtype=str)
		x = odomdata[:,4]
		y = odomdata[:,5]
		
		x = x.astype(float)
		y = y.astype(float)
		
		label = "Ground Truth " + str(i+1)
		
		ax[0].plot(x,y, 'b', label="Ground Truth")
	print("Plotting Noisy Odometry")	
	for i in range(0,teamsize):
		noisy_odom = 'Data/' + folder + '/noisy_odom/Robot' + str(i+1) + '-noisy_odom.csv'
		odomdata = np.loadtxt(noisy_odom,delimiter=',', skiprows=1, dtype=str)
		x = odomdata[:,4]
		y = odomdata[:,5]
		
		x = x.astype(float)
		y = y.astype(float)
		
		label = "Odometry " + str(i+1)
		
		ax[1].plot(x,y, 'r', label="Odometry")
		
	print("Plotting Optimized")
	for i in range(0,teamsize):
		for j in range((i*nodes),((i+1)*nodes)):
			rob1[k,0] = resultPoses[j,0];
			rob1[k,1] = resultPoses[j,1];
			rob1[k,2] = resultPoses[j,2];
			k = k+1
			
		ax[2].plot(rob1[:,0],rob1[:,1], 'g', label='Optimized')
		k=0
		#ax[2]
		
				
		
						

	RPE_odom = np.loadtxt(RPE_odom_file, delimiter=' ', dtype=float)
	RPE_opt = np.loadtxt(RPE_opt_file, delimiter=' ', dtype=float)
	
	ATE_odom = np.loadtxt(ATE_odom_file, delimiter=' ', dtype=float)
	ATE_opt = np.loadtxt(ATE_opt_file, delimiter=' ', dtype=float)

	mean_odom_ATE = np.mean(ATE_odom)
	mean_opt_ATE = np.mean(ATE_opt)

	node_avg_odom_ATE = np.mean(ATE_odom, axis=0)
	node_avg_opt_ATE = np.mean(ATE_opt, axis=0)

	mean_odom_RPE = np.mean(RPE_odom) 
	mean_opt_RPE = np.mean(RPE_opt)
	
	node_avg_odom_RPE = np.mean(RPE_odom, axis=0)
	node_avg_opt_RPE = np.mean(RPE_opt, axis=0)	
	
	ATE_Odom_Output = 'ATE Odometry: ' + str(mean_odom_ATE)
	ATE_Opt_Output = 'ATE Optimized: ' + str(mean_opt_ATE)
	
	RPE_Odom_Output = 'RPE Odometry: ' + str(mean_odom_RPE)
	RPE_Opt_Output = 'RPE Optimized: ' + str(mean_opt_RPE)
	
	print(np.max(RPE_odom))
	print(np.max(RPE_opt))

	print(np.max(ATE_odom))
	print(np.max(ATE_opt))
	
	
	print(ATE_Odom_Output)
	print(ATE_Opt_Output)
	print(RPE_Odom_Output)
	print(RPE_Opt_Output)
	
		
	if Mp == "house":
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

		ax[0].plot(x1, y1, x2, y2, x3, y3, x4, y4, x5, y5, x6, y6, x7, y7, x8, y8, x9, y9, x10, y10, x11, y11, x12, y12, x13, y13, x14, y14, x15, y15, x16, y16, marker='o', color='tab:gray', linewidth=4)
		ax[1].plot(x1, y1, x2, y2, x3, y3, x4, y4, x5, y5, x6, y6, x7, y7, x8, y8, x9, y9, x10, y10, x11, y11, x12, y12, x13, y13, x14, y14, x15, y15, x16, y16, marker='o', color='tab:gray', linewidth=4)
		ax[2].plot(x1, y1, x2, y2, x3, y3, x4, y4, x5, y5, x6, y6, x7, y7, x8, y8, x9, y9, x10, y10, x11, y11, x12, y12, x13, y13, x14, y14, x15, y15, x16, y16, marker='o', color='tab:gray', linewidth=4)

	
	elif Mp == "box":
		x1, y1 = [-10,-10], [-10, 10]
		x2, y2 = [-10, 10], [10, 10]
		x3, y3 = [10, 10], [10, -10]
		x4, y4 = [10, -10], [-10, -10]

		ax[0].plot(x1, y1, x2, y2, x3, y3, x4, y4, marker='o')
		ax[1].plot(x1, y1, x2, y2, x3, y3, x4, y4, marker='o')
		ax[2].plot(x1, y1, x2, y2, x3, y3, x4, y4, marker='o')

	ax[0].set_ylim([-18,18])
	ax[0].set_xlim([-18,18])
	ax[1].set_ylim([-18,18])
	ax[1].set_xlim([-18,18])
	ax[2].set_ylim([-18,18])
	ax[2].set_xlim([-18,18])
	
	fig.supxlabel('X-Position (m)')
	fig.supylabel('Y-Position (m)')
	
	ax[0].set_title('Ground Truth')
	ax[1].set_title('Noisy Odometry')
	ax[2].set_title('Optimized Estimate')
	plt.savefig("IROSPlots/Nodes_MN.pdf", format="pdf")

	plt.figure(2)
	if Mp == "house":
		plt.plot(x1, y1, x2, y2, x3, y3, x4, y4, x5, y5, x6, y6, x7, y7, x8, y8, x9, y9, x10, y10, x11, y11, x12, y12, x13, y13, x14, y14, x15, y15, x16, y16, marker='o', color='tab:gray', linewidth=4)
	elif Mp == "box":
		plt.plot(x1, y1, x2, y2, x3, y3, x4, y4, marker='o')

	plt.scatter(slamnodes[:,0], slamnodes[:,1], label='Detected Object')
	plt.xlabel('X-Position (m)')
	plt.ylabel('Y-Position (m)')

	plt.figure(3)
	for i in range(0, nodes, 40):

		plt.boxplot(RPE_odom[:,i], positions=[i], widths=[10.0], showfliers=False, manage_ticks=False)
		
	for j in range(20, nodes, 40):
		plt.boxplot(RPE_opt[:,j], positions=[j], widths=[10.0], showfliers=False, manage_ticks=False)
		
	
	plt.plot(node_avg_odom_RPE, 'r')
	plt.plot(node_avg_opt_RPE, 'g')
	plt.ylabel('RPE (m)')
	plt.xlabel('Trajectory Node')
	plt.title("Relative Position Error (RPE)")
	plt.savefig("IROSPlots/RPE_MN.pdf", format="pdf")

	
	plt.figure(4)
	for i in range(0, nodes, 40):

		plt.boxplot(ATE_odom[:,i], positions=[i], widths=[10.0], showfliers=False, manage_ticks=False)
		
	for j in range(20,nodes, 40):
		plt.boxplot(ATE_opt[:,j], positions=[j], widths=[10.0], showfliers=False, manage_ticks=False)
		
	
	plt.plot(node_avg_odom_ATE, 'r')
	plt.plot(node_avg_opt_ATE, 'g')
	plt.ylabel('ATE (m)')
	plt.xlabel('Trajectory Node')
	plt.title("Absolute Trajectory Error (ATE)")
	plt.savefig("IROSPlots/ATE_MN.pdf", format="pdf")
	
	
	plt.show()

if __name__ == "__main__":
	main()

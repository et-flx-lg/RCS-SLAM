
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



def main():
		
	odom_MN = 'Data/20min_House_Joy_MN/odom_RPE.csv'			

	opt_MN = 'Data/20min_House_Joy_MN/opt_RPE.csv'			
	opt_LC = 'Data/20min_House_Joy_LC_MN/opt_RPE.csv'
	teamsize = 20

	
	odom_MN_RPE = np.loadtxt(odom_MN,delimiter=' ', dtype=float)

	opt_MN_RPE = np.loadtxt(opt_MN,delimiter=' ', dtype=float)
	opt_LC_RPE = np.loadtxt(opt_LC,delimiter=' ', dtype=float)

	
	odom_avg_MN_RPE = np.mean(odom_MN_RPE, axis=0)

	opt_avg_MN_RPE = np.mean(opt_MN_RPE, axis=0)
	opt_avg_LC_RPE = np.mean(opt_LC_RPE, axis=0)

	ATE_odom_MN = 'Data/20min_House_Joy_MN/odom_ATE.csv'			

	ATE_opt_MN = 'Data/20min_House_Joy_MN/opt_ATE.csv'			
	ATE_opt_LC = 'Data/20min_House_Joy_LC_MN/opt_ATE.csv'
	optimized = 'Data/20min_House_Joy_LC_MN/IneqOut.csv'

	
	odom_MN_ATE = np.loadtxt(ATE_odom_MN,delimiter=' ', dtype=float)

	opt_MN_ATE = np.loadtxt(ATE_opt_MN,delimiter=' ', dtype=float)
	opt_LC_ATE = np.loadtxt(ATE_opt_LC,delimiter=' ', dtype=float)

	
	odom_avg_MN_ATE = np.mean(odom_MN_ATE, axis=0)

	opt_avg_MN_ATE = np.mean(opt_MN_ATE, axis=0)
	opt_avg_LC_ATE = np.mean(opt_LC_ATE, axis=0)
	
	nodes = 400

	opt_graph, initial = gtsam.readG2o(optimized, False)
	resultPoses = gtsam.utilities.extractPose2(initial)
	rob1 = np.zeros((nodes,3));
	
	plt.figure(1)
	
	plt.plot(odom_avg_MN_RPE, 'r', label='Medium Noise Odometry')

	plt.plot(opt_avg_MN_RPE, 'g', linestyle='dashed', label='RCS-SLAM')
	plt.plot(opt_avg_LC_RPE, 'tab:cyan', linestyle='dashed', label='RCS-SLAM with Loop Closure')


	plt.xlabel('Trajectory Node')
	plt.ylabel('RPE (m)')
	plt.legend()
	plt.title("RPE Comparison of RCS-SLAM with Loop Closure")
	plt.savefig("IROSPlots/LC_RPE.pdf", format="pdf")

	plt.figure(2)
	
	plt.xlabel('Trajectory Node')
	plt.ylabel('ATE (m)')
	plt.title('Absolute Trajectory Error (ATE)')
	plt.plot(odom_avg_MN_ATE, 'r', label='Medium Noise Odometry')
	plt.plot(opt_avg_MN_ATE, 'g', linestyle='dashed', label='RCS-SLAM')
	plt.plot(opt_avg_LC_ATE, 'tab:cyan', linestyle='dashed', label='RCS-SLAM with Loop Closure')
	plt.legend()
	plt.savefig("IROSPlots/LC_ATE.pdf", format="pdf")

	plt.title("ATE Comparison of RCS-SLAM with Loop Closure")
	
	plt.figure(3)

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


	for i in range(0,teamsize):
		k=0

		for j in range((i*nodes),((i+1)*nodes)):
			rob1[k,0] = resultPoses[j,0];
			rob1[k,1] = resultPoses[j,1];
			rob1[k,2] = resultPoses[j,2];
			k = k+1
			
		plt.plot(rob1[:,0],rob1[:,1], 'tab:cyan', label='Optimized')
	
	plt.plot(x1, y1, x2, y2, x3, y3, x4, y4, x5, y5, x6, y6, x7, y7, x8, y8, x9, y9, x10, y10, x11, y11, x12, y12, x13, y13, x14, y14, x15, y15, x16, y16, marker='o', color='tab:gray', linewidth=4)
	plt.xlabel('X-Position (m)')
	plt.ylabel('Y-Position (m)')
	plt.title("RCS-SLAM with Loop Closure")
	plt.savefig("IROSPlots/LC_OptimizedEst.pdf", format="pdf")

	
		
	
	plt.show()

if __name__ == "__main__":
	main()

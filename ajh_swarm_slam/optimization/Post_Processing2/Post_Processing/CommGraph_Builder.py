"""
 ----------------------------------------------------------------------------

 * File: CommGraph_Builder.py
 * Author: Adam Hoburg
 * Date: 25 March 2024
 
 * Purpose: This code allows for a new communication graph to be generated based
 upon existing ground truth data from a simulation. This was intended for generating
 communication graphs at varying communication ranges.

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





def main():
	"""Main runner."""

	parser = argparse.ArgumentParser(
		description="Plot initial and optimizied graphs.")
	parser.add_argument('-f', '--folder', type=str,help='Current Data File.')
	parser.add_argument('-t', '--nodetime', type=float, help='Node Interval')
	parser.add_argument('-r', '--range', type=float, help='Comm Range')

	args = parser.parse_args()
	
	folder = 'Current' if args.folder is None\
		else args.folder

	rng = 2.0 if args.range is None\
		else args.range

	nodetime = 3 if args.nodetime is None\
		else args.nodetime
				
	groundtruth = 'Data/' + folder + '/odom.bag'
	#commgraph = 'Data/' + folder + '/commgraph_'+ str(int(rng))+'m.csv'
	commgraph = 'Data/' + folder + '/commgraph.csv'

	teamsize = 20

	#nodes = int(len(resultPoses)/teamsize)
	nodes = 400
	print("Nodes Per Robot: "+ str(nodes))
		
	bag = bagreader(groundtruth)
	odom = bag.odometry_data()
	
	Comm = open(commgraph, "w+")
	
	traj = np.zeros((nodes,teamsize, 2))

	for r in range(0,teamsize): # r is the robot number, k is the n
		odomfile = 'Data/' + folder + '/odom/Robot' + str((r+1)) + '-odom.csv'
		gt = np.loadtxt(odomfile,delimiter=',', skiprows=1, dtype=str)
		step = int(len(gt)/nodes)
		
		for k in range(0,nodes):
			traj[k,r,0] = gt[(k*step),4] # Ground Truth samples about .02s. Should be 250
			traj[k,r,1] = gt[(k*step),5]

			
		
		
	
	for n in range(0,nodes):
		c_graph = np.zeros((teamsize,teamsize))

		for i in range(0,teamsize):

			for j in range(i,teamsize):
				
				P1 = np.array([[traj[n,i,0]],[traj[n,i,1]]])
				P2 = np.array([[traj[n,j,0]],[traj[n,j,1]]])
	
				diff = np.subtract(P1, P2)
				R = np.linalg.norm(diff)

		
				
				if R <= rng:
					c_graph[i,j] = round(R,3)
					c_graph[j,i] = round(R,3)
					
		c_graph = c_graph.reshape((1,400))
		np.savetxt(Comm, c_graph, delimiter=',')
			
	print("Done!")

if __name__ == "__main__":
	main()

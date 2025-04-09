"""
 ----------------------------------------------------------------------------

 * File: BagPostProc.py
 * Author: Adam Hoburg
 * Date: 25 March 2024
 
 * Purpose: This code is used to take in the rosbags from the simulations, 
 convert then to .csvs then also evaluate the communication graph to determine 
 when relay communications would have taken place. Finally, it created the .csv file
 containing the loop closure communications with the stationary robot. 

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
import os


def main():
	"""Main runner."""

	parser = argparse.ArgumentParser()
	parser.add_argument('-f', '--folder', type=str,help='Desired data folder')
	parser.add_argument('-t', '--teamsize', type=int,help='Number of agents.')
	
	args = parser.parse_args()
	
	folder = 'Current' if args.folder is None\
		else args.folder
	
	##etwas geaendert war 20
	team = 2 if args.teamsize is None\
		else args.teamsize
	
	
	comm_bag = 'Data/' + folder + '/commgraph.bag'
	#slam_bag = 'Data/' + folder + '/slam.bag'
	central_bag = 'Data/' + folder + '/centralgraph.bag'
	comm_csv = 'Data/' + folder + '/commgraph.csv'
	central_csv = 'Data/' + folder + '/cppgraph.csv' # This file name is used because the noise characteristics are updated in the Stats_Eval.py script. I changed it because I dont do Stats_estiamte
	#slam_csv = 'Data/' + folder + '/slam.csv'
	comm_out = 'Data/' + folder + '/cppcomm.csv'
	hop_csv = 'Data/' + folder + '/hopgraph.csv' 
	loop_closure = 'Data/' + folder + '/loopclosure.csv'
		
	bc.b2csv(comm_bag, comm_csv, '/commgraph')
	bc.b2csv(central_bag, central_csv, '/central_graph')
	  
	rob1 = 0
	rob2 = 0
	
	comm_out = open(comm_out,'w+')
	cent = open(central_csv, 'r+')
	hop = open(hop_csv,'w+')
	loop = open(loop_closure, 'w+')
	centgraph = cent.readlines()
	comm = np.loadtxt(comm_csv,delimiter=',', usecols=np.arange(0,(int(math.pow(team,2)))), comments=')',dtype=str) # 56 for 10, 211 for 20
	
	graph = np.zeros((team,team))
	
	print(len(comm))             
	for i in range(0,len(comm)): #04March - Discovered swarmgraph wasn't publishing commgraph at node zero. Switched starting point to 1.
		n = 0
		comm[i,0] = '0.0'
		for k in range(0,team):
			for m in range(0,team):
		
				graph[k,m] = comm[i,n]
				
				n = n+1  
	  
	  # Checking for direct communications
		for j in range(0,(team)):
			for k in range(j,(team)):
			
				if (float(graph[j,k]) != 0.0) and (graph[j,k] != 88.0):# and (k>=j):
		  
					r1 = str(j+1)
					r2 = str(k+1)
									
					if i>=100:
						rob1 = r1 + str(i)
						rob2 = r2 + str(i)
					elif i>=10:
						rob1 = r1 + "0" + str(i)
						rob2 = r2 + "0" + str(i)
					else:
						rob1 = r1 + "00" + str(i)
						rob2 = r2 + "00" + str(i)
				
					while rob1 !=0 and rob2 !=0:
						edge1 = rob1 + " " + rob2 + " " + str(graph[j,k])
						edge2 = rob2 + " " + rob1 
						rob1 = 0
						rob2 = 0
						comm_out.write(edge1 + '\n')
						#comm_out.write(edge2 + '\n')
				
				
					#Checking for one-hop comms
					for l in range(0,team):
						if (float(graph[k,l]) != 0.0) and (l!=j) and (float(graph[j,l]) == 0.0):
							r1 = str(j+1)
							r2 = str(l+1)
				
							if i>=100:
								rob1 = r1 + str(i)
								rob2 = r2 + str(i)
							elif i>=10:
								rob1 = r1 + "0" + str(i)
								rob2 = r2 + "0" + str(i)
							else:
								rob1 = r1 + "00" + str(i)
								rob2 = r2 + "00" + str(i)
				
							while rob1 !=0 and rob2 !=0:
								edge1 = rob1 + " " + rob2
								edge2 = rob2 + " " + rob1 
								rob1 = 0
								rob2 = 0
								hop.write(edge1 + '\n')
								hop.write(edge2 + '\n')
								
							graph[j,l] = 88.0; #This is to account for previous hops applied without adding additional direct comms. 88 is a completely random number.

	
	bag = bagreader('Data/' + folder + '/groundtruth.bag')
	for t in bag.topics:
    		data = bag.message_by_topic(t)
    		
    		original_csv_path = data
    		
    		if "1" in t:
        		new_name = 'Data/' + folder + '/groundtruth/Robot1.csv'
    		elif "2" in t:
        		new_name = 'Data/' + folder + '/groundtruth/Robot2.csv'
    		else:
        		new_name = 'Data/' + folder + '/groundtruth/Other.csv'
    		os.rename(original_csv_path, new_name)
        	
    		print(f"Datei umbenannt: {original_csv_path} -> {new_name}")
        	
	bag = bagreader('Data/' + folder + '/bumps.bag')
	for t in bag.topics:
    		data = bag.message_by_topic(t)
    		
    		original_csv_path = data
    		
    		if "1" in t:
        		new_name = 'Data/' + folder + '/bumps/Robot1.csv'
    		elif "2" in t:
        		new_name = 'Data/' + folder + '/bumps/Robot2.csv'
    		else:
        		new_name = 'Data/' + folder + '/bumps/Other.csv'
    		os.rename(original_csv_path, new_name)
        	
    		print(f"Datei umbenannt: {original_csv_path} -> {new_name}") 
    		

	bag = bagreader('Data/' + folder + '/odom.bag')
	odom = bag.odometry_data()					
	s_bot = np.array([[0.25],[0.25]])
	gt_n = np.zeros((2,1))				
	rob1 = 0
	rob2 = 0
	nodes = len(comm)
	for i in range(0,team):
		
		odomfile_abs = 'Data/' + folder + '/odom/Robot' + str((i+1)) + '-odom_abs.csv'
		odomfile = 'Data/' + folder + '/odom/Robot' + str((i+1)) + '-odom.csv'
		if os.path.exists(odomfile_abs):
    			os.rename(odomfile_abs, odomfile)
    			print(f"File renamed from {odomfile_abs} to {odomfile}")
		else:
    			print(f"File {odomfile_abs} does not exist!")
    			
		gt = np.loadtxt(odomfile,delimiter=',', skiprows=1, dtype=str)
		
		step1 = int(len(gt)/nodes)
		
		#print( len(gt))
		#print(len(noisy))
				
		for k in range(0,nodes):
			gt_n[0,0] = gt[(k*step1),4] 
			gt_n[1,0] = gt[(k*step1),5]
			
			diff = np.subtract(gt_n, s_bot)
			meas = np.linalg.norm(diff)
			
			if meas <= 1.0:
				r1 = str(i+1)
				r2 = str(21)
				#rob2 = str(j+1) + "000"
				if k>=100:
					rob1 = r1 + str(k)
					rob2 = r2 + str(k)
              
				elif k>=10:
					rob1 = r1 + "0" + str(k)
					rob2 = r2 + "0" + str(k)
                 
				else:
					rob1 = r1 + "00" + str(k)
					rob2 = r2 + "00" + str(k)

				while rob1 !=0 and rob2 !=0:
					edge = rob1 + " " + rob2 + " " + str(meas)
					rob1 = 0
					rob2 = 0
					loop.write(edge + '\n')
					
					
					
	comm_out.close()
	cent.close()
	hop.close()
	
	print("Central Graph, Comm Graph, and hop graph Files saved.")


if __name__ == "__main__":
	main()

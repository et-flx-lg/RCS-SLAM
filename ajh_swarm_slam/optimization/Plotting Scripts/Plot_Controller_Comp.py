
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

	parser = argparse.ArgumentParser(
		description="Plot comparison of all four optimization approaches.")
	parser.add_argument('-n', '--n', type=str, help='noise level')

	args = parser.parse_args()
	
	n = args.n

			
	odom_Random = 'Data/20min_House_Random_' + n + '/odom_RPE.csv'
	odom_Leader = 'Data/20min_House_Leader_' + n + '/odom_RPE.csv'
	odom_Joy = 'Data/20min_House_Joy_' + n + '/odom_RPE.csv'

	RPE_Random = 'Data/20min_House_Random_' + n + '/opt_RCS_LC_RPE.csv'
	RPE_Leader = 'Data/20min_House_Leader_' + n + '/opt_RCS_LC_RPE.csv'
	RPE_Joy = 'Data/20min_House_Joy_' + n + '/opt_RCS_LC_RPE.csv'

	
	odom_R_RPE = np.loadtxt(odom_Random,delimiter=' ', dtype=float)
	odom_L_RPE = np.loadtxt(odom_Leader,delimiter=' ', dtype=float)
	odom_J_RPE = np.loadtxt(odom_Joy,delimiter=' ', dtype=float)

	RPE_R = np.loadtxt(RPE_Random, delimiter=' ', dtype=float)
	RPE_L = np.loadtxt(RPE_Leader, delimiter=' ', dtype=float)
	RPE_J = np.loadtxt(RPE_Joy, delimiter=' ', dtype=float)

	
	avg_RPE_odom_R = np.mean(odom_R_RPE, axis=0)
	avg_RPE_odom_L = np.mean(odom_L_RPE, axis=0)
	avg_RPE_odom_J = np.mean(odom_J_RPE, axis=0)

	avg_RPE_R = np.mean(RPE_R, axis=0)
	avg_RPE_L = np.mean(RPE_L, axis=0)
	avg_RPE_J = np.mean(RPE_J, axis=0)

	odom_Random = 'Data/20min_House_Random_' + n + '/odom_ATE.csv'
	odom_Leader = 'Data/20min_House_Leader_' + n + '/odom_ATE.csv'
	odom_Joy = 'Data/20min_House_Joy_' + n + '/odom_ATE.csv'

	ATE_Random = 'Data/20min_House_Random_' + n + '/opt_RCS_LC_ATE.csv'
	ATE_Leader = 'Data/20min_House_Leader_' + n + '/opt_RCS_LC_ATE.csv'
	ATE_Joy = 'Data/20min_House_Joy_' + n + '/opt_RCS_LC_ATE.csv'

	
	odom_R_ATE = np.loadtxt(odom_Random,delimiter=' ', dtype=float)
	odom_L_ATE = np.loadtxt(odom_Leader,delimiter=' ', dtype=float)
	odom_J_ATE = np.loadtxt(odom_Joy,delimiter=' ', dtype=float)

	ATE_R = np.loadtxt(ATE_Random, delimiter=' ', dtype=float)
	ATE_L = np.loadtxt(ATE_Leader, delimiter=' ', dtype=float)
	ATE_J = np.loadtxt(ATE_Joy, delimiter=' ', dtype=float)

	
	avg_ATE_odom_R = np.mean(odom_R_ATE, axis=0)
	avg_ATE_odom_L = np.mean(odom_L_ATE, axis=0)
	avg_ATE_odom_J = np.mean(odom_J_ATE, axis=0)

	avg_ATE_R = np.mean(ATE_R, axis=0)
	avg_ATE_L = np.mean(ATE_L, axis=0)
	avg_ATE_J = np.mean(ATE_J, axis=0)

	
		
	plt.figure(1)
	
	#plt.plot(avg_RPE_odom_R, 'b', label='Random')	
	#plt.plot(avg_RPE_odom_L, 'g', label='Leader-Follower')	
	#plt.plot(avg_RPE_odom_J, 'm', label='Centralized')	

	plt.plot(avg_RPE_R, 'b', label='Random')
	plt.plot(avg_RPE_L, 'g', label='Leader')
	plt.plot(avg_RPE_J, 'm', label='Aggregation')


	plt.xlabel('Pose Graph Node')
	plt.ylabel('Average RPE (m)')
	plt.legend()
	plt.title("Relative Position Error (RPE)")
	plt.savefig("ThesisPlots/Controller_Comp_RPE_" + n + ".pdf", format="pdf")

	plt.figure(2)
	
	#plt.plot(avg_ATE_odom_R, 'b', label='Random')	
	#plt.plot(avg_ATE_odom_L, 'g', label='Leader-Follower')	
	#plt.plot(avg_ATE_odom_J, 'm', label='Centralized')	

	plt.plot(avg_ATE_R, 'b', label='Random')
	plt.plot(avg_ATE_L, 'g', label='Leader')
	plt.plot(avg_ATE_J, 'm', label='Aggregation')


	plt.xlabel('Pose Graph Node')
	plt.ylabel('Average ATE (m)')
	plt.legend()
	plt.title("Absolute Trajectory Error (ATE)")
	plt.savefig("ThesisPlots/Controller_Comp_ATE_" + n + ".pdf", format="pdf")
		
	
	plt.show()

if __name__ == "__main__":
	main()

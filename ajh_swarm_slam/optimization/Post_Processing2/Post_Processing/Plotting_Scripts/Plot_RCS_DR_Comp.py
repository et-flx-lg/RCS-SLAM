
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
	parser.add_argument('-c', '--c', type=str,help='Controller.')
	parser.add_argument('-n', '--n', type=str, help='noise level')

	args = parser.parse_args()
	
	c = args.c
	n = args.n

			
	odom = 'Data/20min_House_' + c + '_' + n + '/odom_RPE.csv'

	RPE_RCS = 'Data/20min_House_' + c + '_' + n + '/opt_RCS_RPE.csv'
	RPE_RCS_LC = 'Data/20min_House_' + c + '_' + n + '/opt_RCS_LC_RPE.csv'			
	RPE_DR = 'Data/20min_House_' + c + '_' + n + '/opt_DR_RPE.csv'
	RPE_DR_LC = 'Data/20min_House_' + c + '_HN/opt_DR_LC_RPE.csv'

	
	odom_RPE = np.loadtxt(odom,delimiter=' ', dtype=float)

	opt_RPE_RCS = np.loadtxt(RPE_RCS, delimiter=' ', dtype=float)
	opt_RPE_RCS_LC = np.loadtxt(RPE_RCS_LC,delimiter=' ', dtype=float)
	opt_RPE_DR = np.loadtxt(RPE_DR,delimiter=' ', dtype=float)
	opt_RPE_DR_LC = np.loadtxt(RPE_DR_LC,delimiter=' ', dtype=float)

	
	avg_RPE_odom = np.mean(odom_RPE, axis=0)

	avg_RPE_RCS = np.mean(opt_RPE_RCS, axis=0)
	avg_RPE_RCS_LC = np.mean(opt_RPE_RCS_LC, axis=0)
	avg_RPE_DR = np.mean(opt_RPE_DR, axis=0)
	avg_RPE_DR_LC = np.mean(opt_RPE_DR_LC, axis=0)

	odom = 'Data/20min_House_' + c + '_' + n + '/odom_ATE.csv'

	ATE_RCS = 'Data/20min_House_' + c + '_' + n + '/opt_RCS_ATE.csv'
	ATE_RCS_LC = 'Data/20min_House_' + c + '_' + n + '/opt_RCS_LC_ATE.csv'			
	ATE_DR = 'Data/20min_House_' + c + '_' + n + '/opt_DR_ATE.csv'
	ATE_DR_LC = 'Data/20min_House_' + c + '_' + n + '/opt_DR_LC_ATE.csv'

	
	odom_ATE = np.loadtxt(odom,delimiter=' ', dtype=float)

	opt_ATE_RCS = np.loadtxt(ATE_RCS, delimiter=' ', dtype=float)
	opt_ATE_RCS_LC = np.loadtxt(ATE_RCS_LC,delimiter=' ', dtype=float)
	opt_ATE_DR = np.loadtxt(ATE_DR,delimiter=' ', dtype=float)
	opt_ATE_DR_LC = np.loadtxt(ATE_DR_LC,delimiter=' ', dtype=float)

	
	avg_ATE_odom = np.mean(odom_ATE, axis=0)

	avg_ATE_RCS = np.mean(opt_ATE_RCS, axis=0)
	avg_ATE_RCS_LC = np.mean(opt_ATE_RCS_LC, axis=0)
	avg_ATE_DR = np.mean(opt_ATE_DR, axis=0)
	avg_ATE_DR_LC = np.mean(opt_ATE_DR_LC, axis=0)

	
		
	plt.figure(1)
	
	plt.plot(avg_RPE_odom, 'r', label='Noisy Odometry')	

	plt.plot(avg_RPE_RCS, 'g', label='RCS')
	plt.plot(avg_RPE_RCS_LC, 'tab:cyan', label='RCS-LC')
	plt.plot(avg_RPE_DR, 'tab:purple', label='DR')
	plt.plot(avg_RPE_DR_LC, 'tab:brown', label='DR-LC')


	plt.xlabel('Pose Graph Node')
	plt.ylabel('Average RPE (m)')
	plt.legend()
	plt.title("Relative Position Error (RPE)")
	plt.savefig("ThesisPlots/RPE_Compare_" + c + "_" + n + ".pdf", format="pdf")
	plt.savefig("Data/20min_House_" + c + "_" + n + "/figures/RPE_Compare_" + c + "_" + n + ".pdf", format="pdf")


	plt.figure(2)
	
	plt.plot(avg_ATE_odom, 'r', label='Noisy Odometry')	

	plt.plot(avg_ATE_RCS, 'g', label='RCS')
	plt.plot(avg_ATE_RCS_LC, 'tab:cyan', label='RCS-LC')
	plt.plot(avg_ATE_DR, 'tab:purple', label='DR')
	plt.plot(avg_ATE_DR_LC, 'tab:brown', label='DR-LC')


	plt.xlabel('Pose Graph Node')
	plt.ylabel('Average ATE (m)')
	plt.legend()
	plt.title("Absolute Trajectory Error (ATE)")
	plt.savefig("ThesisPlots/ATE_Compare_" + c + "_" + n + ".pdf", format="pdf")
	plt.savefig("Data/20min_House_" + c + "_" + n + "/figures/ATE_Compare_" + c + "_" + n + ".pdf", format="pdf")
		
	
	#plt.show()

if __name__ == "__main__":
	main()

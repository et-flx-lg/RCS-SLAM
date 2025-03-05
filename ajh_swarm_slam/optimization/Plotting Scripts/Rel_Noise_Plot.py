
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

	c = input('Controller: ')
			
	odom_LN = 'Data/20min_House_' + c + '_LN/odom_RPE.csv'
	odom_MN = 'Data/20min_House_' + c + '_MN/odom_RPE.csv'			
	odom_HN = 'Data/20min_House_' + c + '_HN/odom_RPE.csv'

	opt_LN = 'Data/20min_House_' + c + '_LN/opt_RCS_LC_RPE.csv'
	opt_MN = 'Data/20min_House_' + c + '_MN/opt_RCS_LC_RPE.csv'			
	opt_HN = 'Data/20min_House_' + c + '_HN/opt_RCS_LC_RPE.csv'

	
	odom_LN_RPE = np.loadtxt(odom_LN,delimiter=' ', dtype=float)
	odom_MN_RPE = np.loadtxt(odom_MN,delimiter=' ', dtype=float)
	odom_HN_RPE = np.loadtxt(odom_HN,delimiter=' ', dtype=float)

	opt_LN_RPE = np.loadtxt(opt_LN,delimiter=' ', dtype=float)
	opt_MN_RPE = np.loadtxt(opt_MN,delimiter=' ', dtype=float)
	opt_HN_RPE = np.loadtxt(opt_HN,delimiter=' ', dtype=float)

	
	odom_avg_LN_RPE = np.mean(odom_LN_RPE, axis=0)
	odom_avg_MN_RPE = np.mean(odom_MN_RPE, axis=0)
	odom_avg_HN_RPE = np.mean(odom_HN_RPE, axis=0)

	opt_avg_LN_RPE = np.mean(opt_LN_RPE, axis=0)
	opt_avg_MN_RPE = np.mean(opt_MN_RPE, axis=0)
	opt_avg_HN_RPE = np.mean(opt_HN_RPE, axis=0)

	ATE_odom_LN = 'Data/20min_House_' + c + '_LN/odom_ATE.csv'
	ATE_odom_MN = 'Data/20min_House_' + c + '_MN/odom_ATE.csv'			
	ATE_odom_HN = 'Data/20min_House_' + c + '_HN/odom_ATE.csv'

	ATE_opt_LN = 'Data/20min_House_' + c + '_LN/opt_RCS_LC_ATE.csv'
	ATE_opt_MN = 'Data/20min_House_' + c + '_MN/opt_RCS_LC_ATE.csv'			
	ATE_opt_HN = 'Data/20min_House_' + c + '_HN/opt_RCS_LC_ATE.csv'

	
	odom_LN_ATE = np.loadtxt(ATE_odom_LN,delimiter=' ', dtype=float)
	odom_MN_ATE = np.loadtxt(ATE_odom_MN,delimiter=' ', dtype=float)
	odom_HN_ATE = np.loadtxt(ATE_odom_HN,delimiter=' ', dtype=float)

	opt_LN_ATE = np.loadtxt(ATE_opt_LN,delimiter=' ', dtype=float)
	opt_MN_ATE = np.loadtxt(ATE_opt_MN,delimiter=' ', dtype=float)
	opt_HN_ATE = np.loadtxt(ATE_opt_HN,delimiter=' ', dtype=float)

	
	odom_avg_LN_ATE = np.mean(odom_LN_ATE, axis=0)
	odom_avg_MN_ATE = np.mean(odom_MN_ATE, axis=0)
	odom_avg_HN_ATE = np.mean(odom_HN_ATE, axis=0)

	opt_avg_LN_ATE = np.mean(opt_LN_ATE, axis=0)
	opt_avg_MN_ATE = np.mean(opt_MN_ATE, axis=0)
	opt_avg_HN_ATE = np.mean(opt_HN_ATE, axis=0)

		
	plt.figure(1)
	
	plt.plot(odom_avg_HN_RPE, 'm', label='High Noise')	
	plt.plot(odom_avg_MN_RPE, 'tab:brown', label='Medium Noise')
	plt.plot(odom_avg_LN_RPE, 'tab:orange', label='Low Noise')

	plt.plot(opt_avg_LN_RPE, 'm', linestyle='dashed')
	plt.plot(opt_avg_MN_RPE, 'tab:brown', linestyle='dashed')
	plt.plot(opt_avg_HN_RPE, 'tab:orange', linestyle='dashed')


	plt.xlabel('Pose Graph Node')
	plt.ylabel('Average RPE (m)')
	plt.legend()
	plt.title("Relative Position Error (RPE)")
	plt.savefig("ThesisPlots/Noise_Comp_RPE_" + c + ".pdf", format="pdf")

	plt.figure(2)
	
	plt.plot(odom_avg_HN_ATE, 'm', label='High Noise')
	plt.plot(odom_avg_MN_ATE, 'tab:brown', label='Medium Noise')
	plt.plot(odom_avg_LN_ATE, 'tab:orange', label='Low Noise')

	plt.plot(opt_avg_HN_ATE, 'm', linestyle='dashed')
	plt.plot(opt_avg_MN_ATE, 'tab:brown', linestyle='dashed')
	plt.plot(opt_avg_LN_ATE, 'tab:orange', linestyle='dashed')
	plt.xlabel('Pose Graph Node')
	plt.ylabel('Average ATE (m)')
	plt.legend()
	plt.title("Absolute Pose Graph Error (ATE)")
	plt.savefig("ThesisPlots/Noise_Comp_ATE_" + c + ".pdf", format="pdf")
		
	
	plt.show()

if __name__ == "__main__":
	main()

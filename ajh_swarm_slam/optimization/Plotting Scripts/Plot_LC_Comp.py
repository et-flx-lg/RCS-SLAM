
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
		
	odom = 'Data/20min_House_Joy_MN/odom_ATE.csv'

	opt_no_LC = 'Data/20min_House_Joy_MN/opt_ATE.csv'
	opt_LC = 'Data/20min_House_Joy_LC_MN/opt_ATE.csv'			
	
	Odom = np.loadtxt(odom,delimiter=' ', dtype=float)

	noLC_opt = np.loadtxt(opt_no_LC,delimiter=' ', dtype=float)
	LC_opt = np.loadtxt(opt_LC,delimiter=' ', dtype=float)
	
	avg_odom_opt = np.mean(Odom, axis=0)
	avg_noLC_opt = np.mean(noLC_opt, axis=0)
	avg_LC_opt = np.mean(LC_opt, axis=0)


	
	nodes = 400
	
	plt.figure(1)
	
	plt.plot(avg_odom_opt, 'r', label='Noisy Odometry')	
	plt.plot(avg_noLC_opt, 'g', label='No Loop Closure')
	plt.plot(avg_LC_opt, 'c', label='With Loop Closure')


	plt.xlabel('Trajectory Node')
	plt.ylabel('ATE (m)')
	plt.legend()
	plt.title("ATE at Varying Noise Levels")

	#plt.figure(2)
	
	#plt.plot(odom_avg_LN_ATE, 'r')
	#plt.plot(odom_avg_MN_ATE, 'b')
	#plt.plot(odom_avg_HN_ATE, 'g')

	#plt.plot(opt_avg_LN_ATE, 'r', linestyle='dashed')
	#plt.plot(opt_avg_MN_ATE, 'b', linestyle='dashed')
	#plt.plot(opt_avg_HN_ATE, 'g', linestyle='dashed')

	#plt.title("ATE")
		
	
	plt.show()

if __name__ == "__main__":
	main()

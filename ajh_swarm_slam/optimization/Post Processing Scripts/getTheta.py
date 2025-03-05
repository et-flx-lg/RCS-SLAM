"""
 ----------------------------------------------------------------------------

 * File: getTheta.py
 * Author: Adam Hoburg
 * Date: 02 Feb 2022
 
 * Purpose: Takes quarternion and provides theta orientation in world frame.

 * -------------------------------------------------------------------------- */
"""
import math

def getTheta (Qx, Qy, Qz, Qw):
	Qx = float(Qx)
	Qy = float(Qy)
	Qz = float(Qz)
	Qw = float(Qw)
	t3 = 2*(Qw*Qz + Qx*Qy)
	t4 = 1 - 2 * (Qy*Qy + Qz*Qz)
	
	th = math.atan2(t3,t4)
	if th < 0:
    		th = 2*math.pi + th

				
	return th

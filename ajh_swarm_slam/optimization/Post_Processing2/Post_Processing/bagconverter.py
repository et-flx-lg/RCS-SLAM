#!/usr/bin/env python3
"""
 ----------------------------------------------------------------------------

 * File: bagconverter.py
 * Author: Adam Hoburg
 * Date: 25 March 2024
 
 * Purpose: Used to convert rosbags to .csv files for evaulation and optimization.

 * -------------------------------------------------------------------------- */
"""
import argparse
import csv
import rosbag
import rospy
import numpy as np
import bagpy
from bagpy import bagreader
import pandas as pd


class bc:

  def b2csv(filein,fileout,T):
      
    bag = rosbag.Bag(filein)
        
    data_file = open(fileout,'w')
    data_writer = csv.writer(data_file, delimiter=',')
    
    for topic, msg, t in bag.read_messages(topics=T):
      
      p = msg.data
      
      data_writer.writerow([p])
        
  def b2poses(filein):
    
    bag = bagreader(filein)
    odom = bag.odometry_data()
      
    
    odomdata = np.loadtxt(odom[0],delimiter=',', skiprows=1, dtype=str)
    x = np.transpose(odomdata[:,4])
    y = np.transpose(odomdata[:,5])
        
    x = x.astype(float)
    y = y.astype(float)
      
    x = x-x[0]
    y = y-y[0]

    
    
    return x, y

   

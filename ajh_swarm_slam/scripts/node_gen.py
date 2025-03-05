#!/usr/bin/env python3
"""
 ----------------------------------------------------------------------------

 * File: node_gen.py
 * Author: Adam Hoburg
 * Date: 28 Feb 2024
 
 * Purpose: This code runs as a sub-node to each simulated agent to produce 
 the 2-D pose graph. The code is listening to the output of the noisy odometry
 nodes then placing a node based upon those position estimates at the designated
 time interval which is a parameter in the launch file. The pose graph is generated
 in G2O format for compatibility with GTSAM. Nodes are written to the /central_graph
 topic where they are recorded by the rosbag. This code is also listening for when
 object detection labels are produced by the controller code. The number of nodes
 where objects are detected is then written to the /slam topic.


 * -------------------------------------------------------------------------- */
"""

import rospy
import numpy as np
import math
import time
import re
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState #brauche ich das?, denke
from std_msgs.msg import Float32MultiArray
from scipy.spatial.transform import Rotation as R
import random as r


class NodeGen:  
 
  def __init__(self, event=None):
    
    self.x = 0
    self.y = 0
    self.theta = 0
       
    self.pub_central = rospy.Publisher("/central_graph", String, queue_size=5)
    self.slam = rospy.Publisher("/slam", String, queue_size=5)
    robotname = rospy.get_namespace()
    self.noiseX = rospy.get_param(robotname+"node_gen/lin_noise")
    self.noiseY = rospy.get_param(robotname+"node_gen/lin_noise")
    self.noiseTh = rospy.get_param(robotname+"node_gen/ang_noise")
    
    
    my_num = re.findall(r'\d+', robotname)
    team = rospy.get_param(robotname+"node_gen/team_size");
    IPs = '/home/robolab/catkin_ws/src/ajh_swarm_slam/scripts/Initial_Pos/init_pose_'+str(int(team))+'.csv'
    init = np.loadtxt(IPs,delimiter=' ', dtype=str)
    
    r = int(my_num[0])-1
    
    self.prev_x = float(init[r,0])
    self.prev_y = float(init[r,1])
    self.prev_theta = 0
    
    self.object_detect = False
    
    self.node_id = 1000*(int(my_num[0]))


  def getodom(self, msg, event=None):
    self.x = msg.pose.pose.position.x
    self.y = msg.pose.pose.position.y
    
    Qx = msg.pose.pose.orientation.x
    Qy = msg.pose.pose.orientation.y
    Qz = msg.pose.pose.orientation.z
    Qw = msg.pose.pose.orientation.w
    
    # Convert quaternion to z in radian
    
    t3 = 2*(Qw*Qz + Qx*Qy)
    t4 = 1 - 2 * (Qy*Qy + Qz*Qz)
    
    self.theta = math.atan2(t3,t4)
    if self.theta < 0:
    	self.theta = 2*math.pi + self.theta 
      
  def objectdetect(self, msg):
  	self.object_detect = True
  	self.ray = msg.data
         
  def placenode(self, event=None):
  
    # The subscripts for P and R just correspond to 1 = previous, 2 = current node. Not specific node numbers.
    
    R1 = np.array([[math.cos(self.prev_theta), -math.sin(self.prev_theta)],[math.sin(self.prev_theta),math.cos(self.prev_theta)]])
    R2 = np.array([[math.cos(self.theta), -math.sin(self.theta)],[math.sin(self.theta),math.cos(self.theta)]])
    P1 = np.array([[self.prev_x],[self.prev_y]])
    P2 = np.array([[self.x],[self.y]])
    
    edgeR = np.matmul(np.transpose(R1),R2)
    edgeTh = math.atan2(edgeR[1,0],edgeR[0,0])
    edgeP = np.matmul(np.transpose(R1), (P2-P1))
    
    #if edgeTh < 0.0:
    #	edgeTh = edgeTh + 2*math.pi
 
    vertex = "VERTEX_SE2 " + str(self.node_id) + " " + str(self.x) + " " + str(self.y) + " " + str(self.theta)
    # G2O format is expected inverse covariance matrix. Argument values are standard deviations.
    edge = "EDGE_SE2 " + str(self.node_id - 1) + " " + str(self.node_id) + " " + str(edgeP[0,0]) + " " + str(edgeP[1,0]) + " "+ str(edgeTh) + " " + str(1/(self.noiseX**2)) + " " + str(0) + " " + str(0) + " " + str(1/(self.noiseY**2)) + " " + str(0) +  " " + str(1/(self.noiseTh**2))
    
    

    self.pub_central.publish(vertex)
    time.sleep(.02)
    self.pub_central.publish(edge)
    time.sleep(.02)
    
    if self.object_detect:
    	slam_pub = str(self.node_id)+' ' + self.ray
    	self.slam.publish(slam_pub)
    	self.object_detect = False
  
    self.node_id = self.node_id + 1
    self.prev_x = self.x
    self.prev_y = self.y
    self.prev_theta = self.theta
    
  def initial(self):

    
    vertex = "VERTEX_SE2 " + str(self.node_id) + " " + str(self.prev_x) + " " + str(self.prev_y) + " " + str(0)
    
           
    self.pub_central.publish(vertex)
    
    self.node_id = self.node_id+1

def run(node_interval,n):
	rospy.Subscriber('noisy_odom',Odometry,n.getodom)
	rospy.Subscriber('object_detect',String, n.objectdetect)
	rospy.Timer(rospy.Duration(node_interval), n.placenode)
	rospy.spin()


if __name__ == '__main__':
  
  rospy.init_node('node_gen', anonymous=False)
  n = NodeGen()
  robotname = rospy.get_name()
  node_interval = rospy.get_param(robotname+"/nodetime", 10)
  time.sleep(1)
  n.initial()
  run(node_interval,n)
    

  
  


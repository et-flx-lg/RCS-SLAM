#!/usr/bin/env python3
"""
 ----------------------------------------------------------------------------

 * File: swarm_graph.py
 * Author: Adam Hoburg
 * Date: 04 March 2024
 
 * Purpose: This code runs as its own node in the launch file to produce 1) the 
 communication graph detailing which robots are in communication with one another
 at a given node and 2) the robot detection graph which determines which robots
 are within the 'visual' range of each others' robot detection sensor. The 
 communication graph is published to the /commgraph topic and essentially just
 recorded by the rosbag for offline processing. The robot detection graph is 
 published to /robotfinder and each robot listens to this topic to check when 
 another agent is in front of it. The robots use this information for labeling
 nodes where obstructions are detected that aren't robots. The node interval
 for the communication graph is a paramter that should be declared in the 
 launch file.


 * -------------------------------------------------------------------------- */
"""

import rospy
import numpy as np
import math
import time
from std_msgs.msg import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension



class graph:  # Simulations a communication device capable of effective comms distance and query of pertinent information
 
	def __init__(self, event=None):
       
    
		#self.comm_range = 2
		self.comm_range = rospy.get_param('comm_graph/com_dist');
		self.team_size = int(rospy.get_param('comm_graph/team_size', 4))
		self.robot_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		self.det_range = float(rospy.get_param('comm_graph/det_range'))
    
		self.comms = np.zeros((self.team_size,(self.team_size)))
		self.detector = np.zeros((self.team_size,self.team_size))
    
		self.pub_graph = rospy.Publisher("/commgraph",Float32MultiArray,queue_size=1)
		self.find_pub = rospy.Publisher("/robotfinder", Float32MultiArray, queue_size=1)
		rospy.init_node('comm_graph', anonymous=False)
    
		self.node_id = 0
		
		self.det = Float32MultiArray()
		self.mat = Float32MultiArray()


	def builder(self, event=None):
    
		for i in range(0,(self.team_size)):
      
			agent = 'Robot'+str(i+1)
			rospy.wait_for_service('/gazebo/get_model_state')
			my_state = self.robot_state(agent, "world")
			my_x = my_state.pose.position.x
			my_y = my_state.pose.position.y
			
			my_pose = np.array([[my_x],[my_y]])
      
			Qx = my_state.pose.orientation.x
			Qy = my_state.pose.orientation.y
			Qz = my_state.pose.orientation.z
			Qw = my_state.pose.orientation.w
	
			# Convert quaternion to z in radian
	
			t3 = 2*(Qw*Qz + Qx*Qy)
			t4 = 1 - 2 * (Qy*Qy + Qz*Qz)
	
			my_th = math.atan2(t3,t4)
			if my_th < 0:
				my_th = my_th + 2*math.pi

			for j in range(0,(self.team_size)):
				if i != j:
					self.contact = 'Robot'+str(j+1)
					rospy.wait_for_service('/gazebo/get_model_state')
					contact_state = self.robot_state(self.contact,"world")
          
					c_x = contact_state.pose.position.x
					c_y = contact_state.pose.position.y
					
					c_pose = np.array([[c_x],[c_y]])
					
					Qx = contact_state.pose.orientation.x
					Qy = contact_state.pose.orientation.y
					Qz = contact_state.pose.orientation.z
					Qw = contact_state.pose.orientation.w
					
					# Convert quaternion to z in radian
					t3 = 2*(Qw*Qz + Qx*Qy)
					t4 = 1 - 2 * (Qy*Qy + Qz*Qz)
					
					c_th = math.atan2(t3,t4)
					
										
					if c_th < 0:
						c_th = 2*math.pi + c_th
					diff = np.subtract(c_pose, my_pose)
					measurement = np.linalg.norm(diff)
					
					R1 = np.array([[math.cos(my_th), -math.sin(my_th)],[math.sin(my_th),math.cos(my_th)]])
					R2 = np.array([[math.cos(c_th), -math.sin(c_th)],[math.sin(c_th),math.cos(c_th)]])
					
					P1 = my_pose
					P2 = c_pose
					
					edgeP = np.matmul(np.transpose(R1), (P2-P1))
					
					rel_bearing = abs(math.atan2(edgeP[1,0], edgeP[0,0]))
					          					         
          
					if measurement <= self.comm_range:
						ns_meas = round(measurement,3) + np.random.normal(0,0.02)
						self.comms[i,j] = ns_meas

					else:
						self.comms[i,j] = 0

					if (measurement <= self.det_range):
					
						if rel_bearing <= (0.872): # +/- 50 degrees
							self.detector[i,j] = 1.0
						
						else:
							self.detector[i,j] = 0.0 
				
					else:
						self.detector[i,j] = 0.0

	
		
		self.det.data = self.detector.reshape([int(math.pow(self.team_size,2))])		
		
		self.find_pub.publish(self.det)

  
	def comm(self, event=None):

		self.mat.data = self.comms.reshape([int(math.pow(self.team_size,2))])
		self.pub_graph.publish(self.mat)
      
		self.node_id = self.node_id + 1

def run(g, node_interval):

	#rospy.Subscriber('/outcomm',Float32MultiArray,g.router)
	
	rospy.Timer(rospy.Duration(0.1), g.builder)
	rospy.Timer(rospy.Duration(node_interval), g.comm)
	
	rospy.spin()
	
if __name__ == '__main__':
  
	#time.sleep(2)
	rospy.init_node('comm_graph', anonymous=False)
	g = graph()
	g.comm()
	node_interval = rospy.get_param('comm_graph/nodetime', 10)
  
	run(g, node_interval)

  
  


#!/usr/bin/env python3

# Swarm_controller3 limits the range of the lidar to +/- 15 degrees and leverages all eight rays for the purpose of robot detection.

import rospy
from std_msgs.msg import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
import re
import math
import numpy as np



CMD_PUB = rospy.Publisher("cmd_vel", Twist, queue_size=1)
SLAM_PUB = rospy.Publisher("object_detect", String, queue_size=1)
TEST = rospy.Publisher("TEST", String, queue_size=1)

NODENAME = rospy.get_name()
MY_NUM = re.findall(r'\d+', NODENAME)
MY_NUM = int(MY_NUM[0])

ROB_RANGE = 1.5 # Max range for robot detection

MIN_RANGE = 1.0 # Collision avoidance boundary
STOP = 0.5



class controller:
	def __init__(self):
		self.robot_detect = False
		self.team_size = int(rospy.get_param('motion/team_size'))
		self.lead_bot = rospy.get_param('motion/leader')
		self.vel = float(rospy.get_param("motion/linvel"))
		
		self.leader_name = '/Robot' + str(int(self.lead_bot)) + '/odom'
		
		self.graph = np.zeros((self.team_size,self.team_size))
		self.me = MY_NUM - 1
		
		self.my_pos = np.zeros((2,1))
		self.my_th = 0.0
		self.leader_pos = np.zeros((2,1))
		self.l_th = 0.0
		
		self.rel_bearing = 0.0
		self.rel_range = 0.0 
		
		
		self.slam = "True"
		self.omega = 0.0
		
		self.slam_delay = 15 # Based on rostopic hz analysis, scan updates at 8.26hz and robotfinder is 0.65 hz => 12.7 times faster, 15 is a buffer
		self.slam_count = 0
		self.stop_count = 0
		self.stop_limit = 40
				
		
		
	def update(self, msg):
		msg = np.array(msg.data)
		i = 0

		for k in range(0,self.team_size):
			for m in range(0,self.team_size):
				self.graph[k,m] = msg[i]
				i += 1
		
		for j in range(0,self.team_size):
			if (int(self.graph[self.me, j]) != 0):
				self.robot_detect = True
				break
			else:
				self.robot_detect = False
	

	def scan(self, msg):
	
		samples = len(msg.ranges)
		
		min_contact = 10.0
		min_slam = 10.0
	
		for i in range(samples):
			
			if (i>2) and (i<10): # Only want SLAM detection to be concerned with +/- 45 degrees
				if min_slam > msg.ranges[i]:
					min_slam = msg.ranges[i]
					ray = i
					
			if min_contact > msg.ranges[i]:
				min_contact = msg.ranges[i]
				ray2 = i
				

		if min_slam <= 1.5 and self.robot_detect == False:
			self.slam_count += 1
			if self.slam_count >= self.slam_delay :
				self.slam = str(ray)
				SLAM_PUB.publish(self.slam)
							
		# Greater than collision barrier, less than detection range.
		if min_contact <= STOP:
			self.stop_count += 1
			if self.stop_count <= self.stop_limit:
				if ray2 < 6:
					Omega = 0.25
					linvel = 0.0
				else:
					Omega = -0.25
					linvel = 0.0
			
			else:
				Omega = 0.3
				linvel = 0.0
							
			
			
			
		elif min_contact <= MIN_RANGE:
			self.slam_count = 0
			self.stop_count = 0
			
			if ray2 < 6:
			
				Omega = 0.0
				linvel = self.vel*0.8
				
			else:
				linvel = self.vel*0.8
				Omega = 0.0
					
			
				

		else:
			Omega = 0.0
			linvel = self.vel
			self.slam_count = 0
			self.stop_count = 0
			
		
		cmdmsg = Twist()
		cmdmsg.linear.x = linvel
		cmdmsg.angular.z = Omega
	
		CMD_PUB.publish(cmdmsg)
    
	def run(self):
		rospy.Subscriber("scan", LaserScan, self.scan, queue_size=1)
		rospy.Subscriber("/robotfinder", Float32MultiArray, self.update, queue_size=1)

		rospy.spin()

if __name__ == '__main__':
    
	rospy.init_node('motion', anonymous=False)
    	
	c = controller()
	c.run()





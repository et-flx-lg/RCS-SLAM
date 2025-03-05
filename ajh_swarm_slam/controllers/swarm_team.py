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
#from robot_finder import robot_finder as rf
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
import re
import math
import numpy as np
import time



CMD_PUB = rospy.Publisher("cmd_vel", Twist, queue_size=1)
SLAM_PUB = rospy.Publisher("object_detect", String, queue_size=1)

NODENAME = rospy.get_name()
MY_NUM = re.findall(r'\d+', NODENAME)
MY_NUM = int(MY_NUM[0])

ROB_RANGE = 1.5 # Max range for robot detection

MIN_RANGE = 0.7 # Collision avoidance boundary

STOP = 0.4
OMEGA_CHANGE = 0.25
LINVEL_CHANGE = 0.15



class controller:
	def __init__(self):
		self.robot_detect = False
		self.team_size = int(rospy.get_param('motion/team_size'))
		self.vel = float(rospy.get_param("motion/linvel"))
		self.lead_vel = self.vel + 0.1
		self.graph = np.zeros((self.team_size,self.team_size))
		self.me = MY_NUM - 1

		self.lead_bot = int(rospy.get_param('motion/leader'))
		
		self.leader_name = '/Robot' + str(self.lead_bot) + '/odom'
		

		self.slam = " "
		self.slam_delay = 15 # On fifth detection of an object before it that isn't an robot, publish to slam node.
		self.slam_count = 0
		
		self.stop_counter = 0
		self.stop_limit = 40	

		self.omega = 0.0
		self.Joy_omega = 0.0
		self.Joy_linvel = 0.0

		self.my_pos = np.zeros((2,1))
		self.my_th = 0.0
		self.leader_pos = np.zeros((2,1))
		self.l_th = 0.0
		
		self.rel_bearing = 0.0
		self.rel_range = 0.0 		
		
		self.cmdmsg = Twist()
		self.wait_time = 0.0
		
	def role(self):
		if self.lead_bot == MY_NUM:
			self.role = "Leader"
		else: 
			self.role = "Follower"
		
		return self.role

	def leader(self, msg):
		self.leader_pos[0,0] = msg.pose.pose.position.x
		self.leader_pos[1,0] = msg.pose.pose.position.y
	
		Qx = msg.pose.pose.orientation.x
		Qy = msg.pose.pose.orientation.y
		Qz = msg.pose.pose.orientation.z
		Qw = msg.pose.pose.orientation.w
		
		# Convert quaternion to z in radian
		t3 = 2*(Qw*Qz + Qx*Qy)
		t4 = 1 - 2 * (Qy*Qy + Qz*Qz)
		
		self.l_th = math.atan2(t3,t4)
		
		if self.l_th < 0:
			self.l_th = 2*math.pi + self.l_th 
	
	def follower(self, msg):
		self.my_pos[0,0] = msg.pose.pose.position.x
		self.my_pos[1,0] = msg.pose.pose.position.y
			
		Qx = msg.pose.pose.orientation.x
		Qy = msg.pose.pose.orientation.y
		Qz = msg.pose.pose.orientation.z
		Qw = msg.pose.pose.orientation.w
		
		# Convert quaternion to z in radian
		t3 = 2*(Qw*Qz + Qx*Qy)
		t4 = 1 - 2 * (Qy*Qy + Qz*Qz)
		
		self.my_th = math.atan2(t3,t4)
		
		if self.my_th < 0:
			self.my_th = 2*math.pi + self.my_th
			
		diff = np.subtract(self.leader_pos, self.my_pos)
		self.rel_range = np.linalg.norm(diff)
		
		R1 = np.array([[math.cos(self.my_th), -math.sin(self.my_th)],[math.sin(self.my_th),math.cos(self.my_th)]])
		R2 = np.array([[math.cos(self.l_th), -math.sin(self.l_th)],[math.sin(self.l_th),math.cos(self.l_th)]])
		P1 = self.my_pos
		P2 = self.leader_pos
		
		edgeP = np.matmul(np.transpose(R1), (P2-P1))
		self.rel_bearing = math.atan2(edgeP[1,0], edgeP[0,0])
		
			
				
		
	def update(self, msg):
		#msg = np.fromstring(msg.data, dtype=float, sep=' ')
		msg = np.array(msg.data)
		i = 0

		for k in range(0,self.team_size):
			for m in range(0,self.team_size):
				self.graph[k,m] = msg[i]
				#self.graph[m,k] = msg[i]
				i += 1
		
		for j in range(0,self.team_size):
			if (int(self.graph[self.me, j]) != 0.0):
				self.robot_detect = True
				break
			else:
				self.robot_detect = False
	
	def Joy(self, msg):
		
		joy = msg.axes
		
		if joy[0]!=0 and joy[3]==0:
			self.Joy_omega = 0.25*joy[0]
		elif joy[0]==0 and joy[3]!=0:
			self.Joy_omega = 0.25*joy[3]
		else:
			self.Joy_omega = 0
										
		if joy[1]!=0 and joy[4]==0:
			self.Joy_linvel = 0.2*joy[1]
		elif joy[1]==0 and joy[4]!=0:
			self.Joy_linvel = 0.2*joy[4]
		else:
			self.Joy_linvel = 0


	def lead_role(self, msg):
	
		samples = len(msg.ranges)
		
		min_contact = 10.0
		min_front = 10.0
			
		for i in range(samples):
			
			if (i>2) and (i<10): # Only want SLAM detection to be concerned with +/- 45 degrees
				if min_front > msg.ranges[i]:
					min_front = msg.ranges[i]
					ray = i
					
			if min_contact > msg.ranges[i]:
				min_contact = msg.ranges[i]
				ray2 = i
		
		if min_front <= 1.5 and self.robot_detect == False:
			self.slam_count += 1
			if self.slam_count >= self.slam_delay :
				self.slam = str(ray)
				SLAM_PUB.publish(self.slam)
				

			
		if min_contact <= STOP:
			self.stop_counter += 1
			if self.stop_counter <= self.stop_limit:
				if ray2 < 6:
					Omega = 0.25 + self.Joy_omega
					linvel = 0.0 + self.Joy_linvel
				else:
					Omega = -0.25 + self.Joy_omega
					linvel = 0.0 + self.Joy_linvel
			else:
				Omega = 0.3
				linvel = 0.0
					

		elif min_contact <= 1.2:
			self.stop_counter = 0		
			
			if ray2 < 6:
					
				Omega = 0.25 + self.Joy_omega
				linvel = self.lead_vel + self.Joy_linvel
					
			else:
				linvel = self.lead_vel + self.Joy_linvel
				Omega = -0.25 + self.Joy_omega

			
		else:
			Omega = 0.0 + self.Joy_omega
			linvel = self.lead_vel + self.Joy_linvel
			self.stop_counter = 0

									
		self.cmdmsg.linear.x = linvel
		self.cmdmsg.angular.z = Omega
		CMD_PUB.publish(self.cmdmsg)
		
	def follow_role(self, msg):
		
		samples = len(msg.ranges)
		
		min_contact = 10.0
		min_front = 10.0
			
		for i in range(samples):
			
			if (i>2) and (i<10): # Only want SLAM detection to be concerned with +/- 45 degrees
				if min_front > msg.ranges[i]:
					min_front = msg.ranges[i]
					ray = i
					
			if min_contact > msg.ranges[i]:
				min_contact = msg.ranges[i]
				ray2 = i
		
		if min_front <= 1.5 and self.robot_detect == False:
			self.slam_count += 1
			if self.slam_count >= self.slam_delay :
				self.slam = str(ray)
				SLAM_PUB.publish(self.slam)
				
		if (self.rel_range < 0.5):
			linvel = -0.1
			Omega = 0.0

		elif min_contact <= STOP:
			self.stop_counter += 1
			if self.stop_counter <= self.stop_limit:
				if ray2 < 6:
					Omega = 0.25 + self.Joy_omega
					linvel = 0.0 + self.Joy_linvel
				else:
					Omega = -0.25 + self.Joy_omega
					linvel = 0.0 + self.Joy_linvel
			else:
				Omega = 0.3
				linvel = 0.0

			
		elif (self.rel_range < 0.7):
			linvel = 0.0
			Omega = 0.0
					

		elif min_contact <= MIN_RANGE:
			self.stop_counter = 0		
			
			if ray2 < 6:
					
				Omega = 0.15
				linvel = self.vel*0.8
					
			else:
				#self.stop_counter = 0	
				linvel = self.vel*0.8
				Omega = -0.15
					


		elif (self.rel_range > 2): # was 2.25m
			self.stop_count = 0
			if (self.rel_bearing < 0):
				Omega = self.omega - OMEGA_CHANGE
				
				if (self.rel_bearing < -1.57):
					linvel = self.vel - 0.05
				else:
					linvel = self.vel + LINVEL_CHANGE
	

			else:
				Omega = self.omega + OMEGA_CHANGE
				
				if (self.rel_bearing > 1.57):
					linvel = self.vel - LINVEL_CHANGE
				else:
					linvel = self.vel + LINVEL_CHANGE
						
			
		else:
			Omega = 0.0
			linvel = self.vel
			self.slam_count = 0
			self.stop_counter = 0

		
		self.cmdmsg.linear.x = linvel
		self.cmdmsg.angular.z = Omega
		CMD_PUB.publish(self.cmdmsg)

			
    
	def run_leader(self):
		rospy.Subscriber("scan", LaserScan, self.lead_role, queue_size=1)
		rospy.Subscriber("/robotfinder", Float32MultiArray, c.update, queue_size=1)
		rospy.Subscriber("/joy", Joy, c.Joy, queue_size=1)

		rospy.spin()
		
	def run_follower(self):
		rospy.Subscriber("scan", LaserScan, self.follow_role, queue_size=1)
		rospy.Subscriber("/robotfinder", Float32MultiArray, c.update, queue_size=1)
		rospy.Subscriber(self.leader_name, Odometry, self.leader, queue_size=1)
		rospy.Subscriber('odom', Odometry, self.follower, queue_size=1)

		rospy.spin()


if __name__ == '__main__':
    
	rospy.init_node('motion', anonymous=False)
    
	
	c = controller()
	role = c.role()
	
	if role == "Leader":
		c.run_leader()
	else:
		c.run_follower()





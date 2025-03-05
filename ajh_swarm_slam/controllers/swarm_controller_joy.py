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

MIN_RANGE = 0.6 # Collision avoidance boundary
STOP = 0.4

OMEGA_CHANGE = 0.25
LINVEL_CHANGE = 0.1



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
		
		self.left_count = 0
		self.right_count = 0
		
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
				
		test = str(self.rel_bearing) + ' ' + str(self.rel_range)
		TEST.publish(test)
		
		
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
			if (int(self.graph[self.me, j]) != 0):
				self.robot_detect = True
				break
			else:
				self.robot_detect = False
	
	def Joy(self, msg):
		joy = msg.axes
		
		if joy[0]!=0 and joy[3]==0:
			self.omega = 0.25*joy[0]
		elif joy[0]==0 and joy[3]!=0:
			self.omega = 0.25*joy[3]
		else:
			self.omega = 0
								


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
				

		elif (self.rel_range > 2.25): # 2.25m
			self.stop_count = 0
			if (self.rel_bearing < 0):
				Omega = self.omega - OMEGA_CHANGE
				self.left_count += 1
				if (self.rel_bearing < -1.57):
					#linvel = self.vel - LINVEL_CHANGE
					linvel = self.vel - 0.05
				else:
					linvel = self.vel + LINVEL_CHANGE
	

			else:
				Omega = self.omega + OMEGA_CHANGE
				self.right_count += 1
				if (self.rel_bearing > 1.57):
					#linvel = self.vel - LINVEL_CHANGE
					linvel = self.vel - 0.05
				else:
					linvel = self.vel + LINVEL_CHANGE
				
			
			
			
		elif min_contact <= MIN_RANGE:
			self.stop_count = 0
		
			if ray2 < 6:
					
				Omega = 0.15 + self.omega
				#linvel = self.vel*0.8
				linvel = self.vel*0.5
					
			else:
				#linvel = self.vel*0.8
				linvel = self.vel*0.5
				Omega = -0.15 + self.omega
				#self.slam_count = 0
					
			
				

		else:
			Omega = self.omega
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
		rospy.Subscriber("/joy", Joy, self.Joy, queue_size=1)
		rospy.Subscriber(self.leader_name, Odometry, self.leader, queue_size=1)
		rospy.Subscriber('odom', Odometry, self.follower, queue_size=1)

		rospy.spin()

if __name__ == '__main__':
    
	rospy.init_node('g2o', anonymous=False)
    	
	c = controller()
	c.run()





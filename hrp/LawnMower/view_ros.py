# -*- coding: utf-8 -*-
"""
Created on Tue Mar 29 09:38:30 2016

@author: ozer
"""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from am_driver.msg import SensorStatus
from sensor_msgs.msg import Range

class RosView():
	def __init__(self,model,namespace):
		self.model = model
		self.namespace = namespace

		#init publisher for ROS messages
		try:
			rospy.init_node('movement_node', anonymous=True)
		except:
			pass
			
		self.publisher = rospy.Publisher(namespace + '/cmd_vel', Twist, queue_size=10)
		rospy.Subscriber(namespace + '/odom', Odometry, self.update_position)
		rospy.Subscriber(namespace + '/sensor_status', SensorStatus, self.update_collision)
		rospy.Subscriber(namespace+'/range', Range, self.update_range)

	def update(self):
		self.model.tick()
		#publish new velocities to gazebo model
		self.publish()

	def update_range(self,data):
		r = data.range
		self.model.update_range(r)
		pass

	def update_collision(self,data):
		# sensorStatus = 0x40 --> in charging station
		# sensorStatus = 0x04 --> collision
		# sensorStatus = 0x02 --> out of area

		if data.sensorStatus == 0x40:
			bump = 'charge'
		elif data.sensorStatus == 0x04:
			bump = 'bump'
		elif data.sensorStatus == 0x02:
			bump = 'area'
		else:
			bump = 'none'

		#pass the data to model
		self.model.update_bump(bump)

	def update_position(self,data):
		#pass the data to model
		odom_pos = data.pose.pose.position
		odom_ori = data.pose.pose.orientation
		self.model.update_pos([odom_pos.x,odom_pos.y,odom_pos.z],[odom_ori.x,odom_ori.y,odom_ori.z,odom_ori.w])

	def publish(self):
		move_command = Twist()

		# linear velocity
		move_command.linear.x = self.model.v
		# rotational velocity
		move_command.angular.z = self.model.omega
		self.publisher.publish(move_command)

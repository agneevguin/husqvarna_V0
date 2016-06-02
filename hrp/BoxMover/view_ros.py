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
from gazebo_msgs.msg import ModelStates

class RosView:
	def __init__(self,model, rand=0):
		self.model = model
		self.sent = False
		self.model_data=None
		#init publisher for ROS messages
		try:
			rospy.init_node('movement_node', anonymous=True)
		except:
			pass
		self.name = 'target-'+rand+'-0'
		self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		rospy.Subscriber('/odom', Odometry, self.update_position)
		rospy.Subscriber('/sensor_status', SensorStatus, self.update_collision)
		rospy.Subscriber('/range', Range, self.update_range)
		rospy.Subscriber('/gazebo/model_states', ModelStates, self.read_model)

	def update(self):
		self.model.tick()
		#publish new velocities to gazebo model
		self.publish()

	def read_model(self, data):
		self.model.update_box_pos(data.pose[3].position)
		
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
		#pose = PoseStamped()
		#pose.pose.position.x = 4.
    	#pose.pose.position.y = 4.
		#pose.pose.position.z = 0.
		#pose.header.frame_id = self.namespace + "base_link"
    	#pose.header.stamp = rospy.Time.now()
		#self.goal_publisher.publish(pose)
		# linear velocity
		move_command.linear.x = self.model.v
		# rotational velocity
		move_command.angular.z = self.model.omega
		self.publisher.publish(move_command)


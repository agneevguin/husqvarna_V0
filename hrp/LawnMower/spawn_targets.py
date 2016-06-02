import rospy
import numpy as np
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel

import random

class Spawner():
	def __init__(self, positions, size, z):
		self.targets = positions
		self.z = z
		self.random = random.randint(0,100) #just give a new name everytime the main is called. so you dont have to restart gazebo
		try:
			rospy.init_node('insert_targets', anonymous=True)
		except:
			pass

		self.service = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

		# open sdf file and set desired size
		file = open('box.sdf', 'r')
		sdf = file.read()
		sdf = sdf.replace('{size_x}', str(size))
		sdf = sdf.replace('{size_y}', str(size))
		sdf = sdf.replace('{size_z}', str(size))
		self.sdf = sdf

	def create_targets(self):
		initalPose = Pose()

		for i in range(0, len(self.targets)):
			target = {}
			target['model_name'] = 'target' + '-' + str(self.random) + '-' + str(i)

			initalPose.position.x = self.targets[i][0]
			initalPose.position.y = self.targets[i][1]
			initalPose.position.z = self.z
			target['position'] = initalPose

			self.spawn_object(target)


	def spawn_object(self, target):
		response = self.service(target['model_name'], self.sdf, 'target', target['position'], 'world')


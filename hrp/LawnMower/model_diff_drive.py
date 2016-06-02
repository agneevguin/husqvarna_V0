# -*- coding: utf-8 -*-
"""
Created on Sat Mar 12 15:18:45 2016
@author: ozer
"""

import numpy as np
import motion_model as motion
STATUS_TRACKING='Tracking target'
STATUS_WEIR='Weir sensed'
STATUS_VICTIM='Victim found'
STATUS_COLLUSION='Robot seen'
RANGE = 3.0
class LawnMower():

	def __init__(self, X=[0.,0.,0.],V=[0.,0.,0.],R=[0.,0.,0.,0.],target=[0.,0.,0.], max_v = 100., max_omega = np.pi/4, name = 'noname', bm = None):
		#in case you want to give your robot some cute name. Also useful for debugging.
		self.name = name
		#the handle of the benchmarker this model is subject to.
		self.bm = bm
		#position of the model in 3D
		self.X = np.array(X)
		#velocity of the model, the rotation of the vector is the current
		#heading of the model and the length of the vector is its linear velocity
		self.V = np.array(V)
		#orientation of the model in quaternions
		self.R = np.array(R)
		#maximum linear velocity
		self.max_v = max_v
		#maximum angular velocity
		self.max_omega = max_omega

		#current time in simulation ticks
		self.time = 0
		#the time length in seconds each 'tick' in simulation is
		self.dt = 0.03

		#desired angular velocity
		self.omega = 0.
		#desired linear velocity
		self.v = 0.

		#the target this model is trying to reach
		#initially the target is its own position
		self.target= target 	#self.target = np.array([-30.,30.,0.])

		#list of targets this model will try to follow
		self.target_list=[target] #[[8.,-4.,0.],[3.,4.,0.],[0.,0.,0.],[2.,2.,0.],[-3.,-3.,0.]]

		# will be set by view. possibilities are 'charge' 'bump' and 'none'
		self.bump = 'none'
		
		# will be set by the view, contains the range sensor output
		self.range = -1

		#status of the robot. "run" or "stop". This will be set from outside the model.
		self.status = 'run'

		self.prev_error = 0
		self.prev_error_theta = 0
		self.index = 0
		self.temp_index = 0
		
	def tick(self):
		if self.status == 'run':
			self.printStatus()
			if self.bump == 'area':
				self.v = 0.
				self.omega = 0.
			elif self.bump == 'bump':
				self.v = 0.
				self.omega = 0.
			elif self.range < RANGE:
				self.v = 0.
				self.omega = 0.
			else:
				self.temp_index = self.index
				self.v, self.omega, self.index = motion.updateMotion(self.X, self.R, self.target, self.index)
				if self.index < len(self.target_list) and (self.index != self.temp_index):
				    self.target = self.target_list[self.index]
		else:
			self.omega = 0.
			self.v = 0.
		pass

	def update_pos(self,pos,ori):
		# TODO make use of the data in tick
		oldX = self.X
		self.X = np.array(pos)
		self.V = self.X - oldX
		self.R = np.array(ori)

	def update_bump(self,bump):
		# TODO make use of the data in tick
		self.bump = bump

	def update_range(self,r):
		# TODO make use of the data in tick
		self.range = r

	def setTarget(self,T):
		self.target = np.array(T)

	def addTarget(self,T,multi): #multi=False .insert
		if multi:
			for t in T:
				self.target_list.append(np.array(t))
		else:
			self.target_list.append(np.array(T))

	def printStatus(self):
		print '='*10
		print self.name
		print 'X',self.X
		print 'R',self.R
		print 'V',self.V
		print 'bump',self.bump
		print 'range',self.range
		print 'status',self.status
		print 'target',self.target
		print 'index',self.index
		print 'temp_index',self.temp_index
		print '='*10

	def confirmVictim(self):
		for r in self.bm.robots:
			if self.name != r.name and r.X == self.orientation*1.5:
				return
		#update the benchmarker, it already has the model handle.
		#victim pos is optional, set it to [x,y] when you think you found a victim
		self.bm.update(self.name, victim_pos = None)
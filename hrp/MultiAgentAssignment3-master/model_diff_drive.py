# -*- coding: utf-8 -*-

import numpy as np
import math
import motion_model as motion
import visibility_graph as vis
import visibility_graph_1 as vis1
import triangulation as tri

C1=0.0  #controlled zones go-to-target
D1=0.0  #dead zones go-to-target
C2=1.2  #controlled zones swirl-obstacles
D2=0.75 #dead zones swirl-obstacles
LAMBDA=0.223 #the absolute distance from that calculated point to the target
D3=0.8 #dead zones push
C4=1.5 #controlled zones acquire
D4=0.7 #dead zones acquire
THETA_MAX=math.radians(68) #the angular width of the controlled zone


class LawnMower:
	def __init__(self, X=[0., 0., 0.], V=[0., 0., 0.], R=[0., 0., 0., 0.], name='noname', box_locs=[[0.,0.,0.]],goal=[]):
		# in case you want to give your robot some cute name. Also useful for debugging.
		self.name = name
		# position of the model in 3D
		self.X = np.array(X)
		# velocity of the model, the rotation of the vector is the current
		# heading of the model and the length of the vector is its linear velocity
		self.V = np.array(V)
		# orientation of the model in quaternions
		self.R = np.array(R)
		#location of the box
		self.box_locations = box_locs
		self.box_pos = np.array(box_locs[0])
		self.goal=np.array(goal)
		# current time in simulation ticks
		self.time = 0
		# the time length in seconds each 'tick' in simulation is
		self.dt = 0.03
		# desired angular velocity
		self.omega = 0.
		# desired linear velocity
		self.v = 0.
		self.target=[]
		self.target_list=[]
		# will be set by view. possibilities are 'charge' 'bump' and 'none'
		self.bump = 'none'
		
		# will be set by the view, contains the range sensor output
		self.range = -1
		
		# status of the robot. "run" or "stop". This will be set from outside the model.
		self.status = 'run'

		
	def tick(self):
		#vis.visibility_path(self.X, self.goal)
		path = vis.visibility_path([-15.,10.], [15.,-15.])
		#points1 = [(14.2565389, 48.2248439, 1000), (14.2637736, 48.2331576, 55), (14.2488966, 48.232513, 55)]
		#print tri.triangulate(points1)  #works
		#vis1.visibility_path()  #trial
	
		
		if self.status == 'run':
			self.move()
		else:
			self.omega = 0.
			self.v = 0.
		pass

	def update_pos(self, pos, ori):
		# TODO make use of the data in tick
		oldX = self.X
		self.X = np.array(pos)
		self.V = self.X - oldX
		self.R = np.array(ori)
		
	def update_bump(self, bump):
		# TODO make use of the data in tick
		self.bump = bump

	def update_box_pos(self, pos):
		self.box_pos = np.array([pos.x,pos.y,pos.z])

	def update_range(self, r):
		# TODO make use of the data in tick
		self.range = r

	def set_target(self, T):
		self.target = np.array(t)

	def add_target(self, T, multi):  # multi=False .insert
		if multi:
			for t in T:
				self.target_list.append(np.array(t))
		else:
			self.target_list.append(np.array(T))

	def print_status(self):
		print '=' * 10
		print self.name
		print 'X', self.X
		print 'R', self.R
		print 'V', self.V
		print 'bump', self.bump
		print 'range', self.range
		print 'status', self.status
		print 'target', self.target
		print 'index', self.index
		print 'temp_index', self.temp_index
		print '=' * 10


	def distance(self,point2):
		return math.sqrt(math.pow((self.X[1] - point2[1]), 2) + math.pow((self.X[0] - point2[0]), 2))

	def distance(self, point1, point2):
		return math.sqrt(math.pow((point1[1] - point2[1]), 2) + math.pow((point1[0] - point2[0]), 2))


	# currently returns the box_pos from simulation, if range sensor implemented rotate to find target
	def search(self):
		return self.box_pos

	def go_to_target(self):
		#vector from center of robot to target object
		v = self.search() - self.X
		r = np.linalg.norm(np.array(v))
		if r > C1:
			v = self.search()
		elif D1 < r < C1:
			v = self.search() * ((r-D1)/(C1-D1))
		else:
			v = v *0.
		return v

	def dock(self):
		Vr_to_t = self.search()
		angle_g_to_t=math.atan2((self.goal[1] - self.search()[1]), (self.goal[0] - self.search()[0]))
		angle_r_to_t=math.atan2((self.X[1] - self.search()[1]), (self.X[0] - self.search()[0]))
		theta = angle_r_to_t - angle_g_to_t
		if theta < 0:
			Vperp = np.array([-Vr_to_t[1],Vr_to_t[0],Vr_to_t[2]]) # turn left?
			theta = math.pi + theta
		else:
			Vperp = np.array([Vr_to_t[1],-Vr_to_t[0],Vr_to_t[2]]) # turn right?
			theta = math.pi - theta
		alpha = theta/THETA_MAX
		if theta > THETA_MAX:
			#print 'v perp', Vperp
			return Vperp
		else:
			#print 'v perp + ...', alpha*Vperp + (1-alpha)*Vr_to_t
			return alpha*Vperp + (1-alpha)*Vr_to_t

	def push(self):
		Vr_to_t = self.search()-self.X
		Vt_to_g = self.goal-self.search()
		r = np.linalg.norm(np.array(Vr_to_t))
		if r > D3:
			v_push = Vr_to_t - LAMBDA*Vt_to_g
		else:
			v_push = Vr_to_t + LAMBDA*Vt_to_g
		#print 'v_push', v_push
		return np.array(self.goal)

	def acquire(self):
		v = self.search() - self.X
		r = np.linalg.norm(np.array(v))
		if r > C4:
			beta=1
		elif D4 < r < C4:
			beta=(r-D4)/(C4-D4)
		elif r < D4:
			beta=0
		v_alignment= beta*self.go_to_target() + (1-beta)*np.array(self.dock())
		return np.array(v_alignment)

	def move(self):

		if self.distance(self.box_pos,self.goal) < 2.0:
			self.v = 0.
			self.omega = 0.
			return

		if self.distance(self.box_pos, self.X) > 0.25:
			self.v,self.omega = motion.update_motion(self.X,self.R,self.acquire())
		else:
			self.v = 0.
			self.omega = 0.
		'''
		if self.v == 0 and self.omega == 0:
			print 'Pushing!!!!!!!!!!!!!'
			self.v,self.omega = motion.update_motion(self.X,self.R,self.push())
		print 'v =======> ',self.v
		print 'push =======> ',self.push()
		print 'acquire =======> ',self.acquire()
		print 'Box ############=======> ',self.box_pos
		print 'dist to Box', self.distance(self.box_pos, self.X)
		print 'dist to goal', self.distance(self.box_pos,self.goal)
		'''
	def normalize(self,V):
		n = np.linalg.norm(np.array(V))
		if n == 0:
			return V
		else:
			return np.array(V)/n

	

# -*- coding: utf-8 -*-
"""
Created on Tue Apr  5 15:22:13 2016

@author: ozer
"""
import geometry as geom

class Benchmarker():
	def __init__(self,stime = -1., sbump = -10., sfalse = -500, victim_locs = [], victim_size = 1):
		self.stime = stime #score per tick
		self.sbump = sbump #score per bumping tick
		self.sfalse = sfalse #score per false victim tick
		self.robots = {}
		self.victim_locs = victim_locs
		self.victim_found = [False for i in range(len(victim_locs))]
		self.victim_size = victim_size

	def add_model(self, model):
		self.robots[model.name] = {'score':0, 'model':model}

	def update(self,model_name, victim_pos = None):
		model = self.robots[model_name]['model']
		score = self.robots[model_name]['score']

		#score delta
		d_score = 0

		#penalize time
		d_score += self.stime

		#penalize bumping
		if model.bump == 'bump':
			d_score += self.sbump

		if victim_pos is not None: #robot says it found a victim
			for i in range(len(self.victim_locs)):
				victim_loc = self.victim_locs[i]
				dist = geom.euclidDistance(victim_pos,victim_loc)
				if dist <= self.victim_size: #found victim?
					print model_name,'found victim at',str(victim_pos)
					self.victim_found[i] = True
					print '!!!!!!!!!!!!!!!!!!!VICTIM DETECTED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
				else:
					continue
			if self.victim_found[i] == False: #false victim
				print model_name,'found FALSE victim at',str(victim_pos)
				d_score += self.sfalse

		if all(self.victim_found):
			print 'ALL VICTIMS FOUND'
			for model_name in self.robots:
				self.robots[model_name]['model'].status = 'stop'

		#update score
		self.robots[model_name]['score'] = score+d_score


	def print_marks(self):
		for model_name in self.robots:
			print model_name, self.robots[model_name]['score']


# -*- coding: utf-8 -*-
"""
Created on Sat Mar 12 15:33:50 2016

@author: kkalem
"""

import matplotlib.pyplot as plt

class PlotView():
	def __init__(self,model):
		self.model = model
		self.pos_trace = []

		plt.axis('equal')

		r = model.size
		c1 = plt.Circle(model.X,r, fill = False)
		fig = plt.gcf()
		fig.gca().add_artist(c1)



	def update(self):
		self.model.tick()
		plt.pause(0.0001)

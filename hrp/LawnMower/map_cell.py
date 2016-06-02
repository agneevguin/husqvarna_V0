# -*- coding: utf-8 -*-
"""
Created on Mon Apr 11 16:36:04 2016

@author: ermias
"""

class cell():
    def __init__(self, pos=[0.,0.], obstacle=false, wall=None, wall_probability=0.):
        #Lower right corner of the cell
        self.pos = pos
        self.obstacle = obstacle
        self.wall = wall
        self.wall_prob = wall_probability
        
class discrete_map():
    def __init__(self, X=80., Y=80., cell_size=1, buffer_size=4):
        self.X = X-buffer_size
        self.Y = Y-buffer_size
        self.cells = []
        for x in xrange(0, self.X, cell_size):
            for y in xrange(0, self.Y, cell_size):
                self.cells.append(cell([x - self.X/2, self.Y/2 - y]))
            
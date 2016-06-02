# -*- coding: utf-8 -*-
"""
Created on Mon Apr 25 14:27:58 2016

@author: agneev
"""

import math
import tf
import matplotlib.pylab as plt

def mapping(pos, orientation, target, trial_map):
	x = 0.5 * ceil(2.0 * pos[0])
	y = 0.5 * ceil(2.0 * pos[1])
	if bump detected:
		trial_map[x][y] = 4.5
		turn left, move forward
			if bump detected:
				goto line 12
			else 
				turn right, move forward
					if bump detected:
						goto line 12
					else:
						turn right 
							if bump detected:
								goto line 12
							else:
								goto line 20
	elif target detected:
		trial_map[x][y] = 1.5
	else:
		trial_map[x][y] = 0.0

	
	fig = plt.figure()
	ax = fig.add_subplot(1,1,1)
	ax.set_aspect('equal')
	plt.imshow(trial_map, interpolation='nearest', cmap=plt.cm.ocean, extent=(0.5,10.5,0.5,10.5))
	plt.colorbar()
	plt.show()
									
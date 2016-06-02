def triangulate(allpoints):
	# allpoints in format [(x1, y1, d1), (x2, y2, d2), (x3, y3, d3)]    
	# where (x1, y1), (x2, y2) and (x3, y3) are reference points and d1, d2 and d3 are distances from the reference points
	# (x, y) gives the reference point	
	ws = 0.		
	x = 0.
	y = 0.
	for p in allpoints:
		ws += p[2]	
	points = tuple((x,y,signal/ws) for (x,y,signal) in allpoints)
	x = 0.
	y = 0.
	for p in points:
		x += p[0]*p[2]
		y += p[1]*p[2]
	return (x, y)
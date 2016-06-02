import csv
from planner import Point
from planner import Edge
from planner import Polygon
from planner import visibility_graph
from planner import point_string
from planner import edge_string

def visibility_path(X, Y):
	
	start = Point(X[0], X[1])
	goal = Point(Y[0], Y[1])	
	
	#start = Point(0.0, 5.0)
	#goal = Point(10.0, 5.0)
	'''
	polygonX = [8.8709677419, 8.6405529954, 21.0829493088, 32.3732718894, 58.8709677419, 77.534562212, 53.5714285714, 13.9400921659, 14.8617511521, 56.1059907834, 72.9262672811, 94.5852534562, 93.4331797235, 69.0092165899, 75.4608294931, 75, 37.6728110599, 35.599078341, 19.4700460829, 28.9170506912, 29.3778801843, 17.1658986175, 12.0967741935]
	polygonY = [84.649122807, 62.134502924, 62.4269005848, 75.5847953216, 76.4619883041, 66.5204678363, 42.8362573099, 40.4970760234, 47.8070175439, 57.7485380117, 50.4385964912, 51.0233918129, 38.1578947368, 37.865497076, 25.5847953216, 13.5964912281, 19.4444444444, 29.6783625731, 35.2339181287, 34.0643274854, 13.0116959064, 13.0116959064, 24.4152046784]
	endPoints = [1, 1, 1, 3, 1, 1, 1, 1, 1, 3, 1, 1, 1, 3, 1, 1, 1, 3, 1, 1, 1, 1, 3]
	
	
	polygonX = [10.0,10.0,15.0,15.0,-15.0,-15.0,-10.0,-10.0]
	polygonY = [15.0,10.0,10.0,15.0,-10.0,-15.0,-15,-10.0]
	endPoints = [1,1,1,3,1,1,1,3]
	'''
	polygonX = [2.5,2.5,7.5,7.5,10.0,12.5,10.0,12.5]
	polygonY = [2.5,7.5,7.5,2.5,5.0,5.0,10.0,10.0]
	endPoints = [1,1,1,3,1,1,1,3]
	'''
	
	# Map Points (4 rectangular polygons)
	polygonX = [-10.0,-10.0,-5.0,-5.0,10.0,10.0,15.0,15.0,5.0,5.0,15.0,15.0,-15.0,-15.0,-10.0,-10.0]
	polygonY = [15.0,-5.0,-5.0,15.0,15.0,10.0,10.0,15.0,-5.0,-10.0,-10.0,-5.0,-10.0,-15.0,-15.0,-10.0]
	endPoints = [1,1,1,3,1,1,1,3,1,1,1,3,1,1,1,3]
	
	
	#Extended map points for (4 rectangular polygons)
	polygonX = [-12,-12,-10,-5,-3,-3,-5,-10,-12,8,8,10,15,17,17,15,10,8,3,3,5,15,17,17,15,5,3,-17,-17,-15,-10,-8,-8,-10,-15,-17]
	polygonY = [15,-5,-7,-7,-5,15,17,17,15,15,10,8,8,10,15,17,17,15,-5,-10,-12,-12,-10,-5,-3,-3,-5,-10,-15,-17,-17,-15,-10,-8,-8,-10]
	endPoints = [1,1,1,1,1,1,1,1,3,1,1,1,1,1,1,1,1,3,1,1,1,1,1,1,1,1,3,1,1,1,1,1,1,1,1,3]
	'''
	
	polygonStart = 0
	point_list = []
	edge_list = []
	allPolygons = []
	counter = 0
	for i in endPoints:
		if(i == 3):
			edge = Edge(Point(polygonX[counter], polygonY[counter]),Point(polygonX[polygonStart], polygonY[polygonStart]))
			#update to new polygon
			polygonStart = counter+1; 	
		else:
			edge = Edge(Point(polygonX[counter], polygonY[counter]),Point(polygonX[counter+1], polygonY[counter+1]))
		
		point_list.append(Point(polygonX[counter], polygonY[counter]))
		edge_list.append(edge)
		if(i == 3):
			onePolygon = Polygon(point_list, edge_list)
			allPolygons.append(onePolygon)
			point_list = []
			edge_list = []
		counter = counter + 1
	
	graph = visibility_graph(allPolygons, start, goal)
	
	#returns all feature vertices
	for vertex in graph[0]:
		print point_string(vertex)
		
	'''
	#returns all feature edges
	for edge in graph[1]:
		print edge_string(edge)
	'''
	print '=' * 10
	
	#graph = (vertices, edges)
	return graph 	
	'''
	def search(start, target):
		return 1
	
	def heuristicCost(a, b):
		return math.sqrt(math.pow((a[1] - b[1]), 2) + math.pow((a[0] - b[0]), 2))
	'''
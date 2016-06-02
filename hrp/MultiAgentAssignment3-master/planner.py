import math

def point_string(point):
	l = [ "(", str(point.x), ", ", str(point.y), ")" ]
	s = ""
	for i in l:
		s += i
	return s


def edge_string(edge):
	return point_string(edge.points[0]) + " to " + point_string(edge.points[1])


class Edge:
	def __init__(self, point1, point2):
		self.points = (point1, point2)

	def contains(self, point):
		for p in self.points:
			if point == p:
				return True
		return False

	def __eq__(self, e):
		return set(self.points) == set(e.points)

	def __hash__(self):
		hash = 0
		for point in self.points:
			hash += point.__hash__()
		return hash


class Point:
	def __init__(self, x, y):
		self.x = x
		self.y = y

	def __eq__(self, p):
		return self.x == p.x and self.y == p.y

	def __hash__(self):
		return self.x.__hash__() + self.y.__hash__()


class Polygon:
	def __init__(self, points, edges):
		self.points = points
		self.edges = edges

	def contains_point(self, point):
		return point in set(self.points)

	def contains_edge(self, edge):
		return edge in set(self.edges)


# Returns a list of vertices and edges forming a visibility graph
def visibility_graph(polygons, start, goal, width = 0):
	polygons = polygon_expansion(polygons, width)

	vertices = set([])
	edges = set([])

	for polygon in polygons:
		for vertex in polygon.points:
			vertices.add(vertex)

	vertices.add(start)
	vertices.add(goal)

	for vertex in vertices:
		for other in visible_vertices(vertex, polygons, start, goal):
			edges.add(Edge(vertex, other))

	graph = (vertices, edges)

	interior_points = find_interior_points(graph, polygons)
	graph = remove_points_from_graph(graph, interior_points)

	concave_points = find_concave_points(graph, polygons)
	graph = remove_points_from_graph(graph, concave_points)

	return graph


# Expands all polygons by the given width
def polygon_expansion(polygons, width):
	# TODO
	return polygons


# Removes points and edges including those points, returns the new graph
def remove_points_from_graph(graph, points):
	# TODO
	vertices = graph[0]
	edges = graph[1]

	for point in points:
		if point in vertices:
			vertices.remove(point)
		for edge in edges.copy():
			if edge.contains(point):
				edges.remove(edge)
	newGraph = (vertices, edges)
	return newGraph


# Returns a set of all points in the graph that lie within any of the polygons
def find_interior_points(graph, polygons):
	interior_vertices = set([])
	for vertex in graph[0]:
		for polygon in polygons:
			intersect_count = 0
			for edge in polygon.edges:
				if edge_intersect(vertex, Point(vertex.x + 10, vertex.y), edge):
					if edge.points[0].x == edge.points[1].x:
						if edge.points[0].x > vertex.x:
							intersect_count += 1
					else:
						slope = (edge.points[0].y - edge.points[1].y) / (edge.points[0].x - edge.points[1].x)
						x = edge.points[0].x - edge.points[0].y / slope
						if x > vertex.x:
							intersect_count += 1
			for point in polygon.points:
				if point.y == vertex.y:
					for edge in polygon.edges:
						if edge.contains(point):
							if not counterclockwise(vertex, edge, point):
								intersect_count += 1
			if intersect_count % 2:
				interior_vertices.add(vertex)
	return interior_vertices


# Returns a list of all points in the graph that are concave
def find_concave_points(graph, polygons):
	# TODO
	return []


# Returns a list of edges from point to other visible points
def visible_vertices(point, polygons, start, goal):
	# Compute angles for each point in polygons
	# Sort points by angle from x-axis, closer points first in ties
	# Create edge list E
	# Create point list W
	# Start with first point, add all intersected edges to E
	# For each point in order:
	# 	If point is on the nearest edge:
	# 		Add point to W
	# 	Remove edges from E that are clockwise from point
	# 	Add edges to E that are counterclockwise from point
	# Return W

	# Set of all points that could potentially be reached
	points = set([])
	for polygon in polygons:
		for vertex in polygon.points:
			points.add(vertex)
	points.add(start)
	points.add(goal)
	points.remove(point)
	points = list(points)

	# Sorts points by angle from x-axis with point as origin
	points.sort(key = lambda p: angle(point, p))

	# Finds all edges in polygons
	all_edges = set([])
	for polygon in polygons:
		for edge in polygon.edges:
			all_edges.add(edge)

	# Initializes edge set and adds edges to first point
	# Edges with first point will be removed, no need to account
	open_edges = []
	for edge in all_edges:
		if edge_intersect(point, points[0], edge):
			open_edges.append(edge)
	open_edges.sort(key = lambda e: edge_distance(point, points[0], e))

	# Points list for visible vertices
	visible = []

	# Checks each point for visibility
	for next_point in points:
		# OPTIMIZE: Track edges for a given point
		# Would save looking through all edges
		for edge in all_edges:
			if edge.contains(next_point):
				try:
					open_edges.remove(edge)
				except ValueError:
					pass
		if len(open_edges) == 0 or euclidean_distance(point, next_point) <= edge_distance(point, next_point, open_edges[0]):
			visible.append(next_point)
		for edge in all_edges:
			if edge.contains(next_point):
				if (not edge.contains(point)) and counterclockwise(point, edge, next_point):
					open_edges.append(edge)
			open_edges.sort(key = lambda e: edge_distance(point, next_point, e))

	# Remove edges that cross through polygons
	for polygon in polygons:
		if polygon.contains_point(point):
			for p in polygon.points:
				if not polygon.contains_edge(Edge(point, p)):
					try:
						visible.remove(p)
					except ValueError:
						pass

	return visible


# Computes euclidian distance
def euclidean_distance(point1, point2):
	return math.sqrt(math.pow(point1.x - point2.x, 2) + math.pow(point1.y - point2.y, 2))


# Computes the angle from the x-axis to point in radians [0, 2*pi) with center as the origin
def angle(center, point):
	dx = point.x - center.x
	dy = point.y - center.y

	if dx == 0:
		if dy < 0:
			return math.pi * 3 / 2
		else:
			return math.pi / 2

	if dy == 0:
		if dx < 0:
			return math.pi
		else:
			return 0

	if dx < 0:
		return math.pi + math.atan(dy / dx)

	if dy < 0:
		return 2 * math.pi + math.atan(dy / dx)

	return math.atan(dy / dx)


# Returns true if edge is intersected by the line passing through the given points
# Considered false if the edge contains either point
def edge_intersect(point1, point2, edge):
	if edge.contains(point1) or edge.contains(point2):
		return False

	if point1.x == point2.x:
		x1_left = edge.points[0].x < point1.x
		x2_left = edge.points[1].x < point1.x
		return not (x1_left == x2_left)

	slope = (point1.y - point2.y) / (point1.x - point2.x)

	y1_ex = slope * (edge.points[0].x - point1.x) + point1.y
	y2_ex = slope * (edge.points[1].x - point1.x) + point1.y

	if y1_ex == edge.points[0].y or y2_ex == edge.points[1].y:
		return False

	y1_below = (y1_ex > edge.points[0].y)
	y2_below = (y2_ex > edge.points[1].y)

	return not (y1_below == y2_below)


# Returns the distance from a point to an edge along the line to another point
# If the intersect is not within the edge, won't return a valid distance. So don't do that.
def edge_distance(point, other_point, edge):
	if edge.points[0].x == edge.points[1].x:
		if point.x == other_point.x:
			# Bad input, no intersection
			return 0
		points_slope = (point.y - other_point.y) / (point.x - other_point.x)
		intersect_x = edge.points[0].x
		intersect_y = points_slope * (intersect_x - point.x) + point.y
		intersect = Point(intersect_x, intersect_y)
		return euclidean_distance(intersect, point)

	if point.x == other_point.x:
		edge_slope = (edge.points[0].y - edge.points[1].y) / (edge.points[0].x - edge.points[1].x)
		intersect_x = point.x
		intersect_y = edge_slope * (intersect_x - edge.points[0].x) + edge.points[0].y
		intersect = Point(intersect_x, intersect_y)
		return euclidean_distance(intersect, point)

	edge_slope = (edge.points[0].y - edge.points[1].y) / (edge.points[0].x - edge.points[1].x)
	points_slope = (point.y - other_point.y) / (point.x - other_point.x)

	if edge_slope == points_slope:
		# Bad input, no intersection
		return 0

	intersect_x = (edge_slope * edge.points[0].x - points_slope * point.x + point.y - edge.points[0].y) / (edge_slope - points_slope)
	intersect_y = edge_slope * (intersect_x - edge.points[0].x) + edge.points[0].y

	intersect = Point(intersect_x, intersect_y)

	return euclidean_distance(intersect, point)


# Returns true if the edge goes counterclockwise from the line through point and endpoint
# Only use if endpoint is an endpoint in edge
def counterclockwise(point, edge, endpoint):
	if edge.points[0] == endpoint:
		angle_diff = angle(point, edge.points[1]) - angle(point, endpoint)
	else:
		angle_diff = angle(point, edge.points[0]) - angle(point, endpoint)

	if angle_diff < 0:
		angle_diff += 2 * math.pi

	return angle_diff < math.pi

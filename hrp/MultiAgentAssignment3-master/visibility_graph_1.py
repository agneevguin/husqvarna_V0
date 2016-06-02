#from igraph import *


def visibility_path():
	g = Graph()
	g.add_vertices(3)
	g.add_edges([(0,1), (1,2)])
	g.add_edges((2,0))
	g.add_vertices(3)
	g.add_edges([(2,3),(3,4),(4,5),(5,3)])
	print g
	print g.get_edgelist()[0:10]

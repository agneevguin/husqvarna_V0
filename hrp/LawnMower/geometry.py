# -*- coding: utf-8 -*-
"""
Created on Sun Feb 21 11:44:13 2016

@author: kkalem
"""

import numpy as np
import matplotlib.pyplot as plt


def norm(V):
    return np.linalg.norm(np.array(V))

def normalize(V):
    n = norm(V)
    if n == 0:
        return V
    else:
        return np.array(V)/n


def perp_vec(V):
    P = np.dot(V,np.array([[0,-1],[1,0]]))
    return P

def plot_line(wall,**kwargs):
    plt.plot([wall[0][0],wall[1][0]],[wall[0][1],wall[1][1]],**kwargs)


def euclidDistance(pos1,pos2):
    return np.sqrt((pos2[0]-pos1[0])**2 + (pos2[1]-pos1[1])**2)

# returns a line in the form y = mx+b, returns the m,b
def lineFrom2pts(A,B):
    m = (A[1]-B[1]) / (A[0]-B[0])
    b = A[1]-A[0]*m
    return m,b

#returns the a,b,c of the line perpendicular to this one, passing through X
#y=mx+b
def perpendicularLine(m1,X):
    m2 = -1/m1
    b2 = X[1]-X[0]*m2
    return m2,b2

def in3d(pt,z=0):
    return [pt[0],pt[1],z]


#returns true if the given point is not in a polygon in the map
def validPoint(polys, pt):
    for p in polys:
        poly = polys[p]
        if len(poly) == 0:
            continue
        if ptInPoly(poly,pt):
            return False
    return True

#http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect/19550879#19550879
#taken from above
# l1, l2
def find_intersection( p0, p1, p2, p3 ) :

    s10_x = p1[0] - p0[0]
    s10_y = p1[1] - p0[1]
    s32_x = p3[0] - p2[0]
    s32_y = p3[1] - p2[1]

    denom = s10_x * s32_y - s32_x * s10_y

    if denom == 0 : return 'collinear' # collinear

    denom_is_positive = denom > 0

    s02_x = p0[0] - p2[0]
    s02_y = p0[1] - p2[1]

    s_numer = s10_x * s02_y - s10_y * s02_x

    if (s_numer < 0) == denom_is_positive : return None # no collision

    t_numer = s32_x * s02_y - s32_y * s02_x

    if (t_numer < 0) == denom_is_positive : return None # no collision

    if (s_numer > denom) == denom_is_positive or (t_numer > denom) == denom_is_positive : return None # no collision


    # collision detected

    t = t_numer / denom

    intersection_point = [ p0[0] + (t * s10_x), p0[1] + (t * s10_y) ]


    return intersection_point


#https://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
#taken from above
# MAGIC !!
def __pnpoly(nvert, vertx, verty, testx, testy):
    i = 0
    j = nvert-1
    c = False
    while i < nvert:
        #body
        if ((verty[i] > testy) != (verty[j]>testy)):
            if (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]):
                c = not c
        #/body
        j = i
        i += 1
    return c

#wrapper for pnpoly
#poly is a list of vectors for edge, pt is a 2D point as a vector
def ptInPoly(poly,pt):
    poly = np.array(poly)
    nvert = len(poly[:,0])
    vertx = poly[:,0]
    verty = poly[:,1]
    testx = pt[0]
    testy = pt[1]
    return __pnpoly(nvert,vertx,verty,testx,testy)

#http://stackoverflow.com/a/2233538
#taken from above
def _dist(x1,y1, x2,y2, x3,y3): # x3,y3 is the point
    px = x2-x1
    py = y2-y1

    something = px*px + py*py

    u =  ((x3 - x1) * px + (y3 - y1) * py) / float(something)

    if u > 1:
        u = 1
    elif u < 0:
        u = 0

    x = x1 + u * px
    y = y1 + u * py

    dx = x - x3
    dy = y - y3

    # Note: If the actual distance does not matter,
    # if you only want to compare what this function
    # returns to other results of this function, you
    # can just return the squared distance instead
    # (i.e. remove the sqrt) to gain a little performance

    dist = np.sqrt(dx*dx + dy*dy)

    return dist

# A,B make the line segment, P is the point
def ptToLineSegment(A,B,P):
    return _dist(A[0],A[1],B[0],B[1],P[0],P[1])

#returns the coordiantes for a point between p1 p2 at percentage away from p1
def tracePoint(p1,p2,percent):
    L = euclidDistance(p1,p2)
    l = percent * L
    a = (p2[0]-p1[0]) * (l/L) + p1[0]
    b = (p2[1]-p1[1]) * (l/L) + p1[1]
    return [a,b]

def subdividePath(ideal_path, divs = 1):
    res = [ideal_path[0]]
    for i in range(1,len(ideal_path)):
        mid_pts = []
        for div in range(1,divs+1):
            perc = div/(divs+1.)
            mid_pt = tracePoint(ideal_path[i-1],ideal_path[i],perc)
            mid_pts.append(mid_pt)
        res.extend(mid_pts)
    res.append(ideal_path[-1])
    return res
#! /usr/bin/env python

# Python stuff
from math import sqrt,cos,sin,atan2
import os, sys
import array
import ctypes
import time
import random

# ROS imports
from am_planning.srv import *
from geometry_msgs.msg import PolygonStamped, Point32, Point, PoseStamped, Pose
from nav_msgs.msg import Path
import rospy


from astar import A_star


UNIT_TEST = False

if UNIT_TEST:
    from pygame.locals import *
    import pygame

    XDIM = 640
    YDIM = 480
    WINSIZE = [XDIM, YDIM]

    pygame.init()
    screen = pygame.display.set_mode(WINSIZE)
    pygame.display.set_caption('BiDirectionalRRT Test')
    white = 255, 240, 200
    red = 255, 0, 0
    green = 0, 255, 0
    blue =  0, 0, 255
    gray = 200,200,200
    black = 20, 20, 40
    screen.fill(black)


def astarFindPath(req):

    border = []
    for point in  req.border.polygon.points:
        border.append( (point.x, point.y) )

    #print "Border:", border
        
    start = (req.start.pose.position.x, req.start.pose.position.y)
    #print "Start:", start
    
    goal = (req.goal.pose.position.x, req.goal.pose.position.y)
    #print "Goal:", goal


    path = A_star(0.5, 4, border, start, goal)
    
    #print "Result:", path
    return path


def buildShapeH(polygon, bot):
    fixedConfHeight = 3.0
    fixedConfWidth  = 3.0
    
    # 1
    p = Point32()
    p.x = bot.x - 1.5 * fixedConfWidth
    p.y = bot.y + 1.5 * fixedConfHeight
    p.z = bot.z
    polygon.points.append(p)

    # 2
    p = Point32()
    p.x = bot.x - 0.5 * fixedConfWidth
    p.y = bot.y + 1.5 * fixedConfHeight
    p.z = bot.z
    polygon.points.append(p)

    # 3
    p = Point32()
    p.x = bot.x - 0.5 * fixedConfWidth
    p.y = bot.y + 0.5 * fixedConfHeight
    p.z = bot.z
    polygon.points.append(p)

    # 4
    p = Point32()
    p.x = bot.x + 0.5 * fixedConfWidth
    p.y = bot.y + 0.5 * fixedConfHeight
    p.z = bot.z
    polygon.points.append(p)

    # 5
    p = Point32()
    p.x = bot.x + 0.5 * fixedConfWidth
    p.y = bot.y + 1.5 * fixedConfHeight
    p.z = bot.z
    polygon.points.append(p)

    # 6
    p = Point32()
    p.x = bot.x + 1.5 * fixedConfWidth
    p.y = bot.y + 1.5 * fixedConfHeight
    p.z = bot.z
    polygon.points.append(p)

    # 7
    p = Point32()
    p.x = bot.x + 1.5 * fixedConfWidth
    p.y = bot.y - 1.5 * fixedConfHeight
    p.z = bot.z
    polygon.points.append(p)

    # 8
    p = Point32()
    p.x = bot.x + 0.5 * fixedConfWidth
    p.y = bot.y - 1.5 * fixedConfHeight
    p.z = bot.z
    polygon.points.append(p)

    # 9
    p = Point32()
    p.x = bot.x + 0.5 * fixedConfWidth
    p.y = bot.y - 0.5 * fixedConfHeight
    p.z = bot.z
    polygon.points.append(p)

    # 10
    p = Point32()
    p.x = bot.x - 0.5 * fixedConfWidth
    p.y = bot.y - 0.5 * fixedConfHeight
    p.z = bot.z
    polygon.points.append(p)

    # 11
    p = Point32()
    p.x = bot.x - 0.5 * fixedConfWidth
    p.y = bot.y - 1.5 * fixedConfHeight
    p.z = bot.z
    polygon.points.append(p)

    # 12
    p = Point32()
    p.x = bot.x - 1.5 * fixedConfWidth
    p.y = bot.y - 1.5 * fixedConfHeight
    p.z = bot.z
    polygon.points.append(p)

    #print polygon


def buildCollisionEdges(req):
    collisionEdges = []
    
    # Get last point in polygon as a start
    prevPoint = req.border.polygon.points[-1]
    for point in req.border.polygon.points:
        edge = (prevPoint, point)
        collisionEdges.append(edge)
        prevPoint = point
    return collisionEdges


def drawLine(pin1, pin2, color):
    p1 = (int(XDIM/2 + pin1.x*40), int(YDIM/2 + pin1.y*40))
    p2 = (int(XDIM/2 + pin2.x*40), int(YDIM/2 + pin2.y*40))
    pygame.draw.line(screen, color, p1, p2)
    pygame.display.update()


def unitTest():
    print "Unit Test running..."

    req = PathToPoseRequest()
    
    # Build a sample border for this test
    req.border = PolygonStamped()
    
    center = Point32()
    center.x = 0.0
    center.y = 0.0
    center.z = 0.0
    buildShapeH(req.border.polygon, center)
    
    req.start = PoseStamped()
    req.start.pose.position.x = -3.0
    req.start.pose.position.y = 3.0
    req.start.pose.position.z = 0.0

    req.goal = PoseStamped()
    req.goal.pose.position.x = 3.0
    req.goal.pose.position.y = -3.0
    req.goal.pose.position.z = 0.0

    edges = buildCollisionEdges(req)
    for edge in edges:
        drawLine(edge[0], edge[1], white)
    
    path = astarFindPath(req)
    
    startPoint = path[0]
    for p in path[1::]:
        drawLine(Point(startPoint[0], startPoint[1], 0), Point(p[0], p[1], 0), red)
        startPoint = p
    
    
    result = PathToPose()
    result.path = Path()
    for p in path:
        pose = Pose()
        pose.position.x = p[0]
        pose.position.y = p[1]
        pose.position.z = 0
        
        result.path.poses.append(pose)
        
    print "Unit Test completed..."
    while True:
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Leaving because you requested it.")
    
    
    
    
###
### ROS STUFF
###
def handlePathFromPolygon(req):
    print "handlePathFromPolygon service called"

    myReq = req
    '''    
    # Build a sample border for this test
    myReq.border = PolygonStamped()
    
    center = Point32()
    center.x = 0.0
    center.y = 0.0
    center.z = 0.0
    buildShapeH(myReq.border.polygon, center)
    
    myReq.start = PoseStamped()
    myReq.start.pose.position.x = 1.7
    myReq.start.pose.position.y = -4.2
    myReq.start.pose.position.z = 0.0

    myReq.goal = PoseStamped()
    myReq.goal.pose.position.x = -1.2
    myReq.goal.pose.position.y = -1.2
    myReq.goal.pose.position.z = 0.0
    '''
    path = astarFindPath(myReq)

    result = PathToPoseResponse(req)
    result.path = Path()

    # Add the "start" as first point
    #pose = PoseStamped()
    #pose.pose.position.x = req.start.pose.position.x;
    #pose.pose.position.y = req.start.pose.position.y;
    #pose.pose.position.z = 0
    #result.path.poses.append(pose)
    

    #for i in range(1, len(path)-2):
    #
    #    p = path[i]
    for p in path:
    
        pose = PoseStamped()
        pose.pose.position.x = p[0]
        pose.pose.position.y = p[1]
        pose.pose.position.z = 0
        
        result.path.poses.append(pose)

    # Add the "goal" as last point
    #pose = PoseStamped()
    #pose.pose.position.x = req.goal.pose.position.x;
    #pose.pose.position.y = req.goal.pose.position.y;
    #pose.pose.position.z = 0
    #result.path.poses.append(pose)

    return result

def pathFromPolygonServer():
    rospy.init_node('PathFromPolygonServer')
    s = rospy.Service('get_astar_path_to', PathToPose, handlePathFromPolygon)
    print "A* - Path planner ready Sir!"
    rospy.spin()
 

if __name__ == "__main__":

  
    if UNIT_TEST:
        unitTest()
    else:
        pathFromPolygonServer()
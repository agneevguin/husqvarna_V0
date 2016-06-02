#! /usr/bin/env python

# Python stuff
from math import sqrt,cos,sin,atan2
import os, sys
import array
import ctypes
import time
import random

# ROS imports
from am_planning.srv import PathToPose
from geometry_msgs.msg import PolygonStamped, Point32, Point, PoseStamped
from nav_msgs.msg import Path
import rospy

UNIT_TEST = True

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


class BiDirectionalRRT(object):
    def __init__(self, req, draw):
        
        self.req = req
        self.startNodes = []
        self.goalNodes = []

        self.startTree = []
        self.goalTree = []

        self.maxIterations = 5000
        
        self.path = Path()
        
        self.maxSampleRadius = 100.0
        self.maxStepLength = 0.3
        self.minDistance = 0.3
        
        # Prepare by building all polygon edges
        self.buildCollisionEdges()
        
        self.findMinMax()
        
        self.draw = draw
        if self.draw:
            for edge in self.collisionEdges:
                self.drawLine(edge[0], edge[1], white)
        

    def sqr(self, x):
        return x*x

    def dist2(self, v, w):
        return self.sqr(v.x - w.x) + self.sqr(v.y - w.y)

    # Return minimum distance (squared) between line segment vw and point p
    def distToSegmentSquared(self, p, v, w):
        l2 = self.dist2(v, w)
        
        if l2 == 0:
            return self.dist2(p, v)

        t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l2

        if t < 0:
            return self.dist2(p, v)

        if t > 1:
            return self.dist2(p, w)

        vproj = Point()

        vproj.x = v.x + t * (w.x - v.x)
        vproj.y = v.y + t * (w.y - v.y)

        return self.dist2(p, vproj)


    def distToSegment(self, p, v, w):
        return sqrt(self.distToSegmentSquared(p, v, w))


    def findMinMax(self):
        self.minY = 1000.0
        self.minX = 1000.0
        self.maxY = -1000.0
        self.maxX = -1000.0

        for point in self.req.border.polygon.points:
            if point.x < self.minX:
                self.minX = point.x
            if point.x > self.maxX:
                self.maxX = point.x
            if point.y < self.minY:
                self.minY = point.y
            if point.y > self.maxY:
                self.maxY = point.y

    def buildCollisionEdges(self):
        self.collisionEdges = []
        
        # Get last point in polygon as a start
        prevPoint = self.req.border.polygon.points[-1]
        for point in self.req.border.polygon.points:
            edge = (prevPoint, point)
            self.collisionEdges.append(edge)
            prevPoint = point
            
        #print self.collisionEdges

    # http://www.bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
    def ccw(self, A, B, C):
        return (C.y-A.y)*(B.x-A.x) > (B.y-A.y)*(C.x-A.x)

    def intersects(self, line1, line2):
        # Test if a line intersects with this border,
        # that means collision
        A = line1[0]
        B = line1[1]

        C = line2[0]
        D = line2[1]

        return self.ccw(A,C,D) != self.ccw(B,C,D) and self.ccw(A,B,C) != self.ccw(A,B,D)

    def dist(self, p1, p2):
        return sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y))

    def nearest(self, randomNode, tree):
        nearestNode = tree[0]
        
        for n in tree:
            if self.dist(n,randomNode) < self.dist(nearestNode,randomNode):
                nearestNode = n
        return nearestNode
                
    def takeStep(self, p1, p2):
        if self.dist(p1,p2) < self.maxStepLength:
            return p2
        else:
            theta = atan2(p2.y-p1.y,p2.x-p1.x)
            return Point(p1.x + self.maxStepLength*cos(theta), p1.y + self.maxStepLength*sin(theta), 0.0)
            
    def isValidPathBetween(self, p1, p2):
        # Check if we collide with any border/obstacle line
        testLine = [p1, p2]
        #print "TestLine:", testLine
        for edge in self.collisionEdges:
            if self.intersects(edge, testLine):
                return False
            if self.distToSegment(p2, edge[0], edge[1]) < self.minDistance:
                return False
        #print "Ok path!"
        return True
        
    def drawLine(self, pin1, pin2, color):
        p1 = (int(XDIM/2 + pin1.x*40), int(YDIM/2 + pin1.y*40))
        p2 = (int(XDIM/2 + pin2.x*40), int(YDIM/2 + pin2.y*40))
        pygame.draw.line(screen, color, p1, p2)
        pygame.display.update()
    
    def checkEndLineForValidity(self, qlstart, qlgoal):
        connectionPath = []
        # Check if this path is split into several Points
        # and these are then also valid...
        dx = qlgoal.x - qlstart.x
        dy = qlgoal.y - qlstart.y
        numSteps = sqrt(dx*dx + dy*dy) / self.maxStepLength
        
        dx = dx / numSteps
        dy = dy / numSteps
        
        startPoint = Point(qlstart.x, qlstart.y, 0)
        
        i=0
        noProblem = True
        while i<numSteps and noProblem:
            endPoint = Point(startPoint.x + dx, startPoint.y + dy, 0)
            if self.isValidPathBetween(startPoint, endPoint):
                connectionPath.append([startPoint, endPoint])
            else: 
                noProblem = False
            startPoint = endPoint
            i = i + 1
        
        return noProblem
        
    def getConnectionNode(self, testNode, nodes):
        if testNode == None:
            return None
        for n in nodes:
            if self.isValidPathBetween(n, testNode):
                if self.checkEndLineForValidity(n, testNode):
                    return n
        return None
    
    def buildPath(self, lastStartNode, lastGoalNode):
        # Search backwards through START tree
        print "lastStart: ", lastStartNode
        print "lastGoal: ", lastGoalNode
        #print "goal: ", self.goalTree
        
        startPath = []
        startPath.append(self.startTree[-1])
    
    
    def findPath(self):
    
        # Initial positions
        self.startNodes.append(self.req.start.pose.position)
        self.goalNodes.append(self.req.goal.pose.position)
        
        qlstart = None
        qlgoal = None
        continueSearch = True
        iterations = 0

        while continueSearch and iterations < self.maxIterations:
        
            iterations = iterations + 1
        
            # Extend START tree
            nodes = self.startNodes
            tree  = self.startTree
            
            # Step 1 - Sample a point
            qr = Point(random.uniform(self.minX, self.maxX)*20.0, random.uniform(self.minY, self.maxY)*20.0, 0.0)
            
            # Step 2 - Find near node in tree
            qnear = self.nearest(qr, nodes)
            
            # Step 3 - Take the step towards the qnear
            qnewnode = self.takeStep(qnear, qr)
            
            # Step 4 - Add to tree...if not colliding
            if self.isValidPathBetween(qnear, qnewnode):
                nodes.append(qnewnode)
                qlstart = qnewnode
                tree.append([qnear, qnewnode])
                #print "Append START"

                if self.draw:
                    self.drawLine(qnear, qnewnode, green)

            # Finally check if there are any nodes that connect to this new node?
            qGoalNode = self.getConnectionNode(qlstart, self.goalNodes)
            if qGoalNode != None:
                    # Found path!
                    print "PATH!"
                    if self.draw:
                        self.drawLine(qGoalNode, qlstart, red)
                    self.buildPath(qlstart, qGoalNode)
                    continueSearch = False

            if continueSearch == False:
                break
                
            # Extend GOAL tree
            nodes = self.goalNodes
            tree  = self.goalTree
            
            # Step 1 - Sample a point
            qr = Point(random.uniform(self.minX, self.maxX)*20.0, random.uniform(self.minY, self.maxY)*20.0, 0.0)
            
            # Step 2 - Find near node in tree
            qnear = self.nearest(qr, nodes)
            
            # Step 3 - Take the step towards the qnear
            qnewnode = self.takeStep(qnear, qr)
            
            # Step 4 - Add to tree...if not colliding
            if self.isValidPathBetween(qnear, qnewnode):
                nodes.append(qnewnode)
                qlgoal = qnewnode
                tree.append([qnear, qnewnode])
                #print "Append GOAL"
            
                if self.draw:
                    self.drawLine(qnear, qnewnode, blue)

            # Finally check if there are any nodes that connect to this new node?
            qStartNode = self.getConnectionNode(qlgoal, self.startNodes)
            if qStartNode != None:
                    # Found path!
                    print "PATH!"
                    if self.draw:
                        self.drawLine(qStartNode, qlgoal, red)
                    self.buildPath(qStartNode, qlgoal)
                    continueSearch = False
                
                
            if self.draw:
                for e in pygame.event.get():
                    if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                        sys.exit("Leaving because you requested it.")
        
        
        # Continue to show resulting screen if we are testing
        while self.draw:
            for e in pygame.event.get():
                if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                    sys.exit("Leaving because you requested it.")
        
        return self.path


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

def unitTest():
    print "Unit Test running..."

    req = PathToPose()
    
    # Build a sample border for this test
    req.border = PolygonStamped()
    
    center = Point32()
    center.x = 0.0
    center.y = 0.0
    center.z = 0.0
    buildShapeH(req.border.polygon, center)
    
    req.start = PoseStamped()
    req.start.pose.position.x = -4.49
    req.start.pose.position.y = 4.2
    req.start.pose.position.z = 0.0

    req.goal = PoseStamped()
    req.goal.pose.position.x = 3.0
    req.goal.pose.position.y = -3.0
    req.goal.pose.position.z = 0.0

    rrt = BiDirectionalRRT(req, True)
    
    path = rrt.findPath()
    print path
    
    print "Unit Test completed..."
    
    
    
    
###
### ROS STUFF
###
def handlePathFromPolygon(req):
    print "handlePathFromPolygon service called"
    
    result = PathToPose()
    
    rrt = BiDirectionalRRT(req, False)
    path = rrt.findPath()
    
    return result

def pathFromPolygonServer():
    rospy.init_node('PathFromPolygonServer')
    s = rospy.Service('get_rrt_path_to', PathToPose, handlePathFromPolygon)
    print "RRT - Path planner ready Sir!"
    rospy.spin()
 

if __name__ == "__main__":

  
    if UNIT_TEST:
        unitTest()
    else:
        pathFromPolygonServer()
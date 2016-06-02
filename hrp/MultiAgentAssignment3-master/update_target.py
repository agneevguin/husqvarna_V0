# return next targets in queue, rangeDetectedValue
# included lastDirection and mapSize  FOR SECOND ALGORITHM
import model_diff_drive as mod
import math
import tf

def updateTarget(X, currentVel, currentOri, goal, victim, rangeSensor, bump, lastDirection, lastBump, mapSize):
    
    euler = tf.transformations.euler_from_quaternion(currentOri)
    T = [0.,0.,0.]
    dist_goal2victim = math.sqrt((goal[1]-victim[1])^2 + (goal[0]-victim[0])^2)
    dist_bot2victim = math.sqrt((X[1]-victim[1])^2 + (X[0]-victim[0])^2)
 
 
    theta = math.atan2((goal[1]-victim[1]),(goal[0]-victim[0]))
    ratio = dist_goal2victim/dist_bot2victim
				
    if(dist_goal2victim < 0.25):
        currentvel = [0.,0.,0.] #stop or return to start
    else:
			
        if(ratio>2.):
            T = [(victim[1]-2*math.cos(theta),victim[0]-2*math.sin(theta))]
        else:
            T = [(victim[1] - ratio*math.cos(theta), victim[0] - ratio*math.sin(theta))]
    
    return ratio
    '''
    if (0.65 < rangeSensor <= 2.25):
        targetPoint = [(X[0] + 0.45 + rangeSensor) * math.cos(euler[2]), X[1] * math.sin(euler[2]), X[2]]
        ''''''
        if (-math.pi/4 < euler[2] <= math.pi/4):                  #going in +X -drection
        
            T = [X[0] - 1., X[1] , X[2]], [X[0], X[1] + 2., X[2]], [X[0] + 4., X[1], X[2]], [X[0], X[1] - 2., X[2]], [X[0] + 1., X[1], X[2]]
        elif (math.pi/4 < euler[2] <= 3*math.pi/4):               #going in +Y -drection
            T = [X[0], X[1] - 1., X[2]], [X[0] + 2., X[1], X[2]], [X[0], X[1] + 4., X[2]], [X[0] - 2., X[1], X[2]], [X[0], X[1] + 1., X[2]]
        elif (- 3*math.pi/4 < euler[2] <= -math.pi/4):              #going in -Y -drection
            T = [X[0], X[1] + 1., X[2]], [X[0] - 2., X[1], X[2]], [X[0], X[1] - 4., X[2]], [X[0] + 2., X[1], X[2]], [X[0] - 1., X[1], X[2]]
        else:                                              #going in -X -drection
            T = [X[0] + 1., X[1], X[2]], [X[0], X[1] - 2., X[2]], [X[0] - 4., X[1], X[2]], [X[0], X[1] + 2., X[2]], [X[0] - 1., X[1], X[2]]
            
        ''''''
        if (lastDirection == 'Left' or lastDirection == 'None'):
            #update target to right
            if (-math.pi/4 < euler[2] <= math.pi/4):                  #going in +X -drection
                T = [-mapSize/2., X[1] - 1., X[2]]
            elif (math.pi/4 < euler[2] <= 3*math.pi/4):               #going in +Y -drection
                T = [X[0] + 1., -mapSize/2., X[2]]
            elif (-3*math.pi/4 < euler[2] <= -math.pi/4):              #going in -Y -drection
                T = [X[0] - 1., mapSize/2., X[2]]
            else:                                              #going in -X -drection
                T = [mapSize/2., X[1] + 1., X[2]]
            return T, targetPoint, 'Right', 'None'
                
        else:
            #update target to left
            if (-math.pi/4 < euler[2] <= math.pi/4):                  #going in +X -drection
                T = [-mapSize/2., X[1] + 1., X[2]]
            elif (math.pi/4 < euler[2] <= 3*math.pi/4):               #going in +Y -drection
                T = [X[0] - 1., -mapSize/2., X[2]]
            elif (-3*math.pi/4 < euler[2] <= -math.pi/4):              #going in -Y -drection
                T = [X[0] + 1., mapSize/2., X[2]]
            else:                                              #going in -X -drection
                T = [mapSize/2., X[1] - 1., X[2]]
            return T, targetPoint, 'Left', 'None'
    #return T, targetPoint, lastDirection
        
    elif (0.65 < rangeSensor < 3.):
        #targetPoint = [(X[0] + 0.45 + rangeSensor) * math.cos(euler[2]), X[1] * math.sin(euler[2]), X[2]]
        
        if (-math.pi/4 < euler[2] <= math.pi/4):                  #going in +X -drection
            T = [X[0] + 0.5, X[1], X[2]]
        elif (math.pi/4 < euler[2] <= 3*math.pi/4):               #going in +Y -drection
            T = [X[0], X[1] + 0.5, X[2]]
        elif (-3*math.pi/4 < euler[2] <= -math.pi/4):              #going in -Y -drection
            T = [X[0], X[1] - 0.5, X[2]]
        else:                                              #going in -X -drection
            T = [X[0] - 0.5, X[1], X[2]]
        
        return T, [-50,-50,-50], lastDirection, 'None'
    

    
    #SECOND ALGORITHM START
    
    elif (bump == 'area') and (lastBump != 'BumpDetected'):
        if (-math.pi/4 < euler[2] <= math.pi/4):                  #going in +X -drection
            T = [X[0] - 1., X[1] , X[2]]
        elif (math.pi/4 < euler[2] <= 3*math.pi/4):               #going in +Y -drection
            T = [X[0], X[1] - 1., X[2]]
        elif (- 3*math.pi/4 < euler[2] <= -math.pi/4):              #going in -Y -drection
            T = [X[0], X[1] + 1., X[2]]
        else:                                              #going in -X -drection
            T = [X[0] + 1., X[1], X[2]]
        
        return T, [-50,-50,-50], lastDirection, 'BumpDetected'
        
    elif (lastBump == 'BumpDetected'):
        
        if (lastDirection == 'Left' or lastDirection == 'None'):
            #update target to right
            if (-math.pi/4 < euler[2] <= math.pi/4):                  #going in +X -drection
                T = [-mapSize/2., X[1] - 2., X[2]]
            elif (math.pi/4 < euler[2] <= 3*math.pi/4):               #going in +Y -drection
                T = [X[0] + 2., -mapSize/2., X[2]]
            elif (-3*math.pi/4 < euler[2] <= -math.pi/4):              #going in -Y -drection
                T = [X[0] - 2., mapSize/2., X[2]]
            else:                                              #going in -X -drection
                T = [mapSize/2., X[1] + 2., X[2]]
            return T, [-50,-50,-50], 'Right', 'None'
                
        else:
            #update target to left
            if (-math.pi/4 < euler[2] <= math.pi/4):                  #going in +X -drection
                T = [-mapSize/2., X[1] + 2., X[2]]
            elif (math.pi/4 < euler[2] <= 3*math.pi/4):               #going in +Y -drection
                T = [X[0] - 2., -mapSize/2., X[2]]
            elif (-3*math.pi/4 < euler[2] <= -math.pi/4):              #going in -Y -drection
                T = [X[0] + 2., mapSize/2., X[2]]
            else:                                              #going in -X -drection
                T = [mapSize/2., X[1] - 2., X[2]]
            return T, [-50,-50,-50], 'Left', 'None'
    
    else:
        if (-math.pi/4 < euler[2] <= math.pi/4):                  #going in +X -drection
            T = [X[0] + 1., X[1], X[2]]
        elif (math.pi/4 < euler[2] <= 3*math.pi/4):               #going in +Y -drection
            T = [X[0], X[1] + 1., X[2]]
        elif (-3*math.pi/4 < euler[2] <= -math.pi/4):              #going in -Y -drection
            T = [X[0], X[1] - 1., X[2]]
        else:                                              #going in -X -drection
            T = [X[0] - 1., X[1], X[2]]
        return T, [-50,-50,-50], lastDirection, 'None'
    
    #SECOND ALGORITHM END
    
    
        #FIRST ALGORITHM START
    ''''''
    #USE THIS IF MAP LOOKS BETTER TO TURN RIGHT
    elif (bump == 'area') and (lastDirection == 'Left' or lastDirection == 'None'): # 1 step back and turn right
        
        if (-math.pi/4 < euler[2] <= math.pi/4):                  #going in +X -drection
            T = [X[0] ,X[1] - 1., X[2]]
        elif (math.pi/4 < euler[2] <= 3*math.pi/4):               #going in +Y -drection
            T = [X[0] + 1. ,X[1], X[2]]
        elif (- 3*math.pi/4 < euler[2] <= -math.pi/4):              #going in -Y -drection
            T = [X[0] - 1. ,X[1], X[2]]
        else:                                              #going in -X -drection
            T = [X[0], X[1] + 1., X[2]]
        
        return T, [-50,-50,-50], 'Right'
        
      #USE THIS IF MAP LOOKS BETTER TO TURN LEFT
    
    elif (bump == 'area'): # 1 step back and turn left
        
        if (-math.pi/4 < euler[2] <= math.pi/4):                  #going in +X -drection
            T = [X[0] ,X[1] + 1., X[2]]
        elif (math.pi/4 < euler[2] <= 3*math.pi/4):               #going in +Y -drection
            T = [X[0] - 1. ,X[1], X[2]]
        elif (- 3*math.pi/4 < euler[2] <= -math.pi/4):              #going in -Y -drection
            T = [X[0] + 1. ,X[1], X[2]]
        else:                                              #going in -X -drection
            T = [X[0], X[1] - 1., X[2]]
        
        return T, [-50,-50,-50], 'Left'
    
    else:
        if (-math.pi/4 < euler[2] <= math.pi/4):                  #going in +X -drection
            T = [X[0] + 1., X[1], X[2]]
        elif (math.pi/4 < euler[2] <= 3*math.pi/4):               #going in +Y -drection
            T = [X[0], X[1] + 1., X[2]]
        elif (-3*math.pi/4 < euler[2] <= -math.pi/4):              #going in -Y -drection
            T = [X[0], X[1] - 1., X[2]]
        else:                                              #going in -X -drection
            T = [X[0] - 1., X[1], X[2]]
        print 'CurrentUpdatedTarget : ',T
        return T, [-50,-50,-50], lastDirection
    #FIRST ALGORITHM END
    '''
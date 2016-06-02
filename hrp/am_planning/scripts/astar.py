# -*- coding: utf-8 -*-
"""
Created on Mon Sep 14 08:53:41 2015

@author: H585993
"""


#from __future__ import print_function
import numpy as np


from numpy import ceil, floor, argsort, zeros, sqrt, Inf, shape






def column(matrix, i):
    return [row[i] for row in matrix]

def column2(matrix1, i):
    if matrix1==[]:
        return [[-Inf,-Inf]]
    out = []
    for row in matrix1:
        out.append([row[i],row[i+1]])
    return out

def is_point_in_list(point,list,grid_size):
    for i in range(len(list)):
        #if list[i][0:2]==point: 
        if (abs(list[i][0]-point[0])<grid_size/50) and (abs(list[i][1]-point[1])<grid_size/50): 
            return True
    return False
    
    
def get_index_for_point_in_list(point,list,grid_size):
    index = -1
    for i in range(len(list)):
        #if list[i][0:2]==point: 
        if (abs(list[i][0]-point[0])<grid_size/50) and (abs(list[i][1]-point[1])<grid_size/50): 
            index = i
            break
    return index
    
    

def get_distance_transformation_matrix(lx,hx,ly,hy,grid_points_inside_area,boundary,max_distance_transformation_matrix,grid_size):
    
    #w0 w1 w2
    #w3 p
    
    #   p  w3
    #w0 w1 w2
    
    w = [4, 3, 4, 3]
    #w = [4, 3, 4, 3, 4]
    #w = [sqrt(2), 1, sqrt(2), 1]
    
    
    #distance_transformation_matrix = zeros([hx-lx+2,hy-ly+2])
    #lenx = int(max(column(boundary,0))/grid_size)
    #leny = int(max(column(boundary,1))/grid_size)
    minx = int(min(column(boundary,0))/grid_size)
    miny = int(min(column(boundary,1))/grid_size)
    lenx=hx-minx
    leny=hy-miny
    distance_transformation_matrix = zeros([lenx+1,leny+1])
    #distance_transformation_matrix = zeros([hx-lx,hy-ly])
    
    
    
    # Set all pixels not belonging to inner area to 0 and all inside to Inf.
    for jj in range(leny):
        for ii in range(lenx):
            point= ((float(ii)+minx)*grid_size,(float(jj)+miny)*grid_size)
            if is_point_in_list(point,grid_points_inside_area,grid_size) and not is_point_in_list(point,boundary,grid_size):
                distance_transformation_matrix[ii,jj] = Inf
    
    #Forward pass
    for ii in range(lenx):
        for jj in range(leny):
            if distance_transformation_matrix[ii,jj] > 0:  
                distance_transformation_matrix[ii,jj] = min(distance_transformation_matrix[ii-1,jj+1]+w[0], distance_transformation_matrix[ii,jj+1]+w[1], distance_transformation_matrix[ii+1,jj+1]+w[2], distance_transformation_matrix[ii-1,jj]+w[3])
                

    #Backward pass
    #for ii in range(lenx-1,-1,-1):
    for ii in range(lenx-1,-1,-1):
        #for jj in range(leny-1,-1,-1): 
        for jj in range(leny-1,-1,-1):   
            if distance_transformation_matrix[ii,jj] > 0:
                distance_transformation_matrix[ii,jj] = min(distance_transformation_matrix[ii-1,jj-1]+w[0], distance_transformation_matrix[ii,jj-1]+w[1], distance_transformation_matrix[ii+1,jj-1]+w[2], distance_transformation_matrix[ii+1,jj]+w[3],distance_transformation_matrix[ii,jj])
                

    # The algorithm does not say to do this once more (forward and backward pass), but I found the solution to be asymmetric, and this makes it symmetric. There are another way of doing this more efficiently, and that is to add w[4] = 4, but then the solution is kind of an approximation, and since I don't think this will be a problem computationally, I think this is the best way of doing things, doing the same once more but with w transposed. (Not the best explanation, but if in doubt, ask Mikaela ;) ) 

    #Forward pass2
    for jj in range(leny):
        for ii in range(lenx):
            if distance_transformation_matrix[ii,jj] > 0:  
                distance_transformation_matrix[ii,jj] = min(distance_transformation_matrix[ii-1,jj+1]+w[0], distance_transformation_matrix[ii-1,jj]+w[1], distance_transformation_matrix[ii-1,jj-1]+w[2], distance_transformation_matrix[ii,jj+1]+w[3],distance_transformation_matrix[ii,jj])
                                
                
    #Backward pass2
    for jj in range(leny-1,-1,-1):
        for ii in range(lenx-1,-1,-1):   
            if distance_transformation_matrix[ii,jj] > 0:
                distance_transformation_matrix[ii,jj] = min(distance_transformation_matrix[ii+1,jj+1]+w[0], distance_transformation_matrix[ii+1,jj]+w[1], distance_transformation_matrix[ii+1,jj-1]+w[2], distance_transformation_matrix[ii,jj-1]+w[3],distance_transformation_matrix[ii,jj])
    
                
                
                
    
    # Set the maximum value that will be set to the vertices. This basically means that all vertices inside the polygon at a certain distance from the boundary will be given the same low cost in A* later. 
    for ii in range(shape(distance_transformation_matrix)[0]):
        for jj in range(shape(distance_transformation_matrix)[1]):
            if distance_transformation_matrix[ii,jj] > max_distance_transformation_matrix:
                distance_transformation_matrix[ii,jj] = max_distance_transformation_matrix
    
    
    verts_close_to_boundary = []
    for ii in range(shape(distance_transformation_matrix)[0]):
        for jj in range(shape(distance_transformation_matrix)[1]):
            if distance_transformation_matrix[ii,jj] > 0 and distance_transformation_matrix[ii,jj] < max_distance_transformation_matrix:
                #point= (float(ii)*grid_size,float(jj)*grid_size)
                
                point= ((float(ii)+minx)*grid_size,(float(jj)+miny)*grid_size)
                #value = (distance_transformation_matrix[ii,jj]+1.0)**2 # 1 in middle of map, 4 outside. 
                value = (max_distance_transformation_matrix-distance_transformation_matrix[ii,jj])**1.7
                verts_close_to_boundary.append([point,value])
    
        
    
    
    return verts_close_to_boundary



def get_2cols_list(list_x,list_y):
    out = []
    for i in range(len(list_x)):
        out.append((list_x[i],list_y[i]))
    return out

    


def expand_node(currx,curry,frontier,boundary,visited,goal_found,verts_close_to_boundary,grid_size,goal,max_distance_transformation_matrix,g_cost,grid_points_inside_area):
    # Do not add this point to the frontier if it is an obstacle (or boundary, which is included in the set) or in the visited set
    #if not (abs(currx-goal[0])<0.01 and abs(curry-goal[1])<0.01) and (is_point_in_list((currx,curry),boundary,grid_size) or is_point_in_list((currx,curry),visited,grid_size)):# or currx>max(column(obstacle,0)) or currx<min(column(obstacle,0)) or curry>max(column(obstacle,1)) or curry<min(column(obstacle,1)): # The last four should not be needed, BUT I think something is wrong, so for now, I'm keeping it until I've figured it out...
    #    return
    if not (abs(currx-goal[0])<0.01 and abs(curry-goal[1])<0.01) and (is_point_in_list((currx,curry),visited,grid_size)):# or currx>max(column(obstacle,0)) or currx<min(column(obstacle,0)) or curry>max(column(obstacle,1)) or curry<min(column(obstacle,1)): # The last four should not be needed, BUT I think something is wrong, so for now, I'm keeping it until I've figured it out...
        return

    index_verts_close_to_boundary = get_index_for_point_in_list((currx,curry),column(verts_close_to_boundary,0),grid_size)
    if not index_verts_close_to_boundary==-1:
        cost = verts_close_to_boundary[index_verts_close_to_boundary][1]
    else:
        cost = 1.0
        
    if not is_point_in_list((currx,curry),grid_points_inside_area,grid_size) or is_point_in_list((currx,curry),boundary,grid_size):
        cost = cost*1000
        
    # cost is a value we multiply with g. cost has a higher value the closer the point (currx,curry) is to the boundary.
    # g_cost exists since we want to have a 8-connected grid. If the current point (currx,curry) is one of the 4 closest neighbors to the current position in A*, then g_cost equals 1. otherwise, of it is one of the other 4 neighbors, g_cost equals sqrt(2). 

    k=((np.array(column2(frontier,0)) == [currx,curry]))
    already_in_frontier = False
    temp_index=0
    for i in range(shape(k)[0]):
        if list(k[i])==[True,True]:
            already_in_frontier=True
            temp_index=i
            break
    if already_in_frontier: # if already in frontier
        # Update the f value:
        new_g = (visited[-1][2]+grid_size*g_cost*cost)
        new_f = new_g + frontier[temp_index][3]
        if new_f<frontier[temp_index][4]:
            frontier[temp_index][2] = new_g
            #frontier[temp_index][3] # h not changed
            frontier[temp_index][4] = new_f
            path = visited[-1][5][:]
            path.append((currx,curry))
            frontier[temp_index][5] = path
    else: # if point not in frontier
        if visited==[]:
            g = grid_size*cost*g_cost
            path = []
        else:
            g = (visited[-1][2]+grid_size*g_cost*cost)
            path = visited[-1][5][:]
        h = sqrt((currx-goal[0])**2+(curry-goal[1])**2)  # Our heuristic is the Euclidean distance. The heuristic has to be equal to or smaller than the true distance (multiplied by the highest possible cost in this case), but since the cost can be 1 (not affecting the g value), we have to take the euclidean distance. 
        f = g+h
        path.append((currx,curry))
        frontier.append([currx,curry,g,h,f,path])
    




def get_all_grid_points_inside_area(grid_size,boundary_verts,lx,hx,ly,hy):
    grid_points_inside_area = []
    
    for ii in range(lx,hx+1):
        for jj in range(ly,hy+1):
            
            point= (float(ii)*grid_size,float(jj)*grid_size)
            
            # Is point inside polygon?
            # => Ray casting algorithm    # Note: will not be correct if the point is on the border of the polygon
            num_intersections = 0
            for i in range(len(boundary_verts)-1):
            
                x1 = point[0]-0.1
                y1 = point[1]
                x2 = max(boundary_verts[i][0],boundary_verts[i+1][0])+0.1
                y2 = point[1]
                x3 = boundary_verts[i][0]
                y3 = boundary_verts[i][1]
                x4 = boundary_verts[i+1][0]
                y4 = boundary_verts[i+1][1]
                
                intersects = intersect((x1,y1),(x2,y2),(x3,y3),(x4,y4))                
                if intersects:
                    num_intersections += 1
                    
            if num_intersections%2:
                grid_points_inside_area.append((ii*grid_size,jj*grid_size))
    
    return grid_points_inside_area



    


def get_all_grid_points_that_intersects_boundary(grid_size,boundary_verts,lx,hx,ly,hy):
    
    boundary = []
    for ii in range(lx,hx+1):
        for jj in range(ly,hy+1):
            cell_intersects_with_boundary = False
    
            point= (float(ii)*grid_size,float(jj)*grid_size)
            
            corners = [(point[0]-grid_size/2, point[1]-grid_size/2), (point[0]+grid_size/2, point[1]-grid_size/2), (point[0]+grid_size/2, point[1]+grid_size/2), (point[0]-grid_size/2, point[1]+grid_size/2), (point[0]-grid_size/2, point[1]-grid_size/2)]
            
            for j in range(4):
                if cell_intersects_with_boundary:
                    break
                for i in range(len(boundary_verts)-1):
                    
                    x1 = boundary_verts[i][0]
                    y1 = boundary_verts[i][1]
                    x2 = boundary_verts[i+1][0]
                    y2 = boundary_verts[i+1][1]
                    x3 = corners[j+1][0]
                    y3 = corners[j+1][1]
                    x4 = corners[j][0]
                    y4 = corners[j][1]
                    
                    cell_intersects_with_boundary = intersect((x1,y1),(x2,y2),(x3,y3),(x4,y4))
                
                    if cell_intersects_with_boundary:
                        boundary.append(point)
                        cell_intersects_with_boundary=True
                        break
                        
    return boundary    
    
    
def ccw(A,B,C):
    return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])
def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)
        
        
        
def A_star(grid_size,avoid_border_val,boundary_verts,start,goal):
  
    # In case the user inputs an int
    grid_size = float(grid_size)
    
    #boundary_verts has to end with the same element as it starts with, so this is only a check to make sure that nothing is wrong! (maybe mikaela will forget to tell...)
    if not boundary_verts[-1] == boundary_verts[0]:
        boundary_verts_old = boundary_verts[:]
        boundary_verts = list(zeros(len(boundary_verts)+1))
        boundary_verts[0:-1] = boundary_verts_old[:]
        boundary_verts[-1] = boundary_verts[0]
      
    # move to closest grid point
    orig_start = start
    orig_goal = goal
    start = (round(start[0]/grid_size)*grid_size, round(start[1]/grid_size)*grid_size)
    goal = (round(goal[0]/grid_size)*grid_size, round(goal[1]/grid_size)*grid_size)
    
    
    # get highest and lowest x and y (computationally matter)
    lx = int(ceil(min(column(boundary_verts,0))/grid_size))
    hx = int(floor(max(column(boundary_verts,0))/grid_size))
    ly = int(ceil(min(column(boundary_verts,1))/grid_size))
    hy = int(floor(max(column(boundary_verts,1))/grid_size))
    
    lx -= 1
    hx += 1
    ly -= 1
    hy += 1
    
    
    # find which points are within boundary, used for distance transformation matrix
    grid_points_inside_area = get_all_grid_points_inside_area(grid_size,boundary_verts,lx,hx,ly,hy)
    
    # boundary contains all grid points that intersect with the boundary from boundary_verts
    boundary = get_all_grid_points_that_intersects_boundary(grid_size,boundary_verts,lx,hx,ly,hy)
    
    max_distance_transformation_matrix = avoid_border_val/grid_size # can be set to increase/decrease how close the path can be to boundary. OBS! The is another line in the code that affects this as well (Ask Mikaela..  (max_distance_transformation_matrix-distance_transformation_matrix[ii,jj])**x)... 
    verts_close_to_boundary = get_distance_transformation_matrix(lx,hx,ly,hy,grid_points_inside_area,boundary,max_distance_transformation_matrix,grid_size)
    
  
  
  
    # A* algorithm begins: 
  
    currx = start[0]
    curry = start[1]
    frontier = []
    visited = []
    goal_found = False
    goal_ind = 0
    
    path = []
    path.append((start[0],start[1]))
    frontier.append([start[0],start[1],0,sqrt((goal[0]-currx)**2+(goal[1]-curry)**2)*max_distance_transformation_matrix,sqrt((goal[0]-currx)**2+(goal[1]-curry)**2)*max_distance_transformation_matrix,path])
    
    
    while not frontier==[]:
        
        # Move the one node in the frontier with the lowest f value to the visited set.
        visited.append(frontier[argsort(column(frontier,4))[0]])  # Index of the node with the lowest f value: argsort(column(frontier,4))[0]
        frontier.pop(argsort(column(frontier,4))[0])
        currx = visited[-1][0]
        curry = visited[-1][1]
    
    
    
        # If goal found and it has a lower f than all nodes in the frontier, we are done and can return the path
        if (abs(visited[-1][0] - goal[0])<grid_size/10.0 and (abs(visited[-1][1] - goal[1])<grid_size/10.0)): # using this instead of "==" due to floating-point arithmetic (e.g. 1.1-0.001 != 1.099) 
            if not goal_found or (goal_found and visited[-1][4]<visited[goal_ind][4]):
                goal_found = True
                goal_ind = len(visited)-1
        if goal_found and (frontier==[] or (not frontier==[] and visited[goal_ind][4]<min(column(frontier,4)))):
            # We have found our path! 
            A_star_path = visited[goal_ind][5][:]    
            A_star_path[0] = orig_start
            A_star_path[-1] = orig_goal
            
            
            '''
            # Remove unneccessary points in A_star_path
            A_star_path_orig = A_star_path[:]
            
            best_A_star_path = A_star_path_orig[:]
            for kk in range(10):
                A_star_path = A_star_path_orig[:]
                
                intersects = False
                not_removed_count=0
                while True:
                    if not_removed_count>100 or len(A_star_path)<3:
                        break
                    i = int((len(A_star_path)-2)*np.random.rand())+1
                    
                    remove = True
                    
                    for j in range(len(verts_close_to_boundary)-1):
                        
                        x1 = A_star_path[i-1][0]
                        y1 = A_star_path[i-1][1]
                        x2 = A_star_path[i+1][0]
                        y2 = A_star_path[i+1][1]
                        x3 = verts_close_to_boundary[j][0][0]
                        y3 = verts_close_to_boundary[j][0][1]
                        x4 = verts_close_to_boundary[j+1][0][0]
                        y4 = verts_close_to_boundary[j+1][0][1]
                        
                        if distance(verts_close_to_boundary[j][0],verts_close_to_boundary[j+1][0])>grid_size*2:
                            continue
                        if intersect((x1,y1),(x2,y2),(x3,y3),(x4,y4)):
                            remove = False
                    
                    if remove:
                        A_star_path.pop(i)
                        not_removed_count = 0
                    else:
                        not_removed_count += 1 
                
                if len(A_star_path)<len(best_A_star_path):
                    best_A_star_path = A_star_path[:]
            
            
                    
            return best_A_star_path
            '''
            return A_star_path
                
                
        # Expand the current node (currx,curry), i.e. investigate which neighbors it has and add the new neighbors (nodes) to the frontier (or update the f value, etc for the node in question if it's already in the set) 
        # 4-connected grid
        expand_node(currx+grid_size,curry,frontier,boundary,visited,goal_found,verts_close_to_boundary,grid_size,goal,max_distance_transformation_matrix,1,grid_points_inside_area)
        expand_node(currx-grid_size,curry,frontier,boundary,visited,goal_found,verts_close_to_boundary,grid_size,goal,max_distance_transformation_matrix,1,grid_points_inside_area)
        expand_node(currx,curry+grid_size,frontier,boundary,visited,goal_found,verts_close_to_boundary,grid_size,goal,max_distance_transformation_matrix,1,grid_points_inside_area)
        expand_node(currx,curry-grid_size,frontier,boundary,visited,goal_found,verts_close_to_boundary,grid_size,goal,max_distance_transformation_matrix,1,grid_points_inside_area)
        # Add if 8-connected grid is desired 
        expand_node(currx+grid_size,curry+grid_size,frontier,boundary,visited,goal_found,verts_close_to_boundary,grid_size,goal,max_distance_transformation_matrix,sqrt(2),grid_points_inside_area)
        expand_node(currx+grid_size,curry-grid_size,frontier,boundary,visited,goal_found,verts_close_to_boundary,grid_size,goal,max_distance_transformation_matrix,sqrt(2),grid_points_inside_area)
        expand_node(currx-grid_size,curry+grid_size,frontier,boundary,visited,goal_found,verts_close_to_boundary,grid_size,goal,max_distance_transformation_matrix,sqrt(2),grid_points_inside_area)
        expand_node(currx-grid_size,curry-grid_size,frontier,boundary,visited,goal_found,verts_close_to_boundary,grid_size,goal,max_distance_transformation_matrix,sqrt(2),grid_points_inside_area)
        
        
        
    return [(start[0],start[1])]



def distance(p1,p2):
    return sqrt((p1[0]-p2[0])**2.0+(p1[1]-p2[1])**2.0)
    


if __name__ == "__main__":
    
    import matplotlib.pyplot as plt
    grid_size = 0.4
 
    boundary_verts = [(2, 16),
                     (2, 4),
                     (7, 4),
                     (7, 8),
                     (12, 8),
                     (12, 4),
                     (18, 4),
                     (18, 16),
                     (12, 16),
                     (12, 12),
                     (7, 12),
                     (7, 16)]
    
        
    boundary_verts = [(2, 8),
                     (2, -4),
                     (7, -4),
                     (7, 0),
                     (12, 0),
                     (12, -4),
                     (18, -4),
                     (18, 8),
                     (12, 8),
                     (12, 4),
                     (7, 4),
                     (7, 8)]
    
    
    boundary_verts = [(-2, 8),
                     (-2, -4),
                     (3, -4),
                     (3, 0),
                     (8, 0),
                     (8, -4),
                     (14, -4),
                     (14, 8),
                     (8, 8),
                     (8, 4),
                     (3, 4),
                     (3, 8)]
    
    start = (5.1, 14.0)
    goal = (15.5, 14.5)
    goal = (15.2, 15.6)
    
    
    start = (5.1, 6.0)
    goal = (15.2, 7.6)
    
    start = (1.1, 6.0)
    goal = (11.2, 7.6)
    #A_star_path = A_star(grid_size,boundary_verts,start,goal)
    
    
    
    
    

    # In case the user inputs an int
    grid_size = float(grid_size)
    
    #boundary_verts has to end with the same element as it starts with, so this is only a check to make sure that nothing is wrong! (maybe mikaela will forget to tell...)
    if not boundary_verts[-1] == boundary_verts[0]:
        boundary_verts_old = boundary_verts[:]
        boundary_verts = list(zeros(len(boundary_verts)+1))
        boundary_verts[0:-1] = boundary_verts_old[:]
        boundary_verts[-1] = boundary_verts[0]
        
    # Move to closest grid point    
    #for i in range(len(boundary_verts)):
    #    boundary_verts[i] = (round(boundary_verts[i][0]/grid_size)*grid_size, round(boundary_verts[i][1]/grid_size)*grid_size)
        
        
        
    # get highest and lowest x and y (computationally matter)
    lx = int(ceil(min(column(boundary_verts,0))/grid_size))
    hx = int(floor(max(column(boundary_verts,0))/grid_size))
    ly = int(ceil(min(column(boundary_verts,1))/grid_size))
    hy = int(floor(max(column(boundary_verts,1))/grid_size))
    
    lx -= 1
    hx += 1
    ly -= 1
    hy += 1
    
    
    # find which points are within boundary, used for distance transformation matrix
    grid_points_inside_area = get_all_grid_points_inside_area(grid_size,boundary_verts,lx,hx,ly,hy)
    
    # boundary contains all grid points that intersect with the boundary from boundary_verts
    boundary = get_all_grid_points_that_intersects_boundary(grid_size,boundary_verts,lx,hx,ly,hy)
    
    
    avoid_border_val = 4
    max_distance_transformation_matrix = avoid_border_val/grid_size # can be set to increase/decrease how close the path can be to boundary. OBS! The line below, (1.0+distance_transformation_matrix)**2, affects this as well... 
    verts_close_to_boundary = get_distance_transformation_matrix(lx,hx,ly,hy,grid_points_inside_area,boundary,max_distance_transformation_matrix,grid_size)
    
    A_star_path = A_star(grid_size,avoid_border_val,boundary_verts,start,goal)
    
    
    # move to closest grid point
    start = (round(start[0]/grid_size)*grid_size, round(start[1]/grid_size)*grid_size)
    goal = (round(goal[0]/grid_size)*grid_size, round(goal[1]/grid_size)*grid_size)
    
    
  
    plt.figure()
    for i in range(len(verts_close_to_boundary)):
        #plt.scatter(verts_close_to_boundary[i][0][0],verts_close_to_boundary[i][0][1],s=10)
        plt.scatter(verts_close_to_boundary[i][0][0],verts_close_to_boundary[i][0][1],s=verts_close_to_boundary[i][1])
        
    for i in range(len(A_star_path)):
        plt.scatter(A_star_path[i][0],A_star_path[i][1],s=30,c='r',alpha=0.5)
    
    
    for i in range(len(boundary_verts)):
        plt.scatter(boundary_verts[i][0],boundary_verts[i][1],s=50,c='g',alpha=1)
    
    
    for i in range(len(grid_points_inside_area)):
        plt.scatter(grid_points_inside_area[i][0],grid_points_inside_area[i][1],s=10,c='g',alpha=0.1)
    
    
    for i in range(len(boundary)):
        plt.scatter(boundary[i][0],boundary[i][1],s=10,c='k',alpha=0.3)
    
    
    

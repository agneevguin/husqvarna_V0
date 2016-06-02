# -*- coding: utf-8 -*-
"""
Created on Mon Apr 11 16:36:04 2016
@author: ermias
"""
import math
import sys


class Cell:
    def __init__(self, pos=[0., 0.], obstacle=False, wall=None, wall_probability=0.):
        # Lower right corner of the cell
        self.pos = pos
        self.obstacle = obstacle
        self.wall = wall
        self.wall_prob = wall_probability
        self.covered = False
        self.occupied = False


class DiscreteMap:
    def __init__(self, X=50., Y=50., cell_size=1, buffer_size=10):
        self.X = int(X - buffer_size)
        self.Y = int(Y - buffer_size)
        self.cell_size = cell_size
        self.trees = []
        # Matrix = [[Cell() for x in range(5)] for x in range(5)]
        self.cells = [[Cell() for y in range(self.X +1 )] for x in range(self.Y+1)]
        self.cells_flat = []
        for x in xrange(0, self.X + 1, cell_size):
            for y in xrange(0, self.Y + 1, cell_size):
                self.cells[x][y] = Cell(pos=[x - self.X / 2, self.Y / 2 - y])
                self.cells_flat.append(Cell(pos=[x - self.X / 2, self.Y / 2 - y]))

    def get_tree(self, robots):
        trees = {}
        robot_positions = {}
        for r in robots:
            trees[r.name] = []
            robot_positions[r.name] = [r.X[0], r.X[1]]
        no_moves = False
        while not no_moves:
            no_moves = True
            for r in robot_positions:
                possible_moves = self.possible_next_cell([robot_positions[r][0], robot_positions[r][1]])
                max_dist = -(sys.maxint - 1)
                cell = None
                for i in possible_moves:
                    for rb in robot_positions:
                        if rb != r:
                            if self.dist(robot_positions[rb], i.pos) > max_dist:
                                max_dist = self.dist(robot_positions[rb], i.pos)
                                cell = i
                if cell:
                    no_moves = False
                    self.get_cell(cell.pos).occupied = True
                    trees[r].append([cell.pos[0], cell.pos[1], 0.])
                    robot_positions[r] = cell.pos
        return trees

    def all_cells_occupied(self):
        for c in self.cells_flat:
            if not c.occupied:
                return False
        return True

    def get_cell(self, pos=[0., 0.]):
        x = int(pos[0])
        y = int(pos[1])
        #print 'x,y = {}, {}'.format(x, y)
        #print 'x,y = {}, {}  index = {}, {}'.format(x, y, x + self.X / 2, self.Y / 2 - y)
        if (x + self.X / 2) > self.X or (self.Y / 2 - y) > self.Y:
            return None
        return self.cells[x + self.X / 2][self.Y / 2 - y]

    def get_unoccupied_up(self, pos):
        y = int(pos[1])
        for i in range(self.Y - y):
            up = self.get_cell([pos[0], pos[1] + self.cell_size + i])
            if not up.self.occupied:
                return up, self.cell_size + i

    def get_unoccupied_down(self, pos):
        y = int(pos[1])
        for i in range(self.Y - y):
            up = self.get_cell([pos[0], pos[1] - self.cell_size + i])
            if not up.self.occupied:
                return up

    def get_uncovered_cell(self, pos=[0., 0.]):
        min_dist = sys.maxint
        for c in self.cells:
            dist = self.dist(c, pos)
            if not c.covered and dist < min_dist:
                cell = c
                min_dist = dist
        return cell

    def possible_next_cell(self, pos):
        up = self.get_cell([pos[0], pos[1] + self.cell_size])
        down = self.get_cell([pos[0], pos[1] - self.cell_size])
        left = self.get_cell([pos[0] - self.cell_size, pos[1]])
        right = self.get_cell([pos[0] + self.cell_size, pos[1]])
        list = []
        if up and not up.occupied:
            list.append(up)
        if down and not down.occupied:
            list.append(down)
        if left and not left.occupied:
            list.append(left)
        if right and not right.occupied:
            list.append(right)

        return list

    def dist(self, point1, point2):
        return math.sqrt(math.pow((point1[1] - point2[1]), 2) + math.pow((point1[0] - point2[0]), 2))
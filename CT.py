import numpy as np
import time
import matplotlib.pyplot as plt
import random
from heapq import heappop, heappush

from MAPF import Map
from MAPF import read_map_from_movingai_file, read_tasks_from_movingai_file

class HighNode:
    '''
    HighNode class represents a high-level search node

    - vertexCons: vertex constraints of the node
    - edgeCons: edge constraints of the node
    - sol: solution of the node
    - g: cost of the solution of the node 
    - h: h-value of the node
    - F: f-value of the node
    - parent: pointer to the parent-node 

    '''

    def __init__(self, vertexCons={}, edgeCons={}, sol={}, g=0, h=0, F=None, parent=None, k=0):
        self.vertexCons = vertexCons
        self.edgeCons = edgeCons
        self.sol = sol
        self.g = g
        self.h = h
        self.k = k
        if F is None:
            self.F = g + h
        else:
            self.F = F        
        self.parent = parent
    
    
    def __eq__(self, other):
        return (self.vertexCons == other.vertexCons) and (self.edgeCons == other.edgeCons) and \
               (self.sol == other.sol)
    
    def __lt__(self, other):
        return self.F < other.F or ((self.F == other.F) and (self.h < other.h)) \
        or ((self.F == other.F) and (self.h == other.h))
        
        
        
class LowNode:
    '''
    LowNode class represents a low-level search node

    - i, j: coordinates of corresponding grid element
    - g: g-value of the node 
    - h: h-value of the node
    - F: f-value of the node
    - parent: pointer to the parent-node 
    '''

    def __init__(self, coord, g=0, h=0, F=None, parent=None):
        self.i = coord[0]
        self.j = coord[1]
        self.g = g
        self.h = h
        if F is None:
            self.F = g + h
        else:
            self.F = F        
        self.parent = parent
    
    
    def __eq__(self, other):
        return (self.i == other.i) and (self.j == other.j) and (self.g == other.g)
    
    def __lt__(self, other):
        return self.F < other.F or ((self.F == other.F) and (self.h < other.h)) \
        or ((self.F == other.F) and (self.h == other.h))
        

def MakePath(goal):
    '''
    Creates a path by tracing parent pointers from the goal node to the start node
    It also returns path's length.
    '''

    length = goal.g
    current = goal
    path = []
    while current.parent:
        path.append(current)
        current = current.parent
    path.append(current)
    return path[::-1], length
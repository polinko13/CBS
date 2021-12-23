import numpy as np
import time
import matplotlib.pyplot as plt
import random
from heapq import heappop, heappush

from MAPF import Map
from MAPF import read_map_from_movingai_file, read_tasks_from_movingai_file

from CT import HighNode, LowNode, MakePath
from OpenClosed import OpenHigh, OpenLow, ClosedLow

from Heuristics import ManhattanDistance

class AstarTimesteps:
    def __init__(self, gridMap, iStart, jStart, iGoal, jGoal, vertexCons, edgeCons):
        self.vertexCons = vertexCons
        self.edgeCons = edgeCons
        self.CLOSED = ClosedLow()
        self.gridMap = gridMap
        self.iStart = iStart
        self.jStart = jStart
        self.iGoal = iGoal
        self.jGoal = jGoal
        self.OPEN = OpenLow()
        self.path = []
        
    def CheckMove(self, i1, j1, i2, j2, t):
        for obs in self.vertexCons:
            if obs[1] == t + 1 and obs[0] == (i2, j2):
                return False
        
        for obs in self.edgeCons:
            if obs[2] == t + 1 and obs[0] == (i1, j1) and obs[1] == (i2, j2):
                return False
                
        return True
    
    def FindPath(self):
        startNode = LowNode(coord=(self.iStart, self.jStart))
        self.OPEN.AddNode(startNode)
    
        while not self.OPEN.isEmpty():
            s = self.OPEN.GetBestNode(self.CLOSED) 
            self.CLOSED.AddNode(s)       
            if s.i == self.iGoal and s.j == self.jGoal:
                return (True, s, self.OPEN, self.CLOSED)
            for nbr in self.gridMap.GetNeighbors(s.i, s.j):
                if self.CheckMove(s.i, s.j, nbr[0], nbr[1], s.g) and \
                not self.CLOSED.WasExpanded(LowNode(coord=nbr, g=s.g + 1)):
                    nbrNode = LowNode(coord=nbr, g=s.g + 1, h=ManhattanDistance(nbr[0], nbr[1], self.iGoal, self.jGoal), \
                                      parent=s)
                    self.OPEN.AddNode(nbrNode)
        return (False, None, self.OPEN, self.CLOSED)
import numpy as np
import time
import matplotlib.pyplot as plt
import random
from heapq import heappop, heappush

from MAPF import Map
from MAPF import read_map_from_movingai_file, read_tasks_from_movingai_file

from CT import HighNode, LowNode

class OpenHigh:
    
    def __init__(self):
        self.heap = []
    
    
    def __iter__(self):
        return iter(self.heap)
    
    
    def AddNode(self, node : HighNode):
        heappush(self.heap, node)

        
    def GetBestNode(self):        
        best = heappop(self.heap)
        
        return best
    
class OpenLow:
    
    def __init__(self):
        self.heap = []
        self.elements = {}
    
    def __iter__(self):
        return iter(self.elements.values())
    
    def __len__(self):
        return len(self.elements)

    def isEmpty(self):
        if len(self.elements) != 0:
            return False
        return True

    def AddNode(self, node : LowNode, *args):
        if self.elements.get((node.i, node.j, node.g)) is None or node < self.elements[(node.i, node.j, node.g)]:
            self.elements[(node.i, node.j, node.g)] = node
            heappush(self.heap, node)
        return

    def GetBestNode(self, CLOSED, *args):
        best = heappop(self.heap)    
        while CLOSED.WasExpanded(best):
            best = heappop(self.heap)
        del self.elements[(best.i, best.j, best.g)]
        return best
    
class ClosedLow:
    
    def __init__(self):
        self.elements = {}


    def __iter__(self):
        return iter(self.elements.values())
    

    def __len__(self):
        return len(self.elements)


    def AddNode(self, node : LowNode):
        self.elements[(node.i, node.j, node.g)] = node


    def WasExpanded(self, node : LowNode):
        return not self.elements.get((node.i, node.j, node.g)) is None
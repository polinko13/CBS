import numpy as np
import time
import matplotlib.pyplot as plt
import random
from heapq import heappop, heappush

'''Grid'''
class Map:

    # 
    def __init__(self):
        '''
        Default constructor
        '''

        self.width = 0
        self.height = 0
        self.cells = []
    

    def SetGridCells(self, width, height, gridCells):
        '''
        Initialization of map by list of cells.
        '''
        self.width = width
        self.height = height
        self.cells = gridCells


    def inBounds(self, i, j):
        '''
        Check if the cell is on a grid.
        '''
        return (0 <= i < self.width) and (0 <= j < self.height)
    

    def Traversable(self, i, j):
        '''
        Check if the cell is not an obstacle.
        '''
        return not self.cells[j][i]


    def GetNeighbors(self, i, j):
        '''
        Get a list of neighbouring cells as (i,j) tuples.
        '''   
        neighbors = [[i, j]]
        delta = [[0, 1], [1, 0], [0, -1], [-1, 0]]

        for d in delta:
            if self.inBounds(i + d[0], j + d[1]) and self.Traversable(i + d[0], j + d[1]):
                neighbors.append((i + d[0], j + d[1]))
        return neighbors
    
    
def read_map_from_movingai_file(path):
    map_file = open(path)
    count = 0
    name = map_file.readline()
    height = int(map_file.readline().split()[1])
    width = int(map_file.readline().split()[1])
    type_ = map_file.readline()
    cells = [[0 for _ in range(width)] for _ in range(height)]
    
    i = 0
    j = 0

    for l in map_file:
        j = 0
        for c in l:
            if c == '.':
                cells[i][j] = 0
            elif c == 'T' or c == '@':
                cells[i][j] = 1
            else:
                continue
            
            j += 1
            
        if j != width:
            raise Exception("Size Error. Map width = ", j, ", but must be", width, "(map line: ", i, ")")
                
        i += 1
        if(i == height):
            break
    
    return (width, height, cells)

def read_tasks_from_movingai_file(path):
    tasks = []
    task_file = open(path)
    for l in task_file:
        new_task = l.split()[4:]
        if len(new_task) != 0:
            tasks.append(list(map(float,new_task)))
    #возвращает числа: координаты начала и конца, длину пути
    return np.array(tasks, int)


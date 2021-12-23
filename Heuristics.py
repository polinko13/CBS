import numpy as np
import time
import matplotlib.pyplot as plt
import random
from heapq import heappop, heappush

def ManhattanDistance(i1, j1, i2, j2):
    return abs(i1 - i2) + abs(j1 - j2)
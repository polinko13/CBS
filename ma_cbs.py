import numpy as np
import time
import matplotlib.pyplot as plt
import random
from heapq import heappop, heappush

from MAPF import Map
from MAPF import read_map_from_movingai_file, read_tasks_from_movingai_file

from CT import HighNode, LowNode

from OpenClosed import OpenHigh, OpenLow, ClosedLow

from Heuristics import ManhattanDistance

from CT import MakePath
from AstarTimeSteps import AstarTimesteps


class MetaAgent:
    '''
    MetaAgent class represents a meta-agent
    
    - agents: set of agents of the meta-agent
    '''
    
    def __init__(self, agents):
        self.agents = agents
        
    def __eq__(self, other):
        return self.agents == other.agents
    
    def __repr__(self):   
        return 'Meta-agent consists of №{} agents'.format(" ".join([str(item) for item in self.agents]))
    
    def __hash__(self):
        return hash(repr(self))
    
    
def MAConflict(node: HighNode, CM, a : MetaAgent, b : MetaAgent):
    vertexConflicts = []
    edgeConflicts = []
    
    for x in a.agents:
        for y in b.agents:
            for step in range(min(len(node.sol[a][x]), len(node.sol[b][y]))):
                if node.sol[a][x][step].i == node.sol[b][y][step].i and node.sol[a][x][step].j == node.sol[b][y][step].j:
                    CM[x][y] += 1
                    CM[y][x] += 1
                    vc = (a, b, (node.sol[a][x][step].i, node.sol[a][x][step].j), step)
                    if vc not in vertexConflicts:
                        vertexConflicts.append(vc)
                if step + 1 < min(len(node.sol[a][x]), len(node.sol[b][y])) and \
                   node.sol[a][x][step].i == node.sol[b][y][step + 1].i and node.sol[a][x][step].j == node.sol[b][y][step + 1].j \
                   and node.sol[a][x][step + 1].i == node.sol[b][y][step].i and node.sol[a][x][step + 1].i == node.sol[b][y][step].i:
                    CM[x][y] += 1
                    CM[y][x] += 1
                    ec = (
                        a,
                        b,
                        (node.sol[a][x][step].i, node.sol[a][x][step].j),
                        (node.sol[a][x][step + 1].i, node.sol[a][x][step + 1].j),
                        step,
                    )
                    if ec not in edgeConflicts:
                        edgeConflicts.append(ec)
                        
    return CM, vertexConflicts, edgeConflicts

def CheckMerge(CM, Border, a: MetaAgent, b : MetaAgent):
    cnt = 0
    
    for x in a.agents:
        for y in b.agents:
            cnt += CM[x][y]
            
    return cnt > Border

def Merge(a : MetaAgent, b : MetaAgent, node : HighNode):
    newMetaAgent = MetaAgent(a.agents + b.agents)
    newVertexCons = {}
    newEdgeCons = {}
        
    # удаляем vertex internal constraints
    for ma in node.vertexCons:
        if ma == a or ma == b:
            newVC = []
            for vc in node.vertexCons[ma]:
                if vc[-1] != a and vc[-1] != b:
                    # vertex constraint теперь выглядит так: (subset, v, t, meta-agent)
                    new_vc = (ma.agents, vc[2], vc[3], vc[4])
                    newVC.append(new_vc)
            newVertexCons[ma] = newVC
        else:
            newVertexCons[ma] = node.vertexCons[ma]
    
    # удаляем edge internal constraints
    for ma in node.edgeCons:
        if ma == a or ma == b:
            newEC = []
            for ec in node.edgeCons[ma]:
                if ec[-1] != a and ec[-1] != b:
                    # edge constraint теперь выглядит так: (subset, v1, v2, t, meta-agent)
                    new_ec = (ma.agents, ec[1], ec[2], ec[3], ec[4])
                    newEC.append(new_ec)
            newEdgeCons[ma] = newEC
        else:
            newEdgeCons[ma] = node.edgeCons[ma]
            
    
    return newMetaAgent, newVertexCons, newEdgeCons

def MACBS(gridMap, Starts, Goals, Border, isMR=False):
    tic = time.perf_counter() # начало работы функции
    
    gen = 0
    exp = 0
    
    root = HighNode(vertexCons={}, edgeCons={}, sol={}, k=gen)
    OPEN = OpenHigh()
    agents = [MetaAgent([i]) for i in range(len(Starts))]
    CM = [[0] * len(agents)] * len(agents) # матрица конфликтов
    
    for a in range(len(agents)):
        planner = AstarTimesteps(gridMap, Starts[a][0], Starts[a][1], Goals[a][0], Goals[a][1], [], [])
        res = planner.FindPath()
        if res[0]:
            path = MakePath(res[1])[0]
            root.sol[agents[a]] = {}
            root.sol[agents[a]][a] = path
            root.g += len(path)
        else:
            return (False, None, gen, exp)
    
    root.agents = agents.copy()
    OPEN.AddNode(root)
    gen += 1

    toc = time.perf_counter()
    
    while toc - tic < 60: 
        s = OPEN.GetBestNode()
        exp += 1

        newVertexCons = []
        newEdgeCons = []
        
        wasMerged = False
        
        # обновление CM и поиск конфликтов
        for i, a in enumerate(s.agents):
            for b in s.agents[i + 1 :]:
                
                CM, vc, ec = MAConflict(s, CM, a, b)
                                
                if len(vc) > 0:
                    newVertexCons += vc
                    
                if len(ec) > 0:
                    newEdgeCons += ec
                
        if len(newVertexCons) == 0 and len(newEdgeCons) == 0:
            return (True, s, gen, exp)
        
        for i, a in enumerate(s.agents, 0):
            for b in s.agents[i + 1:]:  
                # проверяем, что еще не сливали и что можно кого-то слить
                if not wasMerged and CheckMerge(CM, Border, a, b):
                    wasMerged = True
                    
                    N = HighNode(
                    vertexCons={},
                    edgeCons={},
                    sol=s.sol.copy(),
                    parent=s.parent,
                    k=gen,
                    )
                    
                    N.agents = s.agents.copy()
                    
                    MA, newVC, newEC = Merge(a, b, s)  
                    
                    # удаляем сливаемых агентов
                    N.agents.remove(a)
                    N.agents.remove(b)
                    
                    # сохраняем косты их решений
                    cost_a = sum([len(path) for path in s.sol[a].values()])
                    del N.sol[a]
                    
                    cost_b = sum([len(path) for path in s.sol[b].values()])
                    del N.sol[b]
                    
                    # добавляем нового мета-агента
                    N.agents.append(MA)
                    
                    # обновляем constraints
                    N.vertexCons = newVC
                    N.edgeCons = newEC
                        
                    # выделяем constraints, касающиеся нового мета-агента
                    VC = []
                    EC = []
                    if MA in newVC:
                        VC = newVC[MA]
                    if MA in newEC:
                        EC = newEC[MA]
                        
                    # если MR, то начинаем поиск заново
                    if isMR:
                        root = HighNode(vertexCons={}, edgeCons={}, sol={}, g=0, k=gen)
                        OPEN = OpenHigh()
                        agents = N.agents.copy()
                        CM = [[0] * len(Starts)] * len(Starts) # матрица конфликтов
    
                        for a in agents:
                            res = CBS(gridMap, Starts, Goals, a.agents, [], [])
                            if res[0]:
                                root.sol[a] = res[1].sol
                                root.g += res[1].g
                            else:
                                return (False, None, gen, exp)
                                
                        root.agents = agents.copy()
                        OPEN.AddNode(root)
                        gen += 1
                        
                        toc = time.perf_counter()
                                
                    else:
                        result = CBS(gridMap, Starts, Goals, MA.agents, VC, EC)
                        if result[0]:
                            N.sol[MA] = result[1].sol
                            N.g = s.g - cost_a - cost_b + result[1].g
                            OPEN.AddNode(N)    
                            gen += 1
                
        # если не сливали, то делаем branching
        if not wasMerged:
            
            # Сейчас сначала разрешаются вершинные конфликты, потом реберные
            if len(newVertexCons) > 0:
                a, b, (i, j), t = newVertexCons[0]
            
                # Разбиваем CT на ноды A и B, разрешая вершинный конфликт 
            
                tmp = s.vertexCons.copy()
                if a in tmp:
                    tmp[a].append((a.agents, (i, j), t, b))   
                else:
                    tmp[a] = [(a.agents, (i, j), t, b)]
            
                A = HighNode(
                    vertexCons=tmp,
                    edgeCons=s.edgeCons.copy(),
                    sol=s.sol.copy(),
                    parent=s,
                    k=gen,
                )
                
                A.agents = s.agents.copy()
            
                ec = []
                if a in A.edgeCons:
                    ec = A.edgeCons[a]
            
                res = CBS(gridMap, Starts, Goals, a.agents, A.vertexCons[a], ec)
                if res[0]:
                    cost_a = 0
                    if a in s.sol:
                        cost_a = sum([len(path) for path in s.sol[a].values()])
                        
                    A.sol[a] = res[1].sol                    
                    A.g = s.g - cost_a + res[1].g
                    OPEN.AddNode(A)
                    gen += 1
                
                tmp = s.vertexCons.copy()
                if b in tmp:
                    tmp[b].append((b.agents, (i, j), t, a))   
                else:
                    tmp[b] = [(b.agents, (i, j), t, a)]
                
                B = HighNode(
                    vertexCons=tmp,
                    edgeCons=s.edgeCons.copy(),
                    sol=s.sol.copy(),
                    parent=s,
                    k=gen,
                )
                
                B.agents = s.agents.copy()
            
                ec = []
                if b in B.edgeCons:
                    ec = B.edgeCons[b]
                
                res = CBS(gridMap, Starts, Goals, b.agents, B.vertexCons[b], ec)
                if res[0]:
                    cost_b = 0
                    if b in s.sol:
                        cost_b = sum([len(path) for path in s.sol[b].values()])
                        
                    B.sol[b] = res[1].sol                    
                    B.g = s.g - cost_b + res[1].g
                    OPEN.AddNode(B)
                    gen += 1
                    
            if len(newEdgeCons) > 0:
                a, b, (i1, j1), (i2, j2), t = newEdgeCons[0]
            
                # Разбиваем CT на ноды A и B, разрешая вершинный конфликт 
            
                tmp = s.edgeCons.copy()
                if a in tmp:
                    tmp[a].append((a.agents, (i1, j1), (i2, j2), t, b))   
                else:
                    tmp[a] = [(a.agents, (i1, j1), (i2, j2), t, b)]
            
                A = HighNode(
                    vertexCons=s.vertexCons.copy(),
                    edgeCons=tmp,
                    sol=s.sol.copy(),
                    parent=s,
                    k=gen,
                )
                
                A.agents = s.agents.copy()
                            
                vc = []
                if a in A.vertexCons:
                    vc = A.vertexCons[a]
            
                res = CBS(gridMap, Starts, Goals, a.agents, vc, A.edgeCons[a])
                if res[0]:
                    cost_a = 0
                    if a in s.sol:
                        cost_a = sum([len(path) for path in s.sol[a].values()])
                        
                    A.sol[a] = res[1].sol                    
                    A.g = s.g - cost_a + res[1].g
                    OPEN.AddNode(A)
                    gen += 1
                
                tmp = s.edgeCons.copy()
                if b in tmp:
                    tmp[b].append((b.agents, (i1, j1), (i2, j2), t, a))   
                else:
                    tmp[b] = [(b.agents, (i1, j1), (i2, j2), t, a)]
                
                B = HighNode(
                    vertexCons=s.vertexCons.copy(),
                    edgeCons=tmp,
                    sol=s.sol.copy(),
                    parent=s,
                    k=gen,
                )
                
                B.agents = s.agents.copy()
                            
                vc = []
                if b in B.vertexCons:
                    vc = B.vertexCons[b]
                
                res = CBS(gridMap, Starts, Goals, b.agents, vc, B.edgeCons[b])
                if res[0]:
                    cost_b = 0
                    if b in s.sol:
                        cost_b = sum([len(path) for path in s.sol[b].values()])
                        
                    B.sol[b] = res[1].sol                    
                    B.g = s.g - cost_b + res[1].g
                    OPEN.AddNode(B)
                    gen += 1
                    
        toc = time.perf_counter()
                    
    return (False, None, gen, exp)
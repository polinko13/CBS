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

def PC_CBS(gridMap, Starts, Goals, subset=[], vertexCons=[], edgeCons=[]): # subset, vertex/edge- Cons - нужны только при вызове CBS в качестве нижнего уровня MACBS
    tic = time.perf_counter() # начало работы функции
    
    gen = 0
    exp = 0
    
    root = HighNode(vertexCons={}, edgeCons={}, sol={}, k=gen)
    OPEN = OpenHigh()
    agents = list(range(len(Starts)))
    if len(subset) > 0:
        agents = subset.copy()
    
    for vc in vertexCons:
        # перебираем агентов из подмножества
        for a in vc[0]:
            if a in root.vertexCons:
                root.vertexCons[a].append((vc[1], vc[2]))
            else:
                root.vertexCons[a] = [(vc[1], vc[2])]
                
    for ec in edgeCons:
        # перебираем агентов из подмножества
        for a in ec[0]:
            if a in root.edgeCons:
                root.edgeCons[a].append((ec[1], ec[2], ec[3]))
            else:
                root.edgeCons[a] = [(ec[1], ec[2], ec[3])]
                
    for a in agents:
        VC = []
        EC = []
        if a in root.vertexCons:
            VC = root.vertexCons[a]
        if a in root.edgeCons:
            EC = root.edgeCons[a]
            
        planner = AstarTimesteps(gridMap, Starts[a][0], Starts[a][1], Goals[a][0], Goals[a][1], VC, EC)
        res = planner.FindPath()
        if res[0]:
            path = MakePath(res[1])
            root.sol[a], _ = path
        else:
            return (False, None, gen, exp)
    
    root.g = sum([len(path) for path in root.sol.values()])
    OPEN.AddNode(root)
    gen += 1
    
    toc = time.perf_counter()
    
    while toc - tic < 120:
        s = OPEN.GetBestNode()
        exp += 1
        
        newVertexCons = []
        newEdgeCons = []
        
        IsCardinal = False
        index = 0
        
        for i, a in enumerate(agents):
            for b in agents[i + 1 :]:
                for step in range(min(len(s.sol[a]), len(s.sol[b]))):
                    if s.sol[a][step].i == s.sol[b][step].i and s.sol[a][step].j == s.sol[b][step].j:
                        newVertexCons.append((a, b, (s.sol[a][step].i, s.sol[a][step].j), step))
                    if step + 1 < min(len(s.sol[a]), len(s.sol[b])) and \
                       s.sol[a][step].i == s.sol[b][step + 1].i and s.sol[a][step].j == s.sol[b][step + 1].j and \
                       s.sol[a][step + 1].i == s.sol[b][step].i and s.sol[a][step + 1].i == s.sol[b][step].i:
                        newEdgeCons.append((
                            a,
                            b,
                            (s.sol[a][step].i, s.sol[a][step].j),
                            (s.sol[a][step + 1].i, s.sol[a][step + 1].j),
                            step,
                        ))
                        
        if len(newVertexCons) == 0 and len(newEdgeCons) == 0:
            return (True, s, gen, exp)
        
        # Сейчас сначала разрешаются вершинные конфликты, потом реберные
        if len(newVertexCons) > 0:
            while not IsCardinal:
                a, b, (i, j), t = newVertexCons[0]
                
                FlagChild1 = False
                FlagChild2 = False
            
                # Разбиваем CT на ноды A и B, разрешая вершинный конфликт 
            
                tmp = s.vertexCons.copy()
                if a in tmp:
                    tmp[a].append(((i, j), t))   
                else:
                    tmp[a] = [((i, j), t)]
            
                A = HighNode(
                    vertexCons=tmp,
                    edgeCons=s.edgeCons.copy(),
                    sol=s.sol.copy(),
                    parent=s,
                    k=gen,
                    )
            
                ec = []
                if a in A.edgeCons:
                    ec = A.edgeCons[a]
            
                planner = AstarTimesteps(gridMap, Starts[a][0], Starts[a][1], Goals[a][0], Goals[a][1], A.vertexCons[a], ec)
                res = planner.FindPath()
                if res[0]:
                    path = MakePath(res[1])
                    A.sol[a], _ = path
                    A.g = sum([len(path) for path in A.sol.values()]) # SIC, можно использовать другой cost; добавить подсчет h, F
                    #OPEN.AddNode(A)
                    if  A.g > s.g:
                        FlagChild1 = True 
                    
                    gen += 1
                    
                tmp = s.vertexCons.copy()
                if b in tmp:
                    tmp[b].append(((i, j), t))   
                else:
                    tmp[b] = [((i, j), t)]
                            
                B = HighNode(
                    vertexCons=tmp,
                    edgeCons=s.edgeCons.copy(),
                    sol=s.sol.copy(),
                    parent=s,
                    k=gen,
                    )
            
                ec = []
                if b in B.edgeCons:
                    ec = B.edgeCons[b]
                    
                planner = AstarTimesteps(gridMap, Starts[b][0], Starts[b][1], Goals[b][0], Goals[b][1], B.vertexCons[b], ec)
                res = planner.FindPath()
                if res[0]:
                    path = MakePath(res[1])
                    B.sol[b], _ = path
                    B.g = sum([len(path) for path in B.sol.values()]) # SIC, можно использовать другой cost; добавить подсчет h, F
                    if B.g > s.g:
                        FlagChild2 = True 
                    gen += 1
                    
                if FlagChild1 and FlagChild2:
                    IsCardinal = True
                    OPEN.AddNode(A)
                    OPEN.AddNode(B)
                elif index == len(newVertexCons)-1:
                    OPEN.AddNode(A)
                    OPEN.AddNode(B)
                    break;
                else: 
                    index+=1
                
        elif len(newEdgeCons) > 0:
            while not IsCardinal:
                a, b, (i1, j1), (i2, j2), t = newEdgeCons[0]
            
                FlagChild1 = False
                FlagChild2 = False
                
                # Разбиваем CT на ноды A и B, разрешая реберный конфликт 
            
                tmp = s.edgeCons.copy()
                if a in tmp:
                    tmp[a].append(((i1, j1), (i2, j2), t))   
                else:
                    tmp[a] = [((i1, j1), (i2, j2), t)]
            
                A = HighNode(
                    vertexCons=s.vertexCons.copy(),
                    edgeCons=tmp,
                    sol=s.sol.copy(),
                    parent=s,
                    k=gen,
                    )
            
                vc = []
                if a in A.vertexCons:
                    vc = A.vertexCons[a]
            
                planner = AstarTimesteps(gridMap, Starts[a][0], Starts[a][1], Goals[a][0], Goals[a][1], vc, A.edgeCons[a])
                res = planner.FindPath()
                if res[0]:
                    path = MakePath(res[1])
                    A.sol[a], _ = path
                    A.g = sum([len(path) for path in A.sol.values()]) # SIC, можно использовать другой cost; добавить подсчет h, F
                    if A.g > s.g:
                        FlagChild1 = True
                    gen += 1
                
                tmp = s.edgeCons.copy()
                if b in tmp:
                    tmp[b].append(((i1, j1), (i2, j2), t))   
                else:
                    tmp[b] = [((i1, j1), (i2, j2), t)]
            
                B = HighNode(
                    vertexCons=s.vertexCons.copy(),
                    edgeCons=tmp,
                    sol=s.sol.copy(),
                    parent=s,
                    k=gen,
                    )
            
                vc = []
                if b in B.vertexCons:
                    vc = B.vertexCons[b]
                    
                planner = AstarTimesteps(gridMap, Starts[b][0], Starts[b][1], Goals[b][0], Goals[b][1], vc, B.edgeCons[b])
                res = planner.FindPath()
                if res[0]:
                    path = MakePath(res[1])
                    B.sol[b], _ = path
                    B.g = sum([len(path) for path in B.sol.values()]) # SIC, можно использовать другой cost; добавить подсчет h, F
                    if B.g > s.g:
                        FlagChild2 = True
                    gen += 1
                    
                if FlagChild1 and FlagChild2:
                    IsCardinal = True
                    OPEN.AddNode(A)
                    OPEN.AddNode(B)
                elif index == len(newEdgeCons)-1:
                    OPEN.AddNode(A)
                    OPEN.AddNode(B)
                else: 
                    index+=1
                
            toc = time.perf_counter()
    
    return (False, None, gen, exp)
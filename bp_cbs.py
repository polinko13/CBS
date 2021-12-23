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

def BP_CBS(gridMap, Starts, Goals, subset=[], vertexCons=[], edgeCons=[]): # subset, vertex/edge- Cons - нужны только при вызове CBS в качестве нижнего уровня MACBS
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
            a, b, (i, j), t = newVertexCons[0]
            
            FlagChildA = False
            FlagChildB = False
            
            # Разбиваем CT на ноды A и B, разрешая вершинный конфликт 
            
            vertexCons_tmp_A = s.vertexCons.copy()
            edgeCons_tmp_A=s.edgeCons.copy()
            sol_tmp_A=s.sol.copy()
            
            if a in vertexCons_tmp_A:
                vertexCons_tmp_A[a].append(((i, j), t))   
            else:
                vertexCons_tmp_A[a] = [((i, j), t)]
            
            #A = HighNode(vertexCons=tmp, edgeCons=s.edgeCons.copy(), sol=s.sol.copy(), parent=s, k=gen)
            
            
            ec = []
            if a in edgeCons_tmp_A:
                ec = edgeCons_tmp_A[a]
            
            planner = AstarTimesteps(gridMap, Starts[a][0], Starts[a][1], Goals[a][0], Goals[a][1], vertexCons_tmp_A[a], ec)
            res = planner.FindPath()
            if res[0]:
                path = MakePath(res[1])
                sol_tmp_A[a], _ = path
                g = sum([len(path) for path in sol_tmp_A.values()]) # SIC, можно использовать другой cost; добавить подсчет h, F
                if  g == s.g:
                    FlagChildA = True 
                
                
            vertexCons_tmp_B = s.vertexCons.copy()
            edgeCons_tmp_B=s.edgeCons.copy()
            sol_tmp_B=s.sol.copy()
            
            if b in vertexCons_tmp_B:
                vertexCons_tmp_B[b].append(((i, j), t))   
            else:
                vertexCons_tmp_B[b] = [((i, j), t)]
                            
            ec = []
            if b in edgeCons_tmp_B:
                ec = edgeCons_tmp_B[b]
            
            planner = AstarTimesteps(gridMap, Starts[b][0], Starts[b][1], Goals[b][0], Goals[b][1], vertexCons_tmp_B[b], ec)
            res = planner.FindPath()
            if res[0]:
                path = MakePath(res[1])
                sol_tmp_B[b], _ = path
                g = sum([len(path) for path in sol_tmp_B.values()]) # SIC, можно использовать другой cost; добавить подсчет h, F
                if g == s.g:
                    FlagChildB = True 
                
            if FlagChildA:
                s.vertexCons=vertexCons_tmp_A
                s.edgeCons=edgeCons_tmp_A
                s.sol=sol_tmp_A
            elif FlagChildB:
                s.vertexCons=tmp_B
                s.edgeCons=edgeCons_tmp_B
                s.sol=sol_tmp_B
            else:
                A = HighNode(
                    vertexCons=vertexCons_tmp_A,
                    edgeCons=edgeCons_tmp_A,
                    sol=sol_tmp_A,
                    parent=s,
                    k=gen,
                )
                gen+=1
                B = HighNode(
                    vertexCons=vertexCons_tmp_B,
                    edgeCons=edgeCons_tmp_B,
                    sol=sol_tmp_B,
                    parent=s,
                    k=gen,
                )
                gen+=1
                OPEN.AddNode(A)
                OPEN.AddNode(B)
                
                      

                
        elif len(newEdgeCons) > 0:
            a, b, (i1, j1), (i2, j2), t = newEdgeCons[0]
            
            FlagChildA = False
            FlagChildB = False
            
            # Разбиваем CT на ноды A и B, разрешая реберный конфликт 
            
            
            vertexCons_tmp_A = s.vertexCons.copy()
            edgeCons_tmp_A=s.edgeCons.copy()
            sol_tmp_A=s.sol.copy()
            
            if a in edgeCons_tmp_A:
                edgeCons_tmp_A[a].append(((i1, j1), (i2, j2), t))   
            else:
                edgeCons_tmp_A[a] = [((i1, j1), (i2, j2), t)]
            
            
            vc = []
            if a in vertexCons_tmp_A:
                vc = vertexCons_tmp_A[a]
            
            planner = AstarTimesteps(gridMap, Starts[a][0], Starts[a][1], Goals[a][0], Goals[a][1], vc, vertexCons_tmp_A[a])
            res = planner.FindPath()
            if res[0]:
                path = MakePath(res[1])
                sol_tmp_A[a], _ = path
                g = sum([len(path) for path in sol_tmp_A.values()]) # SIC, можно использовать другой cost; добавить подсчет h, F
                if  g == s.g:
                    FlagChildA = True 
                
                
            vertexCons_tmp_B = s.vertexCons.copy()
            edgeCons_tmp_B=s.edgeCons.copy()
            sol_tmp_B=s.sol.copy()
            
            if b in edgeCons_tmp_B:
                edgeCons_tmp_B[b].append(((i1, j1), (i2, j2), t))   
            else:
                edgeCons_tmp_B[b] = [((i1, j1), (i2, j2), t)]
            
            vc = []
            if b in vertexCons_tmp_B:
                vc = vertexCons_tmp_B[b]
            
            planner = AstarTimesteps(gridMap, Starts[b][0], Starts[b][1], Goals[b][0], Goals[b][1], vc, edgeCons_tmp_B[b])
            res = planner.FindPath()
            if res[0]:
                path = MakePath(res[1])
                sol_tmp_B[b], _ = path
                g = sum([len(path) for path in sol_tmp_B.values()]) # SIC, можно использовать другой cost; добавить подсчет h, F
                if  g == s.g:
                    FlagChildB = True 
                    
            if FlagChildA:
                s.vertexCons=vertexCons_tmp_A
                s.edgeCons=edgeCons_tmp_A
                s.sol=sol_tmp_A
            elif FlagChildB:
                s.vertexCons=tmp_B
                s.edgeCons=edgeCons_tmp_B
                s.sol=sol_tmp_B
            else:
                A = HighNode(
                    vertexCons=vertexCons_tmp_A,
                    edgeCons=edgeCons_tmp_A,
                    sol=sol_tmp_A,
                    parent=s,
                    k=gen,
                )
                gen+=1
                B = HighNode(
                    vertexCons=vertexCons_tmp_B,
                    edgeCons=edgeCons_tmp_B,
                    sol=sol_tmp_B,
                    parent=s,
                    k=gen,
                )
                gen+=1
                OPEN.AddNode(A)
                OPEN.AddNode(B)
                
        toc = time.perf_counter()
    
    return (False, None, gen, exp)
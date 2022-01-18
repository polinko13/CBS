import time as timer
import heapq
import random
import networkx as nx
import numpy as np
import math
import matplotlib.pyplot as plt


def move(loc, dir):
    directions = [(0, 0), (0, -1), (1, 0), (0, 1), (-1, 0)] 
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(5):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
                or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    table = {}
    for constraint in constraints:
        if constraint['agent'] == agent:
            ts = constraint['timestep']
            if ts not in table:
                table[constraint['timestep']] = []
            table[ts].append(constraint)
        elif constraint['agent'] == 'goal':
            if 'goal' not in table:
                table['goal'] = []
            table['goal'].append(constraint)
        elif constraint['positive'] is True:
            constraint['agent'] = agent
            constraint['positive'] = False
            constraint['loc'].reverse()
            ts = constraint['timestep']
            if ts not in table:
                table[constraint['timestep']] = []
            table[ts].append(constraint)
    return table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    if 'goal' in constraint_table:
        for constraint in constraint_table['goal']:
            if next_loc == constraint['loc'][0] and next_time >= constraint['timestep']:
                return True
    if next_time in constraint_table:
        constraints = constraint_table[next_time]
        for constraint in constraints:
            if constraint['positive'] == False:
                if len(constraint['loc']) == 1:
                    if constraint['loc'][0] == next_loc:
                        return True
                else:
                    if constraint['loc'][0] == curr_loc and constraint['loc'][1] == next_loc:
                        return True
            else:
                if len(constraint['loc']) == 1:
                    if constraint['loc'][0] is not next_loc:
                        return True
                else:
                    if constraint['loc'][0] == curr_loc and constraint['loc'][1] is not next_loc:
                        return True
    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    constraintTable = build_constraint_table(constraints, agent)
    
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep':0}
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root
    path_length = 0
    while len(open_list) > 0:
        curr = pop_node(open_list)
        goalConstraints = False
        for key in constraintTable.keys():
            if key != 'goal' and key > curr['timestep']:
                goal = {'loc': [goal_loc], 'agent': agent, 'timestep': key, 'positive': False}
                if goal in constraintTable[key]:
                    goalConstraints = True
                    break

        if curr['loc'] == goal_loc and not goalConstraints:
            return get_path(curr)

        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
            or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            if is_constrained(curr['loc'], child_loc, curr['timestep'] + 1, constraintTable):
                continue
            
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'timestep': curr['timestep'] + 1}
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)
        path_length = path_length + 1

    return None

def construct_MDD_for_agent(my_map, agent, start_loc, goal_loc, h_values, cost, constraints):
    MDD = nx.DiGraph()
    h_value = h_values[start_loc]
    open_list = []
    constraintTable = build_constraint_table(constraints, agent)

    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep':0}
    open_list.append(root)

    while len(open_list) > 0:
        curr = open_list.pop(0)

        if curr['timestep'] == cost:
            if curr['loc'] == goal_loc:
                path = get_path(curr)
                for i in range(len(path) - 1):
                    # MDD.add_node(path[i])
                    MDD.add_edge((path[i], i), (path[i+1], i+1))
            continue

        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
            or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            if is_constrained(curr['loc'], child_loc, curr['timestep'] + 1, constraintTable):
                continue
            if curr['g_val'] + h_values[child_loc] + 1 > cost:
                continue

            child = {
                'loc': child_loc,
                'g_val': curr['g_val'] + 1,
                'h_val': h_values[child_loc],
                'parent': curr,
                'timestep': curr['timestep'] + 1,
                }
            open_list.append(child)
    return MDD  

def reconstruct_MDD(MDD, start_loc):
    new_MDD = {}
    locations = nx.single_source_shortest_path_length(MDD, (start_loc, 0)) 
    for loc, depth in locations.items():
        if depth not in new_MDD:
            new_MDD[depth] = []
        new_MDD[depth].append(loc[0])
    return new_MDD

def updateMDD(MDD, agent, start_loc, goal_loc, cost, constraints):
    constraintTable = build_constraint_table(constraints, agent)
    MDD_copy = MDD.copy()
    recons_MDD = reconstruct_MDD(MDD, start_loc)

    for timestep, locations in recons_MDD.items():    
        if locations[0] == goal_loc:    
            break
        else: 
            for curr_loc in locations:
                for next_loc in list(MDD_copy.successors((curr_loc,timestep))):
                    if is_constrained(curr_loc, next_loc[0], timestep+1, constraintTable):
                        MDD_copy.remove_edge((curr_loc, timestep), next_loc)

    deleted_nodes = []
    for node in nx.nodes(MDD_copy):
        if node == (start_loc, 0):
            continue
        elif node != (goal_loc, cost):
            if len(list(MDD_copy.predecessors(node))) == 0 \
            or len(list(MDD_copy.successors(node))) == 0:
                deleted_nodes.append(node)
    MDD_copy.remove_nodes_from(deleted_nodes)        
    return MDD_copy


def detect_collision(path1, path2):
    timestep = max(len(path1), len(path2))
    for t in range(timestep):
        loc1 = get_location(path1, t)
        loc2 = get_location(path2, t)
        if loc1 == loc2:
            return ([loc1], t)
        if t < timestep - 1:
            loc1_next = get_location(path1, t+1)
            loc2_next = get_location(path2, t+1)
            if loc1 == loc2_next and loc2 == loc1_next:
                return ([loc1, loc2], t+1)
    return None
    
def detect_collisions(paths):
    collisions = []
    num_of_agents = len(paths)
    for i in range(num_of_agents - 1):
        for j in range(i + 1, num_of_agents):
            collision_t = detect_collision(paths[i], paths[j])
            if collision_t is not None:
                collision = {'a1': i, 'a2': j, 'loc': collision_t[0], 'timestep': collision_t[1]}
                collisions.append(collision)
    return collisions
    
def standard_splitting(collision):
    constraints = []
    loc = collision['loc']
    timestep = collision['timestep']
    a1 = collision['a1']
    a2 = collision['a2']

    if len(loc) == 1:
        constraints.append({'agent': a1, 'loc': loc, 'timestep': timestep, 'positive': False})
        constraints.append({'agent': a2, 'loc': loc, 'timestep': timestep, 'positive': False})
        return constraints

    if len(loc) == 2:
        reverse_loc = loc.copy()
        reverse_loc.reverse()
        constraints.append({'agent': a1, 'loc': loc, 'timestep': timestep, 'positive': False})
        constraints.append({'agent': a2, 'loc': reverse_loc, 'timestep': timestep, 'positive': False})
        return constraints
    
    
def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst


def construct_MDD(my_map, num_of_agents,starts, goals, h_values, paths, constraints):
    MDD = []
    for i in range(num_of_agents):
        MDD.append(construct_MDD_for_agent(my_map, i, starts[i], goals[i], h_values[i], len(paths[i]) - 1, constraints)) 
    return MDD


def compute_DG(MDD, num_of_agents, starts, goals):
    dependency_graph = construct_dependency_graph(num_of_agents, MDD, starts, goals)
    h_value = get_MVC(dependency_graph)
    return h_value


def isViolated(value_list, G, node, value):
    for key, val in value_list.items():
        if (key, node) in G.edges():
            if val + value < G.edges[key, node]['weight']:
                return True
    return False

def construct_dependency_graph(num_of_agents, MDD, starts, goals):
    dependency_graph = nx.Graph()
    for i in range(num_of_agents - 1):
        for j in range(i + 1, num_of_agents):
            joint_MDD, max_level = merge_MDD(MDD[i], starts[i], goals[i], MDD[j], starts[j], goals[j])
            if isDependent(joint_MDD, goals[i], goals[j], max_level) \
            or hasCardinal(MDD[i], starts[i], MDD[j], starts[j]):
                dependency_graph.add_nodes_from([i, j])
                dependency_graph.add_edge(i, j)
    return dependency_graph


def merge_MDD(MDD1, start1, goal1, MDD2, start2, goal2):
    len1 = len(reconstruct_MDD(MDD1, start1))
    len2 = len(reconstruct_MDD(MDD2, start2))
    MDD1_copy = MDD1.copy()
    MDD2_copy = MDD2.copy()
    if len1 > len2:
        edges = []
        for i in range(len2, len1):
            edges.append(((goal2, i-1), (goal2, i)))
        MDD2_copy.add_edges_from(edges)
    elif len1 < len2:
        edges = []
        for i in range(len1, len2):
            edges.append(((goal1, i-1), (goal1, i)))
        MDD1_copy.add_edges_from(edges)
    joint_MDD = {0:[(start1, start2)]}
    for i in range(max(len1, len2) - 1):
        joint_MDD[i+1] = []
        for pair in joint_MDD[i]:
            successor1 = [successor for successor, _ in list(MDD1_copy.successors((pair[0], i)))]
            successor2 = [successor for successor, _ in list(MDD2_copy.successors((pair[1], i)))]
            cross_product = [(x, y) for x in successor1 for y in successor2 if x != y]
            for new_pair in cross_product:
                if new_pair not in joint_MDD[i+1]:
                    joint_MDD[i+1].append(new_pair)
        if len(joint_MDD[i+1]) == 0:
            return joint_MDD, max(len1, len2)-1
            
    return joint_MDD, max(len1, len2)-1


def isDependent(joint_MDD, goal1, goal2, max_level):
    if max_level in joint_MDD:
        if (goal1, goal2) in joint_MDD[max_level]:
            return False
    return True


def hasCardinal(MDD1, start1, MDD2, start2):
    MDD1 = reconstruct_MDD(MDD1, start1)
    MDD2 = reconstruct_MDD(MDD2, start2)
    cost = min(len(MDD1), len(MDD2))
    for timestep in range(cost):
        if len(MDD1[timestep]) == 1 and len(MDD2[timestep]) == 1 \
        and MDD1[timestep][0] == MDD2[timestep][0]:
            return True
        if timestep < cost - 1:
            if len(MDD1[timestep]) == 1 and len(MDD2[timestep]) == 1 \
            and len(MDD1[timestep+1]) == 1 and len(MDD2[timestep+1]) == 1 \
            and MDD1[timestep][0] == MDD2[timestep+1][0] \
            and MDD1[timestep+1][0] == MDD2[timestep][0]:
                return True
    return False


def get_MVC(G):
    upperbound = nx.number_of_nodes(G)
    C = []
    MVC = EMVC(G, upperbound, C)
    return MVC


def EMVC(G, upperbound, C):
    if nx.is_empty(G):
        return len(C)
    cliques = get_disjoint_cliques(G)
    ClqLB = 0
    for clique in cliques:
        ClqLB += len(clique) - 1
    H = G.copy()
    num_of_edges = nx.number_of_edges(G)
    nodes = []
    degrees = []
    for degree in G.degree():
        nodes.append(degree[0])
        degrees.append(degree[1])
    DegLB = compute_DegLB(H, nodes, degrees, num_of_edges)
    if len(C) + max(DegLB, ClqLB) >= upperbound:
        return upperbound
    
    largest_index = np.argmax(degrees)
    vertex = nodes[largest_index]
    neighbors = [n for n in G.neighbors(vertex)]
    A = G.copy()
    A.remove_nodes_from(neighbors)
    A.remove_node(vertex)
    B = G.copy()
    B.remove_node(vertex)
    c1 = EMVC(A, upperbound, C + neighbors)
    c2 = EMVC(B, min(upperbound, c1), C + [vertex])
    return min(c1, c2)

def compute_DegLB(H, nodes, degrees, num_of_edges):
    i = 0
    total_degrees = 0
    while total_degrees < num_of_edges:
        largest_index = np.argmax(degrees)
        total_degrees += degrees[largest_index]
        H.remove_node(nodes[largest_index])
        degrees.remove(degrees[largest_index])
        nodes.remove(nodes[largest_index])
        i += 1
    num_of_edges_afterRemove = nx.number_of_edges(H)
    max_degree_afterRemove = max(degrees)
    DegLB = math.floor(i+num_of_edges_afterRemove/max_degree_afterRemove)
    return DegLB    

def get_disjoint_cliques(G):
    disjoint_cliques = []
    existing_nodes = []
    cliques = list(nx.find_cliques(G))
    cliques.sort(key = len, reverse = True)
    for clique in cliques:
        if len(disjoint_cliques) == 0:
            disjoint_cliques.append(clique)
            existing_nodes = existing_nodes + clique
        else:
            if len(set(clique).intersection(set(existing_nodes))) == 0:
                disjoint_cliques.append(clique)
                existing_nodes = existing_nodes + clique
    if nx.number_of_nodes(G) == len(existing_nodes):
        return disjoint_cliques
    else:
        nodes = [node for node in nx.nodes(G) if node not in existing_nodes]
        subgraph = G.subgraph(nodes)
        disjoint_cliques = disjoint_cliques + get_disjoint_cliques(subgraph)
        return disjoint_cliques


class CBSSolver(object):
    def __init__(self, gridMap, Starts, Goals):
        self.my_map = gridMap.cells
        self.starts = Starts.values()
        self.goals = Goals.values()
        self.num_of_agents = len(self.goals)
        self.heuristic = 'None'

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0
        self.construct_MDD = 0
        self.update_MDD = 0
        self.open_list = []
        self.sum_cost = 0

        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(self.my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        self.num_of_expanded += 1
        return node

    def find_solution(self, heuristic='DG', constraints=[]):
        self.heuristic = heuristic
        self.start_time = timer.time()

        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': [],
                'MDD': []}
        root['constraints'] = constraints.copy()
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)
        
        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])

        if heuristic != 'None':
            start_construct = timer.time()
            MDD = construct_MDD(self.my_map, self.num_of_agents, self.starts, self.goals, self.heuristics, root['paths'], [])
            self.construct_MDD += timer.time() - start_construct
            if heuristic == 'DG':
                h = compute_DG(MDD, self.num_of_agents, self.starts, self.goals)
                
            root['MDD'] = MDD
            
            MDD_all = []
            for i in range(self.num_of_agents):
                mdd_i = {}
                mdd_i[len(root['paths'][i])-1] = MDD[i].copy()
                MDD_all.append(mdd_i)

        self.push_node(root)
        while len(self.open_list) > 0:
            P = self.pop_node()
            if len(P['collisions']) == 0:
                self.sum_cost = self.print_results(P)
                return P['paths']
            collision = P['collisions'][0]
            constraints = standard_splitting(collision)
            for constraint in constraints:
                isAdd = True
                Q = {}
                Q['constraints'] = P['constraints'] + [constraint]
                Q['paths'] = [path.copy() for path in P['paths']]
                Q['MDD'] = [MDD.copy() for MDD in P['MDD']]
                if constraint['positive'] == False:
                    a = constraint['agent']
                    path = a_star(self.my_map, self.starts[a], self.goals[a], self.heuristics[a],
                          a, Q['constraints']) 
                    if path is not None:
                        Q['paths'][a] = path.copy()
                        if heuristic != 'None':
                            if len(P['paths'][a]) < len(path):
                                mdd_temp = 0
                                if (len(path) - 1) in MDD_all[a]: 
                                    
                                    mdd_temp = MDD_all[a][len(path)-1].copy()
                                else:
                                    start_construct = timer.time()
                                    mdd_temp =  construct_MDD_for_agent(self.my_map, a, self.starts[a], self.goals[a],self.heuristics[a], len(path) - 1, [])
                                    self.construct_MDD += timer.time() - start_construct
                                    MDD_all[a][len(path)-1] = mdd_temp.copy()
                                Q['MDD'][a] = mdd_temp.copy()
                            start_update = timer.time()
                            Q['MDD'][a] = updateMDD(Q['MDD'][a], a, self.starts[a], self.goals[a], len(path) - 1, Q['constraints'])
                            self.update_MDD += timer.time() - start_update
                    else:
                        isAdd = False
                    
                if isAdd:
                    Q['collisions'] = detect_collisions(Q['paths'])
                    h_value = 0
                    if heuristic == 'DG':
                        h_value = compute_DG(Q['MDD'], self.num_of_agents, self.starts, self.goals)
                        
                    Q['cost'] = get_sum_of_cost(Q['paths']) + h_value
                    
                    self.push_node(Q)
        
        return root['paths']


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("Use heuristic:    {}".format(self.heuristic))
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Construct MDD time (s):    {:.2f}".format(self.construct_MDD))
        print("Update MDD time (s):    {:.2f}".format(self.update_MDD))
        print("Run time (s):    {:.2f}".format(CPU_time-self.construct_MDD - self.update_MDD))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))

        return get_sum_of_cost(node['paths'])
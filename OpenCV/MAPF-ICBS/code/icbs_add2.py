import time as timer
import heapq
import random
# from single_agent_planner import compute_heuristics, a_star, get_location
# from multi_agent_planner import ll_solver, get_sum_of_cost, compute_heuristics, get_location

COST_WAIT = 1.0
COST_STRAIGHT = 1.2
COST_TURN = 1.6

PEN_YIELD = 0.8

from a_star_class_add1 import A_Star, compute_heuristics, get_location

import copy

import numpy

'''
   ## Reference to class
'''
######
'''
# Developer's cNOTE regarding Python's mutable default arguments:
#       The responsibiliy of preserving mutable values of passed arguments and 
#       preventing retention of local mutable defaults by assigning immuatable default values (i.e. param=None) in parameters
#       is the responsiblity of the function being called upon
#       PEP 505 - None-aware operators: https://www.python.org/dev/peps/pep-0505/#syntax-and-semantics
'''

def _safe_get(path, t):
    return path[t] if t < len(path) else path[-1]

def _step_type(prev_prev, prev, nxt):
    if nxt == prev:
        return "wait"
    if prev_prev is None or prev_prev == prev:
        return "straight"
    v1 = (prev[0]-prev_prev[0], prev[1]-prev_prev[1])
    v2 = (nxt[0]-prev[0],      nxt[1]-prev[1])
    return "straight" if v1 == v2 else "turn"

def build_yield_prev_table(paths, excluded_agents):
    table = {}
    N = len(paths)
    maxT = max((len(p) for p in paths), default=0)
    for t in range(1, maxT+1):
        cells = set()
        for i in range(N):
            if i in excluded_agents or not paths[i]:
                continue
            prev_i = _safe_get(paths[i], t-1)
            cells.add(prev_i)
        table[t] = cells
    return table


def weighted_sum_cost(paths):
    total = 0.0
    N = len(paths)
    maxT = max((len(p) for p in paths), default=0)
    for i, path in enumerate(paths):
        if not path: 
            continue
        for t in range(1, len(path)):
            prev_prev = path[t-2] if t-2 >= 0 else None
            prev = path[t-1]
            curr = path[t]

            st = _step_type(prev_prev, prev, curr)
            if st == "wait":
                total += COST_WAIT
            elif st == "straight":
                total += COST_STRAIGHT
            else:
                total += COST_TURN

            for j in range(N):
                if j == i or not paths[j]:
                    continue
                prev_j = _safe_get(paths[j], t-1)
                if curr == prev_j:
                    total += PEN_YIELD
                    break
    return total

def generate_child(constraints, paths, agent_collisions, ma_list):

    assert isinstance(ma_list , list)

    collisions = detect_collisions(paths, ma_list)
    cost = weighted_sum_cost(paths)
    child_node = {
        'cost':cost,
        'constraints': copy.deepcopy(constraints),
        'paths': copy.deepcopy(paths), # {0: {'path':[..path...]}, ... , n: {'path':[..path...]} # not sure if other keys are needed
        'ma_collisions': collisions,
        'agent_collisions':copy.deepcopy(agent_collisions), # matrix of collisions in history between pairs of simple agents
        'ma_list': copy.deepcopy(ma_list) # [{a1,a2}, ... ]
    }
    return child_node

def detect_collision(path1, path2, pos=None):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    assert pos is None
    if pos is None:
        pos = []
    t_range = max(len(path1),len(path2))
    for t in range(t_range):
        loc_c1 = get_location(path1,t)
        loc_c2 = get_location(path2,t)
        loc1 = get_location(path1,t+1)
        loc2 = get_location(path2,t+1)
        # vertex collision
        if loc1 == loc2:
            pos.append(loc1)
            return pos,t
        # edge collision
        if[loc_c1,loc1] ==[loc2,loc_c2]:
            pos.append(loc2)
            pos.append(loc_c2)
            return pos,t
        
       
    return None


def detect_collisions(paths, ma_list, collisions=None):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.

    if collisions is None:
        collisions = []
    for ai in range(len(paths)-1):
        for aj in range(ai+1,len(paths)):
            if detect_collision(paths[ai],paths[aj]) !=None:
                position,t = detect_collision(paths[ai],paths[aj])

                # find meta-agents of agents in collision 
                assert isinstance(ma_list , list)
                ma_i = get_ma_of_agent(ai, ma_list)
                assert isinstance(ma_list , list)
                ma_j = get_ma_of_agent(aj, ma_list)

                # check if internal collision in the same meta-agent
                if ma_i != ma_j:
                    collisions.append({'a1':ai, 'ma1':ma_i,
                                    'a2':aj, 'ma2':ma_j,
                                    'loc':position,
                                    'timestep':t+1})
    return collisions

def count_all_collisions_pair(path1, path2):
    collisions = 0
    t_range = max(len(path1),len(path2))
    for t in range(t_range):
        loc_c1 =get_location(path1,t)
        loc_c2 = get_location(path2,t)
        loc1 = get_location(path1,t+1)
        loc2 = get_location(path2,t+1)
        if loc1 == loc2 or [loc_c1,loc1] ==[loc2,loc_c2]:
            collisions += 1
    return collisions

def count_all_collisions(paths):
    collisions = 0
    for i in range(len(paths)-1):
        for j in range(i+1,len(paths)):
            ij_collisions = count_all_collisions_pair(paths[i],paths[j])
            collisions += ij_collisions

    # print("number of collisions: ", collisions)
    return collisions    
  
def standard_splitting(collision, constraints=None):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    if constraints is None:
        constraints = []


    if len(collision['loc'])==1:
        constraints.append({'agent':collision['a1'],
                            'meta_agent': collision['ma1'],
                            'loc':collision['loc'],
                            'timestep':collision['timestep'],
                            'positive':False
                            })
        constraints.append({'agent':collision['a2'],
                            'meta_agent': collision['ma2'],
                            'loc':collision['loc'],
                            'timestep':collision['timestep'],
                            'positive':False
                            })
    else:
        constraints.append({'agent':collision['a1'],
                            'meta_agent': collision['ma1'],
                            'loc':[collision['loc'][0],collision['loc'][1]],
                            'timestep':collision['timestep'],
                            'positive':False
                            })
        constraints.append({'agent':collision['a2'],
                            'meta_agent': collision['ma2'],
                            'loc':[collision['loc'][1],collision['loc'][0]],
                            'timestep':collision['timestep'],
                            'positive':False
                            })
    return constraints

    # pass


def disjoint_splitting(collision, constraints=None):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    if constraints is None:
        constraints = []

    a = random.choice([('a1','ma1'), ('a2','ma2')]) # chosen agent
    agent = a[0]
    meta_agent = a[1]

    print(agent, collision)

    if len(collision['loc'])==1:
        constraints.append({'agent':collision[agent],
                            'meta_agent': collision[meta_agent],
                            'loc':collision['loc'],
                            'timestep':collision['timestep'],
                            'positive':True
                            })
        constraints.append({'agent':collision[agent],
                            'meta_agent': collision[meta_agent],
                            'loc':collision['loc'],
                            'timestep':collision['timestep'],
                            'positive':False
                            })
    else:
        if agent == 'a1':
            constraints.append({'agent':collision[agent],
                                'meta_agent': collision[meta_agent],
                                'loc':[collision['loc'][0],collision['loc'][1]],
                                'timestep':collision['timestep'],
                                'positive':True
                                })
            constraints.append({'agent':collision[agent],
                                'meta_agent': collision[meta_agent],
                                'loc':[collision['loc'][0],collision['loc'][1]],
                                'timestep':collision['timestep'],
                                'positive':False
                                })
        else:
            constraints.append({'agent':collision[agent],
                                'meta_agent': collision[meta_agent],
                                'loc':[collision['loc'][1],collision['loc'][0]],
                                'timestep':collision['timestep'],
                                'positive':True
                                })
            constraints.append({'agent':collision[agent],
                                'meta_agent': collision[meta_agent],
                                'loc':[collision['loc'][1],collision['loc'][0]],
                                'timestep':collision['timestep'],
                                'positive':False
                                })
    return constraints

# get the meta-agent an agent is a part of
# do NOT use for constraints, use key 'meta-agent' in constraint
def get_ma_of_agent(agent, ma_list):

    assert isinstance(ma_list , list)
    for ma in ma_list:
        # print(ma, ma_list)
        if agent in ma:
            # print(agent, ma)
            return ma
    raise BaseException('No meta-agent found for agent')
                        

# find meta-agents of the agents that violates constraint
def meta_agents_violate_constraint(constraint, paths, ma_list, violating_ma=None):
    assert constraint['positive'] is True
    if violating_ma is None:
        violating_ma = []

    for i in range(len(paths)):
        ma_i = get_ma_of_agent(i, ma_list)

        if ma_i == constraint['meta_agent'] or ma_i in violating_ma:
            continue


        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                # if ma_i not in violating_ma:
                    violating_ma.append(ma_i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                # if ma_i not in violating_ma:
                violating_ma.append(ma_i)

    return violating_ma


def paths_violate_constraint(constraint, paths, rst=None):
    assert constraint['positive'] is True
    if rst is None:
        rst = []

    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst

def combined_constraints(constraints, new_constraints, updated_constraints=None):
    assert updated_constraints is None

    if isinstance(new_constraints, list):
        updated_constraints = copy.deepcopy(new_constraints)
    else:
        updated_constraints = [new_constraints]

    # print('combining constraints:')
    # print('const1: ', constraints)
    # print('const2: ', updated_constraints)

    for c in constraints:
        if c not in updated_constraints:
            updated_constraints.append(c)

    assert len(updated_constraints) <= len(constraints) + len(new_constraints)
    return updated_constraints


def bypass_found(curr_cost, new_cost, curr_collisions_num, new_collisions_num, eps=1e-9):
    if abs(curr_cost - new_cost) < eps and (new_collisions_num < curr_collisions_num):
        return True
    return False


def should_merge(collision, p, N=0):
    a1 = collision['a1']
    a2 = collision['a2']

    if a1 > a2:
        a1, a2 = a2, a1
    assert a1 < a2
    p['agent_collisions'][a1][a2] += 1

    if p['agent_collisions'][a1][a2] > N:
        return True

    ma1 = collision['ma1']
    ma2 = collision['ma2']
    
    # check it is same meta-agent
    assert ma1 != ma2
    assert not (a2 in ma1 or a1 in ma2)

    return False


class ICBS_Solver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['ma_collisions']), self.num_of_generated, node))
        print("> Generate node {} with cost {}".format(self.num_of_generated, node['cost']))
        self.num_of_generated += 1
        

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("> Expand node {} with cost {}".format(id, node['cost']))
        self.num_of_expanded += 1
        return node

    def empty_tree(self):
        self.open_list.clear()

    # algorithm for detecting cardinality
    # as 'non-cardinal' or 'semi-cardinal' or 'cardinal'
    # using standard splitting
    def detect_cardinal_conflict(self, AStar, p, collision):
        cardinality = 'non-cardinal'

        # temporary constraints (standard splitting) for detecting cardinal collision purposes
        temp_constraints = standard_splitting(collision)


        ma1 = collision['ma1'] #agent a1
        yield_prev1 = build_yield_prev_table(p['paths'], excluded_agents=ma1)

        # print('Sending ma1 in collision {} to A* '.format(ma1))

        assert temp_constraints[0]['meta_agent'] == ma1
        path1_constraints = combined_constraints(p['constraints'], temp_constraints[0])
        astar_ma1 = AStar(self.my_map,self.starts,self.goals,self.heuristics,
                  list(ma1), path1_constraints,
                  yield_prev_table=yield_prev1)
        alt_paths1 = astar_ma1.find_paths()

        # get current paths of meta-agent
        curr_paths = []
        for a1 in ma1:

            not_nested_list = p['paths'][a1]
            assert any(isinstance(i, list) for i in not_nested_list) == False


            curr_paths.append(p['paths'][a1])

        # print(curr_paths)
        # print(alt_paths1)

        # get costs for the meta agent
        curr_cost = weighted_sum_cost(curr_paths)
        
        alt_cost = 0 # write inline if later
        if alt_paths1:
            alt_cost = weighted_sum_cost(alt_paths1)

        # print('\t oldcost:{} newcost:{}'.format(curr_cost, alt_cost))

        if not alt_paths1 or alt_cost > curr_cost:
            cardinality = 'semi-cardinal'
            
            print('alt_path1 takes longer or is empty. at least semi-cardinal.')
            
            
        ma2 = collision['ma2'] #agent a2
        yield_prev2 = build_yield_prev_table(p['paths'], excluded_agents=ma2)
        # print('Sending ma2 in collision {} to A* '.format(ma2))

        assert temp_constraints[1]['meta_agent'] == ma2
        path2_constraints = combined_constraints(p['constraints'], temp_constraints[1])
        astar_ma2 = AStar(self.my_map,self.starts, self.goals,self.heuristics,list(ma2),path2_constraints,yield_prev_table=yield_prev2)
        alt_paths2 = astar_ma2.find_paths()

        # if not alt_path2 or bigger:
        curr_paths = []
        for a2 in ma2:    
            not_nested_list = p['paths'][a2]
            assert any(isinstance(i, list) for i in not_nested_list) == False

            curr_paths.append(p['paths'][a2])
            
        # print(curr_paths)
        # print(alt_paths2)

        # get costs for the meta agent
        curr_cost = weighted_sum_cost(curr_paths)
        
        alt_cost = 0 # write inline if later
        if alt_paths2:
            alt_cost = weighted_sum_cost(alt_paths2)

        # print('\t oldcost:{} newcost:{}'.format(curr_cost, alt_cost))

        if not alt_paths2 or alt_cost > curr_cost:
            # cardinality = 'semi-cardinal'
            if cardinality == 'semi-cardinal':
                cardinality = 'cardinal'
                
                # print('identified cardinal conflict')

            else:
                cardinality = 'semi-cardinal'
                
                # print('alt_path2 takes longer or is empty. semi-cardinal.')   
        # print('cardinality: ', cardinality)
            
        return cardinality        

    # returns new merged agents (the meta-agent), and updated list of ma_list
    def merge_agents(self, collision, ma_list):

        # constraints = standard_splitting(collision)
        
        # collision simple agents and their meta-agent group
        a1 = collision['a1']
        a2 = collision['a2']
        ma1 = collision['ma1']
        ma2 = collision['ma2']

        meta_agent = set.union(ma1, ma2)

        print('new merged meta_agent ', meta_agent)

        assert meta_agent not in ma_list

        ma_list.remove(ma1)
        ma_list.remove(ma2)
        ma_list.append(meta_agent)

        return meta_agent, ma_list


    def find_solution(self, disjoint):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint         - use disjoint splitting or not
        """

        self.start_time = timer.time()
        
        if disjoint:
            splitter = disjoint_splitting
        else:
            splitter = standard_splitting

        AStar = A_Star

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {
            'cost':0,
            'constraints': [],
            'paths': [],
            'ma_collisions': [],
            'agent_collisions': None, # matrix of collisions in history between pairs of (meta-)agents
            'ma_list': [] # [{a1,a2}, ... ]
        }       
        
        for i in range(self.num_of_agents):  # Find initial path for each agent
            astar = AStar(self.my_map, self.starts, self.goals, self.heuristics, [i], root['constraints'])
            path = astar.find_paths()


            if path is None:
                raise BaseException('No solutions')
            root['ma_list'].append({i})
            root['paths'].extend(path)



        root['cost'] = weighted_sum_cost(root['paths'])
        root['ma_collisions'] = detect_collisions(root['paths'], root['ma_list'])
        root['agent_collisions'] = numpy.zeros((self.num_of_agents, self.num_of_agents))
        self.push_node(root)



        # ATTENTION: THE CBS LOOOOOOOOOOOOP ============@#￥#%#@￥@#%##@￥======  STARTS ---#￥%------   HERE  ---- @
        # normal CBS with disjoint and standard splitting
        while len(self.open_list) > 0:
            if self.num_of_generated > 50000:
                print('reached maximum number of nodes. Returning...')
                return None 
            print('\n')  
            p = self.pop_node()
            if p['ma_collisions'] == []:
                self.print_results(p)
                # for pa in p['paths']:
                #     # print('asfasdfasdf       ',pa)
                return p['paths'], self.num_of_generated, self.num_of_expanded # number of nodes generated/expanded for comparing implementations


            print('Node expanded. Collisions: ', p['ma_collisions'])
            for pa in p['paths']:
                print(pa)

            print('\n> Find Collision Type')

            # USING STANDARD SPLITTING
            # select a cardinal conflict;
            # if none, select a semi-cardinal conflict
            # if none, select a random conflict
            chosen_collision = None
            new_constraints = None
            collision_type = None
            for collision in p['ma_collisions']:

                print(collision)

                collision_type = self.detect_cardinal_conflict(AStar, p, collision)
                if collision_type == 'cardinal' and new_constraints is None:    
                    print('Detected cardinal collision. Chose it.')
                    print(collision)

                    chosen_collision = collision
                    # collision_type = 'cardinal'
                    break

            else: # no cardinal collisions found
                for collision in p['ma_collisions']:
                    collision_type = self.detect_cardinal_conflict(AStar, p, collision)
                    if collision_type == 'semi-cardinal':    
                        
                        print('Detected semi-cardinal collision. Chose it.')
                        print(collision)
                        chosen_collision = collision
                        # collision_type = 'semi-cardinal'
                        break

                else: # no semi-cardinal collision found
                    chosen_collision = p['ma_collisions'][0] 
                    assert chosen_collision is not None
                    collision_type = 'non-cardinal'
                    print('No cardinal or semi-cardinal conflict. Randomly choosing...')


            # keep track of collisions in history (aSh)
            chosen_a1 = chosen_collision['a1']
            chosen_a2 = chosen_collision['a2']
            if chosen_a1 > chosen_a2:
                # swap to only fill half of the matrix
                chosen_a1, chosen_a2 = chosen_a2, chosen_a1
            p['agent_collisions'][chosen_a1][chosen_a2] += 1


            new_constraints = splitter(chosen_collision)

            print('OLD CONSTS:')
            print(p['constraints'])      

            print('NEW CONSTS:')
            print(new_constraints)
            print('\n')
            # child_nodes = None
            child_nodes = []
            assert child_nodes == []
            bypass_successful = False
            for constraint in new_constraints:
                print(constraint)
                
                updated_constraints = combined_constraints(p['constraints'], constraint)
                q = generate_child(updated_constraints, p['paths'], p['agent_collisions'], p['ma_list'])


                assert isinstance(p['ma_list'] , list)
                assert isinstance(q['ma_list'] , list)

                ma = constraint['meta_agent']
                yield_prev = build_yield_prev_table(q['paths'], excluded_agents=ma)

                print('\nSending meta_agent {} of constrained agent {} to A* '.format(ma, constraint['agent']))
                print('\twith constraints ', q['constraints'])

                for a in ma:
                    print (q['paths'][a])

                astar = AStar(self.my_map, self.starts, self.goals,
                            self.heuristics, list(ma), q['constraints'],
                            yield_prev_table=yield_prev)
                paths = astar.find_paths()

                if paths is not None:
                    
                    for i in range(len(paths)):
                                print (paths[i])
                    for i, agent in enumerate(ma):

                        

                        not_nested_list = paths[i]
                        assert any(isinstance(j, list) for j in not_nested_list) == False


                        q['paths'][agent] = paths[i]

                    if constraint['positive']:
                        # vol = paths_violate_constraint(constraint,q['paths'])
                        violating_ma_list = meta_agents_violate_constraint(constraint, q['paths'], q['ma_list'])
                        no_solution = False
                        for v_ma in violating_ma_list:
                            
                            print('\nSending meta-agent violating constraint {} to A* '.format(v_ma))
                            print('\twith constraints ', q['constraints'])

                            for a in v_ma:
                                print (q['paths'][a])


                            v_ma_list = list(v_ma) # should use same list for all uses
                            yield_prev_v = build_yield_prev_table(q['paths'], excluded_agents=v_ma)
                            astar_v_ma = AStar(self.my_map,self.starts,self.goals,self.heuristics,v_ma_list,q['constraints'], yield_prev_table=yield_prev_v)
                            paths_v_ma = astar_v_ma.find_paths()



                            # replace paths of meta-agent with new paths found
                            if paths_v_ma is not None:

                                for i in range(len(v_ma_list)):
                                    print (paths_v_ma[i])

                                for i, agent in enumerate(v_ma_list):

                                    assert paths_v_ma[i] is not None
                                    print(paths_v_ma[i])

                                    not_nested_list = paths_v_ma[i]
                                    assert any(isinstance(j, list) for j in not_nested_list) == False


                                    q['paths'][agent] = paths_v_ma[i]
                            else:
                                print("no solution, moving on to next constraint")   
                                no_solution = True
                                break # move on the next constraint
                                
                        if no_solution:
                            continue # move on to the next constraint

                    q['ma_collisions'] = detect_collisions(q['paths'],q['ma_list'])

                    if chosen_collision in q['ma_collisions']:
                        print(q['paths'])
                        print('\nOH NO!!!!! chosen_collision is still in child :\'(')
                        print(chosen_collision)

                    assert chosen_collision not in q['ma_collisions']

                    q['cost'] = weighted_sum_cost(q['paths'])


                    # assert that bypass is not possible if cardinal
                    if collision_type == 'cardinal':
                        assert bypass_found(p['cost'], q['cost'], len(p['ma_collisions']), len(q['ma_collisions'])) == False

                    # conflict should be resolved due to new constraints; compare costs and total number of collisions
                    if collision_type != 'cardinal' \
                            and bypass_found(p['cost'], q['cost'], len(p['ma_collisions']), len(q['ma_collisions'])):
                        print('> Take Bypass')
                        self.push_node(q)
                        
                        bypass_successful = True
                        break # break out of constraint loop
                    assert not bypass_successful
                    child_nodes.append(copy.deepcopy(q))

            if bypass_successful:
                continue # start of while loop

            assert not bypass_successful

            # MA-CBS
            if should_merge(collision, p, 7):
                print('> Merge meta-agents into a new')
                # returns meta_agent, ma_list
                meta_agent, updated_ma_list = self.merge_agents(collision, p['ma_list'])


                # updated constraints
                updated_constraints = copy.deepcopy(p['constraints'])
                for c in updated_constraints:
                    if c['meta_agent'].issubset(meta_agent):
                        c['meta_agent'] = meta_agent

                print('Sending newly merged meta_agent {} to A* '.format(meta_agent))
                print('\twith constraints ', updated_constraints)

                for a in meta_agent:
                    print (p['paths'][a])
                yield_prev = build_yield_prev_table(p['paths'], excluded_agents=meta_agent)
                # Update paths
                ma_astar = AStar(self.my_map,self.starts, self.goals,self.heuristics,list(meta_agent), updated_constraints, yield_prev)
                ma_paths = ma_astar.find_paths()


                # if can be 
                if ma_paths:

                    for i in range(len(meta_agent)):
                        print (ma_paths[i])
                                        
                    updated_paths = copy.deepcopy(p['paths'])

                    for i, agent in enumerate(meta_agent):
                        
                        assert isinstance(i, int)

                        not_nested_list = ma_paths[i]
                        assert any(isinstance(j, list) for j in not_nested_list) == False



                        updated_paths[agent] = ma_paths[i]


                    # for a in meta_agent:
                    #     print (updated_paths[a])

                    # Update collisions, cost
                    updated_node = generate_child(updated_constraints, updated_paths, p['agent_collisions'], updated_ma_list) 


                    # print('agents {}, {} merged into agent {}'.format(collision['a1'], a2, meta_agent))

                    # Merge & restart
                    # restart with only updated node with merged agents
                    self.empty_tree()

                    assert self.open_list == []

                    self.push_node(updated_node)    

                    continue # start of while loop
            else:
                print("do not merge")
                
            assert len(child_nodes) <= 2
            print('bypass not found')
            for n in child_nodes:
                self.push_node(n)     
                    
        return None


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(weighted_sum_cost(node['paths'])))
        
        # file = "nodes-generated.csv"
        # result_file = open(file, "a", buffering=1)
        # result_file.write("{}\n".format(self.num_of_generated))

        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))


        print("Solution:")
        for i in range(len(node['paths'])):
            print("agent", i, ": ", node['paths'][i])

class InICBS:
    """
    D-확장 루프를 내부에서 도는 Incremental ICBS 래퍼.

    외부에서는:
      - dynamic_starts, dynamic_goals : 이번에 다시 짤 로봇들
      - other_paths : 나머지 로봇들의 남은 경로들
    만 넘겨주면 되고, 내부에서 D 확장 + 여러 번 ICBS 호출을 관리한다.
    """

    def __init__(self, my_map, dynamic_starts, dynamic_goals, other_paths):
        """
        my_map          : 기존 CBS와 동일한 맵 표현
        dynamic_starts  : [(...), ...]   # 이번에 다시 짤 로봇들의 시작 위치
        dynamic_goals   : [(...), ...]   # 위 로봇들의 목표 위치
        other_paths     : [path_j, ...]  # 나머지 로봇들의 '남은 경로'

        path 포맷은 기존 ICBS/A*에서 쓰는 것과 동일하다고 가정.
        """
        assert len(dynamic_starts) == len(dynamic_goals)

        self.my_map = my_map
        self.dynamic_starts = list(dynamic_starts)
        self.dynamic_goals = list(dynamic_goals)
        self.other_paths = list(other_paths)

        # 전체 에이전트 개수 (global index 0..N_total-1 를 이 solve 안에서만 사용)
        self.num_dynamic_initial = len(self.dynamic_starts)
        self.num_fixed_initial = len(self.other_paths)
        self.num_total = self.num_dynamic_initial + self.num_fixed_initial

        # 내부에서 쓸 global index:
        #  - 0 .. D0-1           : 처음부터 dynamic인 로봇
        #  - D0 .. D0+R0-1       : 처음에는 fixed였던 로봇
        self.dynamic_ids = set(range(self.num_dynamic_initial))
        self.fixed_ids = set(range(self.num_dynamic_initial, self.num_total))

        # fixed 에이전트들도 나중에 dynamic으로 편입될 수 있으니,
        # 미리 start/goal을 추출해 둔다.
        # start = 남은 경로의 첫 위치, goal = 남은 경로의 마지막 위치
        self.fixed_starts = {}
        self.fixed_goals = {}
        for idx, path in enumerate(self.other_paths):
            global_id = self.num_dynamic_initial + idx
            if not path:
                # 경로가 비어있는 경우는 거의 없다고 가정하지만, 방어적으로 처리
                self.fixed_starts[global_id] = None
                self.fixed_goals[global_id] = None
            else:
                self.fixed_starts[global_id] = path[0]
                self.fixed_goals[global_id] = path[-1]

        # fixed_paths: global_id -> path
        self.fixed_paths = {}
        for idx, path in enumerate(self.other_paths):
            global_id = self.num_dynamic_initial + idx
            self.fixed_paths[global_id] = path

        # 통계용
        self.total_nodes_generated = 0
        self.total_nodes_expanded = 0

    def _build_starts_goals_for_D(self):
        """
        현재 dynamic_ids 집합 D에 대해,
        ICBS_Solver에 넘길 starts/goals 리스트와
        local index -> global index 매핑을 만든다.
        """
        starts_D = []
        goals_D = []
        local_to_global = []

        # deterministic하게 하기 위해 sorted 사용 (원하면 다른 순서도 가능)
        for g_id in sorted(self.dynamic_ids):
            local_to_global.append(g_id)
            if g_id < self.num_dynamic_initial:
                # 처음부터 dynamic이었던 에이전트
                starts_D.append(self.dynamic_starts[g_id])
                goals_D.append(self.dynamic_goals[g_id])
            else:
                # 원래 fixed였다가 dynamic으로 편입된 에이전트
                s = self.fixed_starts.get(g_id)
                g = self.fixed_goals.get(g_id)
                assert s is not None and g is not None, \
                    f"fixed agent {g_id} has no start/goal"
                starts_D.append(s)
                goals_D.append(g)

        return starts_D, goals_D, local_to_global

    def _merge_candidate_paths(self, D_paths_local, local_to_global):
        """
        D에 대한 새 경로(D_paths_local)를
        기존 fixed_paths와 합쳐서 full candidate plan을 만든다.
        반환값: cand_paths (길이 = num_total, index=global_id 기준)
        """
        # 1) local -> global 매핑을 dict로 바꿈
        D_paths_global = {}
        for li, gi in enumerate(local_to_global):
            D_paths_global[gi] = D_paths_local[li]

        # 2) 전체 candidate plan 생성
        cand_paths = [None] * self.num_total
        for g_id in range(self.num_total):
            if g_id in D_paths_global:
                cand_paths[g_id] = D_paths_global[g_id]
            else:
                # 아직 fixed인 애들
                cand_paths[g_id] = self.fixed_paths[g_id]

        return cand_paths

    def _find_colliding_fixed_agents(self, cand_paths):
        """
        cand_paths에서 dynamic vs fixed 충돌을 검사하고,
        새로 dynamic으로 편입되어야 할 fixed 에이전트 global_id 집합을 반환한다.
        """
        # meta-agent는 전부 singleton으로 둔다.
        ma_list = [[i] for i in range(self.num_total)]

        collisions = detect_collisions(cand_paths, ma_list)
        colliding_fixed = set()

        for col in collisions:
            a1 = col['a1']
            a2 = col['a2']
            inD1 = a1 in self.dynamic_ids
            inD2 = a2 in self.dynamic_ids

            # D 내부 충돌은 ICBS가 해결해 줬다고 가정 (collision-free)
            # 우리가 관심 있는 건 D와 R 사이의 충돌만.
            if inD1 and not inD2:
                colliding_fixed.add(a2)
            elif inD2 and not inD1:
                colliding_fixed.add(a1)

        return colliding_fixed

    def solve(self, disjoint=False):
        """
        외부에서 한 번만 호출되는 엔트리포인트.

        - 내부에서 D 확장 루프를 돌며,
        - 최종적으로 dynamic 초기 집합(0..num_dynamic_initial-1)에 대한 새 경로 리스트를 리턴한다.

        return:
            paths_dynamic, total_nodes_generated, total_nodes_expanded
        """
        # D 확장 루프
        while True:
            # 1) 현재 D에 대해 starts/goals/local_to_global 구성
            starts_D, goals_D, local_to_global = self._build_starts_goals_for_D()

            # 2) 부분 집합 D에 대해 ICBS 한 번 실행
            icbs = ICBS_Solver(self.my_map, starts_D, goals_D)
            result = icbs.find_solution(disjoint)

            if result is None:
                # 해를 못 찾은 경우 - 정책에 따라 처리
                return None, self.total_nodes_generated, self.total_nodes_expanded

            D_paths_local, nodes_gen, nodes_exp = result
    
            # 통계 누적
            self.total_nodes_generated += nodes_gen
            self.total_nodes_expanded += nodes_exp

            # 3) candidate full plan 구성
            cand_paths = self._merge_candidate_paths(D_paths_local, local_to_global)

            # 4) dynamic vs fixed 충돌 검사
            colliding_fixed = self._find_colliding_fixed_agents(cand_paths)

            if not colliding_fixed:
                return cand_paths, self.total_nodes_generated, self.total_nodes_expanded

            # 5) 새로 충돌한 fixed 에이전트를 D로 편입
            self.dynamic_ids |= colliding_fixed
            self.fixed_ids -= colliding_fixed

            # 방어적: 모든 에이전트가 dynamic이 되었는데도 충돌이 남는다 → ICBS가 실패한 상태
            if len(self.dynamic_ids) == self.num_total and len(colliding_fixed) > 0:
                # full ICBS로도 해결이 안 되는 경우니 실패로 본다.
                # (필요하면 여기서 다시 한 번 full ICBS를 직접 돌리는 것도 가능)
                return None, self.total_nodes_generated, self.total_nodes_expanded

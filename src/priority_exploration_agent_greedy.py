# -- Global Imports
import numpy as np
import sys
import heapq
from datetime import datetime
# -- Local Imports
from exploration_env import GraphExplorationEnvironment
from int_wrapper import Wrapper
from copy import deepcopy

class PriorityExplorationAgent:
    def __init__(self, t_a, t_r0, priority_filename, edges_filename, vertextag_filename, init_position, output_filename):
        self.priority_filename = priority_filename
        self.edges_filename = edges_filename
        self.vertextag_filename = vertextag_filename
        self.init_position = init_position
        self.output_filename = output_filename
        
        # -- reset output file
        self._reset_output_file()
        
        # -- initialize vertex tag
        self.node_priority = self._read_node_priority(priority_filename)
        
        # -- initialize exploration environment
        self.env = GraphExplorationEnvironment(edges_filename, 
                                               vertextag_filename, 
                                               priority_filename,
                                               t_a,
                                               t_r0,
                                               init_position)
        
        # -- initialize visited nodes
        self.visited_nodes = []
        self.all_time_limits = []
        
        # -- initialize completed exploration
        self.flag_completed = True
        self.flag_reached_home = True
        self.flag_go_straight_home = True
        
        # -- set current state
        self.current_position = init_position
        print ("PEA: Initialized, current position: {}".format(self.current_position))
    
  
    def explore(self, debug_print = False):
        observation = self.env.get_state_consolidated()
        self._print_observation_to_file(observation)
        self._print_observation_to_screen(observation)

        while True:
            # -- update visited nodes
            self.visited_nodes.append(observation[0])
            
            # -- add time limits to global list
            self.all_time_limits.append(observation[2])

            # -- check for time remaining
            t_r = observation[2]
            if t_r is not None and t_r <= 0:
                break

            # -- check if exploration is complete
            if len(observation[1]) == 0:
                # we do not have places to move
                if debug_print:
                    print("PEA: Region completely explored as discovered vertices are unavailable")
                # -- if robot is not in starting node, go to starting node
                if observation[0] == self.init_position:
                    break
                else:
                    if debug_print:
                        print("PEA: Agent not in starting position, go back to starting position")
                    observation = self.env.step(self.init_position)
            else:
                # -- if time remaining is unknown
                if t_r is None:
                    action = self._choose_greedy_action(observation)
                    action_cost = action[0]
                    action_node = action[1]
                elif t_r <= 0:
                    break
                else:
                    # -- if time limit is known
                    action = self._choose_greedy_action_time(observation)
                    action_node = action[1]
                    
                # -- take the action
                observation = self.env.step(action_node)
            
            # -- print the results to screen
            self._print_observation_to_file(observation)
            # -- print the results to file
            self._print_observation_to_screen(observation)
        
        # -- output the list of visited vertices to file
        self._print_visited_nodes_to_file()
        return


    def _reset_output_file(self):
        sample = open(self.output_filename, 'w') 
        print ("Agent priority filename: {}".format(self.priority_filename), file=sample)
        d = datetime.today()
        print ("Time: {}-{}-{}:{}:{}:{}".format(d.year, d.month, d.day, d.hour, d.minute, d.second), file=sample)
        print ("-"*80, file=sample)
        sample.close()


    def _choose_greedy_action(self, observation:list, debug_print:bool = False):
        """
        Step 1: Get current state, local and remote states available.
        Step 2: Remove local and remote states that are visited.
        Step 3: Add nodes to priority queue
        """
        # -- decompose from observation
        cur_state = observation[0]
        discovered_states = observation[1]
        
        # -- initialize priority queue
        lowest_cost_action = []

        # -- print the discovered states
        print ("Discovered states: ", discovered_states)
        
        # -- add local and remote vertices to priority queue
        self._add_nodes_priority_queue_no_time_limit(discovered_states, lowest_cost_action)
        
        if debug_print:
            print ("PEA: priority queue by agent: ", lowest_cost_action)
        
        # -- fetch the cheapest action
        action = heapq.heappop(lowest_cost_action)
        action = (action[0], action[1], action[2].val)
        if debug_print:
            print ("PEA: Actions chosen by agent: ", action)
        
        # -- return the action
        return (action[1], action[2])

    
    def _choose_greedy_action_time(self, observation: list, debug_print: bool=False):
        """ Chooses a greedy action from discovered states available with a deadline
        """
        cur_state = observation[0]
        discovered_states = observation[1]
        t_r = observation[2] # time remaining
        home_node = self.init_position
        
        # -- get distance to home
        dist_to_home = self.env.get_all_distances_from_node(home_node, debug_print=False)
        
        
        # -- flag return possible to True
        flag_return_possible = True
        
        if debug_print:
            print ("PEA: _choose_greedy_action_time:: Get distance from home node: {} to current node: {}".format(home_node, cur_state))
            print ("PEA: _choose_greedy_action_time:: All Distances from home node: ")
            print (dist_to_home)
            print ("-"*80)

        # -- initialize the priorityQueue
        lowest_cost_action_pqueue = []
        
        # -- get distance from current to remote node of starting location
        distance_current_home = dist_to_home[cur_state]
        action_home = (distance_current_home, home_node)
        action_stay = (0.0, cur_state)

        if debug_print:
            print ("PEA: _choose_greedy_action_time:: distance from home: ", distance_current_home)
            print ("PEA: _choose_greedy_action_time:: time remaining: ", t_r)
            print ("PEA: _choose_greedy_action_time:: Action home: ", action_home)
            print ("PEA: _choose_greedy_action_time:: Action stay: ", action_stay)
        
        if distance_current_home == t_r: # if cost to home is equals time remaining
            if debug_print:
                print ("PEA: _choose_greedy_action_time:: distance current home = t_r")
            return action_home
        elif (distance_current_home + 1) == t_r: # if cost to home is equals time remaining
            if debug_print:
                print ("PEA: _choose_greedy_action_time:: distance current home +1 = t_r")
            return action_home
        elif distance_current_home > t_r: # if cost to home is greater than time remaining
            if debug_print:
                print ("PEA: _choose_greedy_action_time:: Return home is not possible")
                print ("                                  exploring the rest of the environment")
            flag_return_possible = False
            # Return home is NOT POSSIBLE
            # -- get the vertex to visit without deadline
            self._add_nodes_priority_queue_no_time_limit(discovered_states, lowest_cost_action_pqueue)
            while True:
                if len(lowest_cost_action_pqueue) > 0:
                    action = heapq.heappop(lowest_cost_action_pqueue)
                    if action[1] <= t_r: # the vertex can be visited
                        return (action[1], action[2].val)
                else: # robot stays in position
                    return action_stay 
        else: # cost to home is less than time remaining
            # Return to home IS POSSIBLE
            # -- add discovered vertices to priority queue
            self._add_nodes_priority_queue_with_time_limit(discovered_states, dist_to_home, lowest_cost_action_pqueue, t_r)
            
            if debug_print:
                print ("PEA: _choose_greedy_action_time:: priority queue by agent: ", lowest_cost_action_pqueue)
            
            # if the length of the lowest_cost_action_pqueue is zero, then the robot 
            # did not find any possible vertices to visit
            if len(lowest_cost_action_pqueue) == 0:
                # -- return is not possible to home
                if flag_return_possible == True:
                    return action_home
            
            # -- fetch the cheapest action
            action = heapq.heappop(lowest_cost_action_pqueue)
            action = (action[0], action[1], action[2], action[3].val) # remove the wrapper

        # -- if action is generated to stay in place to or to go back home
        if len(action) == 2:
            return (action[0], action[1])
        # -- if action is generated to go to any other node
        else:
            if debug_print:
                print ("PEA: choose_greedy_action_time_limit::  Action selected: {}".format(action))
                print ("PEA: choose_greedy_action_time_limit::  Cost: {}".format(action[1]))
                print ("PEA: choose_greedy_action_time_limit::  return: ({},{})".format(action[1], action[3]))
            return (action[1], action[3])


    def _remove_visited_vertices(self, states):
        """ Remove visited vertices from states
        """
        # -- Initialize
        del_idx = []

        # -- identify index positions to remove
        for node_idx in states.items():
            if node_idx in self.visited_nodes:
                del_idx.append(node_idx)
        
        # -- delete the vertices
        for node_idx in del_idx:
            del(states[node_idx])
    

    def _add_nodes_priority_queue_no_time_limit(self, states, priority_queue):
        """ Creates a priority queue with vertices (when deadline is not available)
        Node priorities:
        1. Priority of the vertex
        2. Cost to travel to vertex 
        3. Wrapped vertex_idx (so that node_id comparisons are not made)

        Arguments:
            states: <dict> (key: vertex_idx(int), val: cost to state from current vertex)
            priority_queue: <list<int>> priority queue of tuples with items on list above
        """
        for node_idx, node_cost in states.items():
            node_type = self.env.get_node_type(node_idx)
            heapq.heappush(priority_queue, (self.node_priority[node_type], node_cost, Wrapper(node_idx)))
        return
    
    
    def _add_nodes_priority_queue_with_time_limit(self, states, dist_to_home, priority_queue, t_r):
        """ Creates a priority queue with vertices (when deadline is available)
        Makes a min heap (to create a priority queue) based on 3 priority values: 
        (The 4th priority value is included to keep a track of the node_idx. However, a wrapper class
        is used so that the comparison against node indices are considered as random True/False. This
        eliminates priority due to insertion order or node index.)
        Heapq: https://docs.python.org/3/library/heapq.html
        FUTURE TODO: If using Python 3.7, upgrade wrapper class to dataclass
        Priority 1. priority of vertex (highest priority)
        Priority 2. node cost (current -> target)
        Priority 3. cost to return back home (target->home)
        Priority 4. Wrapped vertex_idx to avoid comparisons in (so that vertex_idx comparisons are not made)

        Arguments:
            states: <dict> (key: vertex_idx(int), val: cost to state from current vertex)
            dist_to_home: <dict> (key: vertex_idx(int), val: cost from home)
            priority_queue: <list<int>> priority queue of tuples with items on list above
            t_r: time remaining for exploration
        """
        for node_idx, node_cost in states.items():
            cost_new_node_back_home = node_cost + dist_to_home[node_idx]
            if cost_new_node_back_home <= t_r:
                node_type = self.env.get_node_type(node_idx)
                heapq.heappush(priority_queue, (self.node_priority[node_type], node_cost, dist_to_home[node_idx], Wrapper(node_idx)))
        return


    def _read_node_priority(self, filename):
        """ Reads node priority for an exploration agent.
        """
        node_priority = {}
        with open(filename) as fp:
            while True:
                line = fp.readline()
                line = line.strip()
                if not line:
                    break
                # -- split line
                elems = line.split()
                node_type = elems[0]
                node_priority[node_type] = float(elems[1])
        return node_priority


    def _print_observation_to_screen(self, observation):
        print ("*="*40)
        print ("PEA: Current State: ", observation[0])
        print ("PEA: Discovered States: ", observation[1])
        print ("PEA: time remaining: ", observation[2])
        print ("PEA: cost total: ", observation[3])
        print ("*="*40)


    def _print_observation_to_file(self, observation):
        sample = open(self.output_filename, 'a') 
        print ("PEA: Current State: ", observation[0], file=sample)
        print ("PEA: Discovered States: ", len(observation[1]), file=sample)
        print (self._dictionary_to_string(observation[1]), file=sample)
        print ("PEA: time remaining: ", observation[2], file=sample)
        print ("PEA: cost total: ", observation[3], file=sample)
        print ("-"*80, file=sample)
        sample.close()


    def _dictionary_to_string(self, idx_cost_dict):
        str_return = ""
        for idx, cost in idx_cost_dict.items():
            str_return = "{}{} {}\n".format(str_return, idx, cost)
        str_return = str_return[0:-1]
        return str_return


    def _print_visited_nodes_to_file(self):
        sample = open(self.output_filename, 'a')
        for node in self.visited_nodes:
            print (node, end=' ', file=sample)
        print ("",file=sample)
        sample.close()

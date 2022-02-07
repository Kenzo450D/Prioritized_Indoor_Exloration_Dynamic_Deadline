#!/usr/bin/env python3
import numpy as np
import sys
import heapq
from datetime import datetime
from int_wrapper import Wrapper
from collections import defaultdict

class PriorityExplorationAlgorithm:
    def __init__(self, vertex_priority_file_name, output_filename, init_position, env):
        """ Initialize priority exploration algorithm with static variables
        """
        self.home_vertex = init_position
        self.output_filename = output_filename
        self.env = env #instance of class weighted_graph_env


        # -- initialize visited nodes
        self.visited_nodes = []
        self.all_time_limits = []

        # -- initialize vertex priority
        self.vertex_priority_file_name = vertex_priority_file_name
        self.node_priority = self._read_node_priority_file(vertex_priority_file_name)

        # -- reset output file
        self._reset_output_file()


    def get_node_type(self, node_idx):
        return self.env.node_tags[node_idx] # three types (corridor, large room, small room)


    def _read_node_priority_file(self, filename):
        """ Read vertex priority from file
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


    def getTargetVertex(self, deadline, current_vertex, visited_vertices, explored_graph, debug_print = False):
        print("Function: getTargetVertex")
        print("Time remaining: ", deadline)
        print("Current robot position: ", current_vertex)
        print("Visited vertices: ", visited_vertices)
        print("Explored_graph: ", explored_graph)
        print("Debug print (optional parameter): ", debug_print)
        
        # -- check if deadline is imposed
        if deadline == None:
            return self.choose_greedy_action_no_deadline(current_vertex, visited_vertices, explored_graph)
        else:
            return self.choose_greedy_action_deadline(deadline, current_vertex, visited_vertices, explored_graph)


    def choose_greedy_action_no_deadline(self, current_vertex, visited_vertices, explored_graph, debug_print=False):
        """
        Step 1: Remove visited nodes from the explored graph
        Step 2: Add nodes to priority queue (heap)
        Step 3: Fetch the cheapest action
        Step 4: Return the action
        """
        print("Function: choose_greedy_action_deadline")
        # -- get cost to all vertices
        #from_node, explored_graph, debug_print
        min_distances_to_v, source_of_v = self._get_cost_to_all_vertices(current_vertex, explored_graph)

        # -- remove the visited vertices
        self._remove_visited_vertices(min_distances_to_v, visited_vertices)

        # -- add vertices to priority queue
        lowest_cost_action_priority_queue = []
        self._add_nodes_priority_queue_no_time_limit(min_distances_to_v, lowest_cost_action_priority_queue)
        print ("Lowest Cost Action: ", lowest_cost_action_priority_queue)
        
        # -- fetch the action
        action = heapq.heappop(lowest_cost_action_priority_queue)
        print ("Action fetched: ", action)
        action = (action[1], action[2].val)
        return action


    def choose_greedy_action_deadline(self, deadline, current_vertex, explored_graph, debug_print=False):
        """ Chooses a greedy action based on the priority order
        Step 1: Remove visited nodes from the explored graph
        Step 2: Add nodes to priority queue (heap)
        Step 3: Fetch the cheapest action
        Step 4: Return the action
        """
        print("Function: choose_greedy_action_no_deadline")
        # -- initialize
        flag_return_possible = True
        
        # -- get cost to all vertices
        min_distance_to_cur_vertex, source_of_v = self._get_cost_to_all_vertices(current_vertex, explored_graph)
        
        # -- remove the visited vertices
        self._remove_visited_vertices(min_distance_to_cur_vertex, visited_vertices)
        
        # -- get cost to all vertices from home
        min_distance_to_home, source_of_home = self._get_cost_to_all_vertices(self.home_vertex, explored_graph)
        
        # -- add vertices to priority queue
        # -- initialize the priorityQueue
        lowest_cost_action_queue = []
        
        # -- corner check: If time to return home is equals to deadline
        distance_current_home = min_distance_to_home[current_vertex]
        action_home = (distance_current_home, home_vertex)
        action_stay = (0.0, current_vertex)
        if distance_current_home == self.env.t_r: # if time remaining is exactly equals to time to go back home
            return action_home
        elif (distance_current_home + 1 == self.env.t_r): #time remaining should be at least 2 timesteps to visit any other vertex
            return action_home
        if self.env.t_r < 2: # robot does not have adequate time to explore other vertices
            return action_stay 
        elif (distance_current_home + 1) > self.env.t_r:
            flag_return_possible = False
            # Return home is NOT POSSIBLE
            # -- get the vertex to visit without deadline
            self._add_nodes_priority_queue_no_time_limit(min_distance_to_cur_vertex, lowest_cost_action_queue)
            while (True):
                if len(lowest_cost_action_queue) > 0:
                    action = heapq.heappop(lowest_cost_action_queue)
                    if action[1] <= self.env.t_r: # the vertex can be visited
                        return (action[1], action[2].val) # robot visits vertex without chance of heading home
                else:
                    return action_stay # robot stays in position
        
        # Return home IS POSSIBLE
        # -- add discovered vertices to priority queue
        self._add_nodes_priority_queue_with_time_limit(min_distance_to_cur_vertex, min_distance_to_home, lowest_cost_action_queue, self.env.t_r)
        
        # if the length of the lowest_cost_action_queue is zero, then the robot 
        # did not find any possible vertices to visit
        if len(lowest_cost_action_queue) == 0:
            # -- return is not possible to home
            if flag_return_possible == True:
                return action_home
                
            
        action = heapq.heappop(lowest_cost_action_queue)
        action = (action[1], action[3].val) #action[3] = vertex idx (from Wrapper class)
        print ("PEA: choose_greedy_action_time_limit::  Action selected: {}".format(action))
        
        return action


    def _remove_visited_vertices(self, min_distances_to_v, visited_vertices):
        """ Removes visited vertices from dictionary
        Input: min_distances_to_v: <dict> key: vertex ID, val: cost from current vertex
        """    
        for v in visited_vertices:
            if v in min_distances_to_v:
                del min_distances_to_v[v]
        return


    def _reset_output_file(self):
        """ Resets the output file to clear of previous output
        """
        sample = open(self.output_filename, 'w') 
        print ("Agent priority filename: {}".format(self.vertex_priority_file_name), file=sample)
        d = datetime.today()
        print ("Time: {}-{}-{}:{}:{}:{}".format(d.year, d.month, d.day, d.hour, d.minute, d.second), file=sample)
        print ("-"*80, file=sample)
        sample.close()


    def _add_nodes_priority_queue_no_time_limit(self, states, q):
        """ Makes a heap based on 3 priority values: node priority, cost to travel to node, node index
        Input:
        1. states: (both the local states and remote states)
        2. q: The queue (an empty array)
        """
        for node_idx, node_cost in states.items():
            node_type = self.get_node_type(node_idx)
            heapq.heappush(q, (self.node_priority[node_type], node_cost, Wrapper(node_idx)))
        
        return


    def _add_nodes_priority_queue_with_time_limit(self, states, dist_to_home, q, t_r):
        """Makes a priority queue based on 3 priority values
        Priority 1: Priority of vertex
        Priority 2: Node cost from current vertex
        Priority 3: Distance to home vertex'
        Priority 4: Wrapped Node ID (So that the node value is not compared with one another)
        Input:
            states: dictionary of discovered vertex tuples (key: node_idx, val: node_cost)
            dist_to_home: <dictionary> key: vertex_index, val: cost to home
            q: <List> Priority queue to populate heap
            t_r: <int> time remaining to complete the exploration
        """
        for node_idx, node_cost in states.items():
            cost_new_node_back_home = node_cost + dist_to_home[node_idx]
            if cost_new_node_back_home <= t_r:
                node_type = self.env.get_node_type(node_idx)
                heapq.heappush(q, (self.node_priority[node_type], node_cost, dist_to_home[node_idx], Wrapper(node_idx)))
        return


    def _get_cost_to_all_vertices(self, from_node, explored_graph, debug_print = True):
        """ Get cost to all vertices from vertex and in explored graph
        Uses dijkstra's algorithm to get the distance too all the vertices
        ------------------------------------------------------------------------
        Input:
        from_node: Distance from the source node to where all the node distances
                   are calculated
        explored_graph: The explored graph as a dictionary
        debug_print: print the nodes to debug
        """
        if debug_print:
            print(f"prioritized_greedy_exploration_algorithm:: _get_cost_to_all_vertices :: from_node: {from_node}")
        
        # -- Initialize
        min_distances_to_v = defaultdict(lambda: np.inf)
        min_distances_to_v [from_node] = 0
        source_of_v = defaultdict(lambda: from_node)
        source_of_v[from_node] = from_node
        lowest_distance_priority_queue = []

        # -- mark visited nodes
        visited_nodes = []
        visited_nodes.append(from_node)
        if debug_print:
            print ("PEA: _get_cost_to_all_vertices:: Visited Nodes: ", visited_nodes)
            print ("PEA: _get_cost_to_all_vertices:: Discovered Nodes: ", explored_graph.keys())
        
        nsteps = 0
        last_node = None
        cur_node = from_node
        cost_to_cur_node = 0
        # --  loop
        if debug_print:
            print ("PEA: _get_cost_to_all_vertices:: While Loop -- ")
        while True:
            if debug_print:
                print ("\tBack to start of loop...")
            # -- get the nodes nearby
            # if the node is discovered node, and not visited, do not give details of its connections
            if cur_node not in explored_graph:
                if debug_print:
                    print (f"\tNode: {cur_node} is not in explored nodes")
                    print (f"\tExplored Nodes: {explored_graph}")
                nodes_nearby = []
            else:
                nodes_nearby = explored_graph[cur_node]
            
            # BEGIN debug print
            if debug_print:
                print("\tcur_node: ", cur_node)
                print("\tnodes_nearby: ", nodes_nearby)
            # END debug print
            
            # -- loop through nearby nodes
            for node in nodes_nearby:
                # BEGIN debug print
                if debug_print:
                    print (f"\t\tNode: {node}\t Node Idx: {node[0]}\tCost To reach: {node[1]}")
                # END debug print
                
                # -- get index and cost to node
                nodeIdx = node[0]
                node_cost = node[1]
                
                # check if cost to node (distance to current node + node cost)
                # is less than minimum recorded distance
                distance_to_nodeIdx = min_distances_to_v[cur_node] + node_cost
                
                # BEGIN debug print
                if debug_print:
                    print (f"\t\t Cost to Reach from node <{from_node}> :: {distance_to_nodeIdx}")
                # END debug print
                
                # -- if cost is lower than current minimum cost, update
                if distance_to_nodeIdx < min_distances_to_v[nodeIdx]:
                    min_distances_to_v[nodeIdx] = distance_to_nodeIdx
                    source_of_v[nodeIdx] = cur_node
                    # -- add to queue
                    heapq.heappush(lowest_distance_priority_queue, (distance_to_nodeIdx, nodeIdx))
                    
            # -- get the lowest cost node
            if len(lowest_distance_priority_queue) == 0:
                break
            last_node = cur_node
            cur_node = heapq.heappop(lowest_distance_priority_queue)[1]
            
            # BEGIN debug print
            if debug_print:
                print ("\tcur_node: ", cur_node)
            # END debug print
            
            # -- add to visited nodes
            if cur_node not in visited_nodes:
                if debug_print:
                    print (f"\tNode {cur_node}, not in Visited Nodes")
                visited_nodes.append(cur_node)
            
            # BEGIN debug print
            if debug_print:
                print ("\tCurrent Node: ", cur_node)
                print ("\tVisited Nodes: ", visited_nodes)
                print ("\tDistances: ", dict(min_distances_to_v))
                print ("\tCurrent Queue: ", lowest_distance_priority_queue)
                print ("\t","- "*30)
            # END debug print
            nsteps += 1
        
        # BEGIN debug print
        if debug_print:
            print ("PEA: _get_cost_to_all_nodes:: All remote nodes visited!")
            print ("PEA: _get_cost_to_all_nodes:: Minimum distances are: ", dict(min_distances_to_v))
            print ("PEA: _get_cost_to_all_nodes:: Sources of vertices are: ", dict(source_of_v))
            print ("PEA: _get_cost_to_all_nodes:: Return back to control")
            print ("="*100)
        # END debug print
        return dict(min_distances_to_v), dict(source_of_v)

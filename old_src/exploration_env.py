from collections import defaultdict
from copy import deepcopy
import sys
# -- local import
from env_graphs import WeightedGraphEnv
from env_clock import ExplorationClock
"""
This file handles the entire environment and keeps a track of the explored
graph. It provides the agent with the current state of the robot.
It keeps a track of the visited and seen nodes, through which it can create
this state".

So to write it in steps:
 1. Read the file which contains the edges, and nodes, and make the ground 
    truth and pruned graph
 2. Initialize the explored graph
 2. Initialize the robot to a position
    Consider the neighboring vertices as seen, and the current vertex as visited.
 3. Create the state of robot considering local and remote vertices in each direction. 
    Return to the agent a set of local and remote vertices, each towards a neighboring vertex. So if a vertex has 5 neighbors, it should report this information:
     a. Local: (Neighboring Node, Cost)
        Remote: (Neighboring Node, Remote Node, Cost)
        To represent this information we write:
        Neighboring_Node: {'lcost':local_cost, 'r_node':Remote_Node, 'r_cost:',remote_cost}
    b. the agent returns the neighboring node which the agent plans to head towards.
 4. Update the explored graph, 
"""


class GraphExplorationEnvironment:
    def __init__(self, 
                 edgeFileName,
                 vertexTagFileName,
                 vertexPriorityFileName,
                 fixed_tw, fixed_tb,
                 initial_robot_position):
        
        # -- set up filenames
        self.vertexTagFileName = vertexTagFileName
        self.filename = edgeFileName
        self.vertexPriorityFileName = vertexPriorityFileName
        
        # -- initialize the graphs
        #print ("EXPLORATION ENV: edge file: ", edgeFileName)
        self.env = WeightedGraphEnv(edgeFileName, vertexTagFileName)
        
        # -- set up the initial robot position
        self.initial_robot_position = initial_robot_position
        self.current_robot_position = initial_robot_position
        self.env.set_up_initial_robot_position(self.initial_robot_position)
        
        # -- set up timers
        self.tw = fixed_tw
        self.tb = fixed_tb
        self._set_up_clock()
        
        # -- set up cost
        self.cost_incurred = 0
    
    def _set_up_clock(self):
        self.exploration_clock = ExplorationClock(self.tw, self.tb)
    

    def get_state_consolidated(self):
        cur_pos = self.current_robot_position

        # -- the current state is a combination of local states and remote nodes towards a direction
        local_states = self.env.get_local_unvisited_nodes_gt(cur_pos)
        print ("EXPLORATION ENV: GET_STATE_CONSOLIDATED: Local states: ", local_states)
        local_states = self._convert_local_states_list_to_dict(local_states)
        print ("EXPLORATION ENV: GET_STATE_CONSOLIDATED: Local state dictionary: ", local_states)
        remote_states = self.env.get_remote_nodes(cur_pos)
        print ("EXPLORATION ENV: GET_STATE_CONSOLIDATED: Remote states: ", remote_states)
        
        # -- get all discovered states
        discovered_states = self.env.get_discovered_nodes(cur_pos)
        self.discovered_states, self.source_discovered_states = discovered_states
        print ("EXPLORATION ENV: GET_STATE_CONSOLIDATED: Discovered states: ", discovered_states)
        # input("Continue?")

        # -- save to class variables
        self.local_states = local_states
        self.remote_states, self.source_remote_states = remote_states
        
        # -- return the state
        return (cur_pos, self.discovered_states, self.exploration_clock.get_time_limit(), self.cost_incurred)
        

    # def get_state(self):
    #     cur_pos = self.current_robot_position
        
    #     # -- the current state is a combination of local states and remote nodes towards a direction
    #     local_states = self.env.get_local_unvisited_nodes_gt(cur_pos)
    #     print ("Local states: ", local_states)
    #     local_states = self._convert_local_states_list_to_dict(local_states)
    #     print ("Local state dictionary: ", local_states)
    #     remote_states = self.env.get_remote_nodes(cur_pos)
    #     print ("Remote states: ", remote_states)
        
    #     # -- save to class variables
    #     self.local_states = local_states
    #     self.remote_states, self.source_remote_states = remote_states
        
    #     # -- return the state
    #     return (cur_pos, local_states, self.remote_states, self.exploration_clock.get_time_limit(), self.cost_incurred)
    
    def _convert_local_states_list_to_dict(self, local_states):
        local_state_dict = {}
        for node in local_states:
            local_state_dict[node[0]] = node[1]
        return local_state_dict
    
    def get_exploration_graph(self):
        # Create deep copy so that accidental changes do not affect the internal graph
        ge = deepcopy(self.env.graph_explored)
        return ge
    
    
    def get_node_type(self, node_idx):
        return self.env.node_tags[node_idx]
    
    def step(self, goto_node, debug_print = False):
        # -- print the goto node
        if debug_print:
            print ("EXPLORATION ENV: step :: Node selected to travel: ", goto_node)
            print ("EXPLORATION ENV: step :: Current node", self.current_robot_position)
        cost = 0
        
        node_reached = None
        if self.current_robot_position == goto_node:
            # -- robot is trying to reach the same robot
            print("EXPLORATION ENV: step :: Robot is trying to back to the same node ", goto_node)
            time_remaining = self.exploration_clock.update_time_limit()
            return self.get_state_consolidated()
        elif goto_node in self.local_states.keys():
            # -- robot is trying to reach a local node
            print("EXPLORATION ENV: step :: Robot headed for local node")
            node_reached, cost = self.env.go_local_node(goto_node)
            # -- increment total cost
            self.cost_incurred += cost
        elif goto_node in self.remote_states.keys():
            # -- robot is trying to reach a remote node
            print("EXPLORATION ENV: step :: Robot headed for remote node {}".format(goto_node))
            node_reached, cost = self.env.go_remote_node(self.current_robot_position,goto_node)
            # -- increment total cost
            self.cost_incurred += cost
        elif goto_node == self.initial_robot_position:
            # -- robot is trying to reach home
            print("EXPLORATION ENV: step :: Robot headed for home node")
            node_reached, cost = self.env.go_remote_node(self.current_robot_position,goto_node)
            # -- increment total cost
            self.cost_incurred += cost
        else:
            # -- robot is trying to go for a node that is visited
            # this should be an error state
            print ("EXPLORATION_ENV: Function step::")
            print ("ERROR: Target vertex is not among discovered vertices.")
            print ("\tTarget vertex not accepted")
            print ("\tTarget Vertex:  ", goto_node)
            print ("\tCurrent vertex: ", self.current_robot_position)
            raise ValueError
        
        # -- update current node
        self.current_robot_position = node_reached
        print("EXPLORATION ENV: step :: Current robot position = ", self.current_robot_position)
        print("EXPLORATION ENV: step :: Step  Cost Incurred: ", cost)
        print("EXPLORATION ENV: step :: Total Cost Incurred: ", self.cost_incurred)
        
        # -- update time limit
        if cost == 0:  #for transfer to the same node
            time_remaining = self.exploration_clock.update_time_limit()
        while (cost != 0):
            time_remaining = self.exploration_clock.update_time_limit()
            cost = cost -1
        print ("EXPLORATION ENV: step:: Time Remaining = ", time_remaining)
        return self.get_state_consolidated()
    
    def get_time_limit(self):
        return self.exploration_clock.get_time_limit()
    
    def get_distance_to_node(self, nodeIdx):
        return deepcopy(self.env.distance_all_nodes[nodeIdx])
    
    def get_all_distances_from_home(self, debug_print = True):
        node = self.initial_robot_position
        return self.get_all_distances_from_node(self.initial_robot_position, debug_print)
    
    def get_all_distances_from_node(self, node, debug_print = False):
        """ Returns a dictionary of all distances of other nodes from the input node.
        """
        dist_all_nodes, source_all_nodes = self.env.get_all_nodes_with_path_cost(node)
        if debug_print:
            print ("EXPLORATION ENV: get_all_distances_from_node {}".format(node))
            print ("\tDistance to nodes: {}".format(dist_all_nodes))
        return deepcopy(dist_all_nodes)
    
    def get_unvisited_distances_from_node(self, node, debug_print = False):
        dist_unvisited_nodes, source_unvisited_nodes = self.env.get_all_unvisited_nodes_with_path_cost(node)
        if debug_print:
            print ("EXPLORATION ENV: get_unvisited_distances_from_node {}".format(node))
            print ("\t Distance to unvisited nodes: {}".format(dist_unvisited_nodes))
        return deepcopy(dist_unvisited_nodes)
    
    
        
        
        
        

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
                 edge_file_name,
                 vertex_tag_file_name,
                 vertex_priority_file_name,
                 time_before_whistle, time_after_whistle,
                 initial_robot_position):
        """ Graph Exploration Environment: Creates and maintains the graph exploration environment.
        Input:
            edge_file_name: <string> filename to get the edge information
            vertex_tag_file_name: <string> filename to get the vertex tags (corridor/small_room/large_room)
            time_before_whistle: <int> time after which the deadline is provided to the robot
            time_after_whistle: <int> deadline provided to robot
        """

        # -- set up filenames
        self.vertexTagFileName = vertex_tag_file_name
        self.filename = edge_file_name
        self.vertexPriorityFileName = vertex_priority_file_name

        # -- initialize the graphs
        print("EXPLORATION ENV: edge file: ", edge_file_name)
        self.env = WeightedGraphEnv(edge_file_name, vertex_tag_file_name, vertex_priority_file_name)

        # -- set up the initial robot position
        self.initial_robot_position = initial_robot_position
        self.current_robot_position = initial_robot_position
        self.env.set_up_initial_robot_position(self.initial_robot_position)

        # -- set up timers
        self.t_a = time_before_whistle
        self.t_r_0 = time_after_whistle
        self._set_up_clock()

        # -- set up cost
        self.cost_incurred = 0

    def _set_up_clock(self):
        self.exploration_clock = ExplorationClock(self.t_a, self.t_r_0)

    def get_state(self):
        cur_pos = self.current_robot_position
        print("Exploration environment: Get State: Cur_pos: ", cur_pos)

        # -- the current state is a combination of local states and remote nodes towards a direction
        # discovered_vertices = self.env.get_discovered_vertices(cur_pos)
        # local_states = self.env.get_local_nodes_gt(cur_pos)
        local_states = self.env.get_local_unvisited_nodes_gt(cur_pos)
        local_states = self._convert_local_states_list_to_dict(local_states)
        remote_states = self.env.get_remote_nodes(cur_pos)
        print("EXPLORATION ENV: get_state :: Remote States: {}".format(remote_states))
        self.local_states = local_states
        self.remote_states, self.source_remote_states = remote_states

        return (cur_pos, local_states, self.remote_states, self.exploration_clock.get_time_limit(), self.cost_incurred)

    def _convert_local_states_list_to_dict(self, local_states):
        local_state_dict = {}
        for node in local_states:
            local_state_dict[node[0]] = node[1]
        return local_state_dict

    def get_node_type(self, node_idx):
        return self.env.node_tags[node_idx]

    def step(self, goto_node, debug_print=True):
        # -- print the goto node
        if debug_print:
            print("EXPLORATION ENV: step :: Node selected to travel: ", goto_node)
            print("EXPLORATION ENV: step :: Current node", self.current_robot_position)
        cost = 0

        node_reached = None
        if self.current_robot_position == goto_node:
            # -- robot is trying to reach the same robot
            print("EXPLORATION ENV: step :: Robot is trying to back to the same node ", goto_node)
            time_remaining = self.exploration_clock.update_time_limit()
            return self.get_state()
        elif goto_node in self.local_states.keys():
            # -- robot is trying to reach a local node
            print("EXPLORATION ENV: step :: Robot headed for local node")
            node_reached, cost = self.env.go_local_node(goto_node)
            # -- increment total cost
            self.cost_incurred += cost
        elif goto_node in self.remote_states.keys():
            # -- robot is trying to reach a remote node
            print("EXPLORATION ENV: step :: Robot headed for remote node {}".format(goto_node))
            node_reached, cost = self.env.go_remote_node(self.current_robot_position, goto_node)
            # -- increment total cost
            self.cost_incurred += cost
        elif goto_node == self.initial_robot_position:
            # -- robot is trying to reach home
            print("EXPLORATION ENV: step :: Robot headed for home node")
            node_reached, cost = self.env.go_remote_node(self.current_robot_position, goto_node)
            # -- increment total cost
            self.cost_incurred += cost
        else:
            # -- robot is trying to go for a node that is visited
            # this should be an error state
            print("EXPLORATION_ENV: Function step::")
            print("ERROR: Target vertex is not among discovered vertices.")
            print("\tTarget vertex not accepted")
            print("\tTarget Vertex:  ", goto_node)
            print("\tCurrent vertex: ", self.current_robot_position)
            raise ValueError

        # -- update current node
        self.current_robot_position = node_reached
        print("EXPLORATION ENV: step :: Current robot position = ", self.current_robot_position)
        print("EXPLORATION ENV: step :: Step  Cost Incurred: ", cost)
        print("EXPLORATION ENV: step :: Total Cost Incurred: ", self.cost_incurred)
        # -- NOTE: exit
        # print ("Code exit at line 98 exploration_env.py")

        # -- update time limit
        if cost == 0:  # for transfer to the same node
            time_remaining = self.exploration_clock.update_time_limit()
        while (cost != 0):
            time_remaining = self.exploration_clock.update_time_limit()
            cost = cost - 1
        print("EXPLORATION ENV: step:: Time Remaining = ", time_remaining)
        # sys.exit(0)
        return self.get_state()

    def get_time_limit(self):
        return self.exploration_clock.get_time_limit()

    def get_distance_to_node(self, nodeIdx):
        return deepcopy(env.distance_all_nodes[nodeIdx])

    def get_all_distances_from_home(self, debug_print=True):
        node = self.initial_robot_position
        return self.get_all_distances_from_node(self.initial_robot_position, debug_print)

    def get_all_distances_from_node(self, node, debug_print=True):
        dist_all_nodes, source_all_nodes = self.env.get_all_nodes_with_path_cost(node)
        if debug_print:
            print("EXPLORATION ENV: get_all_distances_from_node {}".format(node))
            print("\tDistance to nodes: {}".format(dist_all_nodes))
        return deepcopy(dist_all_nodes)

    def get_unvisited_distances_from_node(self, node, debug_print=True):
        dist_unvisited_nodes, source_unvisited_nodes = self.env.get_all_unvisited_nodes_with_path_cost(node)
        if debug_print:
            print("EXPLORATION ENV: get_unvisited_distances_from_node {}".format(node))
            print("\t Distance to unvisited nodes: {}".format(dist_unvisited_nodes))
        return deepcopy(dist_unvisited_nodes)

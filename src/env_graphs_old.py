from collections import defaultdict
from queue import PriorityQueue
import heapq
import numpy
from copy import deepcopy
import sys
import numpy as np
from random import random



"""
Data elements:
==============
graph_explored:  <defaultdict> of the nodes explored. Each node has 
                 connectivity information along with it.
seen_nodes: <list> list of seen nodes
explored_nodes: <list> list of nodes visited.

Function:
=========
set_up_initial_robot_position
-----------------------------



get_local_nodes_gt 
------------------
    get the local nodes which can be viewed from a particular node
    also comes with path cost
    list of tuple (node_id, node_cost)
    
get_local_unvisited_nodes_gt
----------------------------
    get the local nodes which can be viewed and is unvisited from a particular node
    list of tuple (node_id, node_cost)

get_remote_nodes
----------------
    get the nodes which are remote from the given node in the explored graph,
    and which are not in local nodes and which are not visited/explored

get_all_nodes_with_path_cost
----------------------------
    get the nodes which are connected form the given node in the explored graph,
    including all visited or seen nodes

get_all_unvisited_nodes_with_path_cost
--------------------------------------
    get the nodes which are connected from the given node in the explored graph,
    including local nodes and excluding visited nodes.
"""
class WeightedGraphEnv:
    """
    This class is to read the graph from the filename, 
    """
    def __init__(self,
                 edgeFileName='testEdgeFile.txt', 
                 nodeTagFileName='testVertexTagFile.txt',
                 nodePriorityFileName='node_priority.txt'):
        
        print ("Edge filename: ", edgeFileName)
        #sys.exit(0)
        
        # -- set up filenames
        self.nodeTagFileName = nodeTagFileName
        self.filename = edgeFileName
        
        # -- read the node tags
        self._read_node_tags(self.nodeTagFileName)
        
        # -- read the graph
        self._initialize_graph(edgeFileName)
        
        # -- initialize the explored graph
        self._initialize_graph_explored()
        
        
        # -- make pruned graph
        self._make_pruned_graph()
        
        # -- return
        return
        
    
    # --------------------------------------------------------------------------
    # Function: get_local_nodes_gt
    # --------------------------------------------------------------------------
    # Returns the local nodes based on the ground truth explored graph
    # --------------------------------------------------------------------------
    # Input: 
    #   node: A node of the graph environment
    # --------------------------------------------------------------------------
    def get_local_nodes_gt(self, node):
        if node not in self.graph_gt:
            print ("Function: get_local_nodes :: Error: Node <{}> not in graph".format(node))
            raise ValueError
        # -- get local nodes from graph_gt
        local_nodes = deepcopy(self.graph_gt[node])
        return local_nodes
    
    
    # --------------------------------------------------------------------------
    # Function: get_local_unvisited_nodes_gt
    # --------------------------------------------------------------------------
    # Returns the set of unvisited nodes from the local neighborhood
    # --------------------------------------------------------------------------
    # Input:
    #   node: A node of the graph environment
    #   debug_print: Prints on screen if true
    # --------------------------------------------------------------------------
    def get_local_unvisited_nodes_gt(self, node, debug_print = False):
        local_nodes = self.get_local_nodes_gt(node)
        
        del_idx = []
        for local_nodes_idx, node in enumerate(local_nodes):
            if node[0] in self.explored_nodes:
                del_idx.append(local_nodes_idx)
        
        if debug_print:
            print ("ENV GRAPH: Function: get_local_unvisited_nodes_gt :: local_nodes: {}".format(local_nodes))
            print ("ENV GRAPH: Function: get_local_unvisited_nodes_gt :: del_idx: {}".format(del_idx))
        
        for idx in sorted(del_idx, reverse=True):
            del(local_nodes[idx])
        
        # print the unvisited local nodes
        
        if debug_print:
            print ("ENV_GRAPH: GET_LOCAL_UNVISITED_NODES :: Unvisited Local Nodes ::: {}".format(local_nodes))
        return local_nodes
    
    
    # --------------------------------------------------------------------------
    # Function: get_remote_nodes
    # --------------------------------------------------------------------------
    # Returns remote nodes from the current node
    # --------------------------------------------------------------------------
    # Input:
    #   node: A node of the graph environment
    #   debug_print: Prints on screen if true
    # --------------------------------------------------------------------------
    def get_remote_nodes(self, node, debug_print = False):
        if node not in self.graph_gt:
            print ("Function: get_local_nodes :: Error: Node {} not in graph".format(node))
            raise ValueError
        # -- get remote nodes from graph_gt
        """
        we would want to fetch the remote node which is closest available.
        considering we now have a weighted graph instead of a unweighted graph, 
        we would have now have to explore the completely explored graph for each
        step.
        """
        distance_all_nodes, source_all_nodes = self._get_cost_to_all_nodes(node, debug_print)
        self.distance_all_nodes = deepcopy(distance_all_nodes)
        self.source_all_nodes = deepcopy(source_all_nodes)
        
        # BEGIN debug print
        if debug_print:
            print ("ENV GRAPHS: get_remote_nodes:: distance_all_nodes: {}".format(distance_all_nodes))
            print ("ENV GRAPHS: get_remote_nodes:: source_all_nodes: {}".format(source_all_nodes))
        # END debug print
        
        # -- filter the nodes which are local
        local_nodes = deepcopy(self.graph_gt[node])
        local_node_indices = []
        for node in local_nodes:
            local_node_indices.append(node[0])
        
        # BEGIN debug print
        if debug_print:
            print ("ENV GRAPHS: Local Nodes: ", local_nodes)
            print ("ENV GRAPHS: Local Node Indices: ", local_node_indices)
            print ("ENV GRAPHS: get_remote_nodes:: explored nodes: {}".format(self.explored_nodes))
            print ("\tLoop over the distance_all_nodes...")
        # END debug print
        
        # -- filter the nodes which are explored but not visited
        del_node = []
        for node, cost in distance_all_nodes.items():
            # BEGIN debug_print
            if debug_print:
                print ("\t\tNode: {}\tCost:{}".format(node, cost))
            # END debug_print
            if node in self.explored_nodes:
                del_node.append(node)
            if node in local_node_indices:
                del_node.append(node)
        
        # BEGIN debug print
        if debug_print:
            print ("ENV GRAPHS: get_remote_nodes:: nodes to be deleted: {}".format(del_node))
        # END debug print
        
        # -- get unique elements
        """
        as distance_all_nodes is dict, we can delete nodes without sorting 
        indices in reverse order
        """
        del_node = list(set(del_node))
        for node in del_node:
            del(distance_all_nodes[node])
            del(source_all_nodes[node])
        
        # BEGIN debug print
        if debug_print:
            print ("Function: get_remote_nodes :: Remaining node: \n\t", distance_all_nodes)
            print ("Function: get_remote_nodes :: Saved remote nodes: \n\t", self.distance_all_nodes)
            print ("Function: get_remote_nodes :: END")
            print ("--"*50)
        # END debug print
        return distance_all_nodes, source_all_nodes
    
    # --------------------------------------------------------------------------
    # Function: get_all_nodes_with_path_cost
    # --------------------------------------------------------------------------
    def get_all_nodes_with_path_cost(self, node, debug_print = True):
        if debug_print:
            print ("Function: get_all_remote_nodes:: Begin here")
            print ("Function: get_all_remote_nodes:: Node: {}".format(node))
        distance_all_nodes, source_all_nodes = self._get_cost_to_all_nodes(node, debug_print)
        return distance_all_nodes, source_all_nodes
    
    
    def get_all_unvisited_nodes_with_path_cost(self, node, debug_print = True):
        distance_all_nodes, source_all_nodes = self._get_cost_to_all_nodes(node, debug_print)
        
        # -- filter the nodes which are seen but not visited
        del_node = []
        for node, cost in distance_all_nodes.items():
            if debug_print:
                print ("\t\tNode: {}\tCost:{}".format(node, cost))
            if node in self.explored_nodes:
                del_node.append(node)
        
        if debug_print:
            print ("ENV GRAPHS: get_all_unvisited_nodes_with_path_cost:: nodes to be deleted: {}".format(del_node))
        
        # -- get unique elements
        del_node = list(set(del_node))
        # as distance_all_nodes is dict, we can delete nodes without sorting 
        # indices in reverse order
        for node in del_node:
            del(distance_all_nodes[node])
            del(source_all_nodes[node])
        
        return distance_all_nodes, source_all_nodes
    
    def go_local_node(self, node_idx, debug_print = True):
        if debug_print:
            print ("ENV GRAPHS:  GO LOCAL NODE: explored nodes before update: {}".format(self.explored_nodes))
            print ("ENV GRAPH: GO LOCAL NODE: update explored graph with node: {}".format(node_idx))
        self.update_graph_explored(node_idx)
        if debug_print:
            print ("ENV GRAPHS:  GO LOCAL NODE: explored nodes before update: {}".format(self.explored_nodes))
            print ("ENV GRAPHS: GO LOCAL NODE: distance_all_nodes: {}".format(self.distance_all_nodes))
        # return the current node and the cost incurred to reach there
        return node_idx, self.distance_all_nodes[node_idx]
    
    # --------------------------------------------------------------------------
    # Function: go_remote_node
    # --------------------------------------------------------------------------
    # Input:
    #   current_node:
    #   node_idx:
    #   step_flag
    #   debug_print
    # --------------------------------------------------------------------------
    # Output:
    #   Updates the graph explored
    #   Returns the node_idx, distance to all nodes from the node idx
    # --------------------------------------------------------------------------
    # Description:
    #
    # --------------------------------------------------------------------------
    def go_remote_node(self, current_node, node_idx, step_flag=True, debug_print=True):
        if debug_print:
            print("ENV GRAPH: go_remote_node:: current_node: {} \tGoal Node: {}".format(current_node, node_idx))
            print ("\tStep Flag is set to {}".format(step_flag))
        if step_flag:
            # -- trace back the vertex to the one closest to the current vertex.
            # -- consider one step after that.
            # Step 1. Backtrack
            source_node = node_idx
            path = []
            while (source_node != current_node):
                path.insert(0,source_node)
                source_node = self.source_all_nodes[source_node]
            # -- once we get source node as the current node, we would break 
            # from the while loop. The path would not contain the source_node
            # the cost of the tour would be the path till the first element
            # of the next node, and we would return the cost to the agent
            # -- print the distance to all nodes
            if debug_print:
                print ("ENV GRAPH: go_remote_node:: Path: ", path)
            if len(path) == 0:
                # the current node is the same as goal node
                return source_node, 1
            
            if debug_print:
                print ("ENV GRAPH: go_remote_node:: Distance to all nodes: \n")#, self.distance_all_nodes)
                for dist_node_idx, dist_node_cost in self.distance_all_nodes.items():
                    print("\tNode: {}\tCost: {}".format(dist_node_idx,dist_node_cost ))
            
            cost = self.distance_all_nodes[path[0]]
            
            if debug_print:
                print ("ENV GRAPH: go_remote_node:: Path generated: {}".format(path))
                print ("ENV GRAPH: go_remote_node:: Cost to reach first node {} : {}".format(path[0], cost))
                print ("ENV GRAPH: go_remote_node:: Update graph explored to: {}".format(path[0]))
            
            # -- update the graph explored
            self.update_graph_explored(path[0])
            return path[0], cost
                
        else:
            self._go_remote_node_directly(node_idx)
            return node_idx, self.distance_all_nodes[node_idx]
        
    
    def _go_remote_node_directly(self, node_idx):
        self.update_graph_explored(node_idx)
        return
        
    # --------------------------------------------------------------------------
    # Function: _get_cost_to_all_nodes
    # --------------------------------------------------------------------------
    # get cost to all nodes in the explored graph. Uses dijkstra's algorithm
    # to get the distance too all the vertices
    # --------------------------------------------------------------------------
    # Input:
    #   from_node: Distance from the source node to where all the node distances
    #              are calculated
    #   debug_print: print the nodes to debug
    # --------------------------------------------------------------------------
    def _get_cost_to_all_nodes(self, from_node, debug_print = True):
        """
        Step 1: 
            Initialize two default dictionaries. Minimum Distance and Source.
            min_distances_to_v: stores the current minimum distance from the 
                                source vertex to any vertex in the graph
            source_of_v: stores the previous vertex to reach that particular 
                         vertex
        Step 2:
            Mark visited nodes
        Step 3:
            
            
        """
        if debug_print:
            print ("ENV GRAPH: _get_cost_to_all_nodes :: from_node: {}".format(from_node))
        
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
            print ("ENV_GRAPH: _get_cost_to_all_nodes:: Visited Nodes: ", visited_nodes)
            print ("ENV_GRAPH: _get_cost_to_all_nodes:: Seen Nodes: ", self.seen_nodes)
        nsteps = 0
        last_node = None
        cur_node = from_node
        cost_to_cur_node = 0
        # --  loop
        if debug_print:
            print ("ENV_GRAPH: _get_cost_to_all_nodes:: While Loop -- ")
        while True:
            if debug_print:
                print ("\tBack to start of loop...")
            # -- get the nodes nearby
            # if the node is discovered node, and not visited, do not give details of its connections
            if cur_node not in self.explored_nodes:
                if debug_print:
                    print ("\tNode: {} is not in explored nodes".format(cur_node))
                    print ("\tExplored Nodes: {}".format(self.explored_nodes))
                nodes_nearby = []
            else:
                nodes_nearby = self.graph_explored[cur_node]
            
            # BEGIN debug print
            if debug_print:
                print("\tcur_node: ", cur_node)
                print("\tnodes_nearby: ", nodes_nearby)
            # END debug print
            
            # -- loop through nearby nodes
            for node in nodes_nearby:
                # BEGIN debug print
                if debug_print:
                    print ("\t\tNode: {}\t Node Idx: {}\tCost To reach: {}".format(node, node[0], node[1]))
                # END debug print
                
                # -- get index and cost to node
                nodeIdx = node[0]
                node_cost = node[1]
                
                # check if cost to node (distance to current node + node cost)
                # is less than minimum recorded distance
                distance_to_nodeIdx = min_distances_to_v[cur_node] + node_cost
                
                # BEGIN debug print
                if debug_print:
                    print ("\t\t Cost to Reach from node <{}> :: {}".format(from_node, distance_to_nodeIdx))
                # END debug print
                
                # -- if cost is lower than current minimum cost, update
                if distance_to_nodeIdx < min_distances_to_v[nodeIdx]:
                    min_distances_to_v[nodeIdx] = distance_to_nodeIdx
                    source_of_v[nodeIdx] = cur_node
                    # -- add to queue
                    heapq.heappush(lowest_distance_priority_queue, (distance_to_nodeIdx, nodeIdx))
                    #lowest_distance_priority_queue.put((distance_to_nodeIdx, nodeIdx ))
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
                    print ("\tNode {}, not in Visited Nodes".format(cur_node))
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
            print ("ENV_GRAPH: _get_cost_to_all_nodes:: All remote nodes visited!")
            print ("ENV_GRAPH: _get_cost_to_all_nodes:: Minimum distances are: ", dict(min_distances_to_v))
            print ("ENV_GRAPH: _get_cost_to_all_nodes:: Sources of vertices are: ", dict(source_of_v))
            print ("ENV_GRAPH: _get_cost_to_all_nodes:: Return back to control")
            print ("="*100)
        # END debug print
        return dict(min_distances_to_v), dict(source_of_v)
        
    
    def _get_local_nodes_explored(self, node):
        if node not in gt_explored:
            return None
        else:
            return deepcopy(self.graph_explored[node])
        
    
    def set_up_initial_robot_position(self, position, debug_print = True):
        if debug_print:
            print ("Function: set_up_initial_robot_position :: Position Selected = {}".format(position))
        
        if position is not None and position not in self.graph_gt:
            print ("Function: set_up_initial_robot_position :: Position NOT found in dictionary, selecting random node")
            position = None
        
        if position is None:
            position = random.choice(list(self.graph_gt.keys()))
            if debug_print:
                print ("Function: set_up_initial_robot_position :: Position Selected = {}".format(position))
        
        # -- initialize explored nodes
        self.explored_nodes = []
        
        # -- initialize seen nodes
        self.seen_nodes = [position]
        
        # -- add node to explored graph (and update explored and seen nodes)
        self._add_node_graph_explored(position)
        
        # -- return the explored graph_explored, explored_nodes, and seen nodes
        return self.graph_explored, self.explored_nodes, self.seen_nodes
        
    def update_graph_explored(self, node):
        self._add_node_graph_explored(node)
    
    # --------------------------------------------------------------------------
    # Function: Takes a input graph and prints it
    # --------------------------------------------------------------------------
    # Outputs the graph on screen
    # --------------------------------------------------------------------------
    # Input: Graph <dict>
    # --------------------------------------------------------------------------
    # Output: Screen output showing the print of the graph
    # --------------------------------------------------------------------------
    def _print_graph(self, graph, graph_name=""):
        # The input should be in form of a dictionary
        print ("--"*50, "\t {} Graph: ".format(graph_name))
        for index,  connections in graph.items():
            print ("\tNode: ", index)
            print ("\tConnections: ", connections)
            print ("-"*30)
        print ("##"*50)
        return 

    def _add_node_graph_explored(self, node, debug_print=True):
        """ Updates self.graph_explored
        """
        if node not in self.graph_gt:
            print ("ERROR!")
            print ("Function: _add_node_graph_explored :: Node {} does not exist in graph_gt".format(node))
            self._print_graph(self.graph_gt, "Ground Truth")
            raise ValueError
        # -- update the graph
        if debug_print:
            print ("ENV GRAPH: _add_node_graph_explored :: ", node)
            print ("\tself.graph_explored: {}".format(dict(self.graph_explored)))
        if node in self.explored_nodes:
            return
        else:
            self.graph_explored[node] = deepcopy(self.graph_gt[node])
        
        # -- update the explored nodes
        self.explored_nodes.append(node)
        if debug_print:
            print ("add_node_graph_explored:: Graph_updated_see_graph_here:")
            print (dict(self.graph_explored))
        
        # -- create the node from the seen nodes to the visited node
        if debug_print:
            print ("ENV GRAPH:: add nodes for the local nodes")
        for local_node_idx, local_node_cost in self.graph_explored[node]:
            if debug_print:
                print ("\t Local Node Index: ", local_node_idx)
                print ("\t Local node Cost: ", local_node_cost)
            if local_node_idx in self.graph_explored:
                if debug_print:
                    print ("\t\tLocal Node : {} found in graph_explored".format(local_node_idx))
                # -- get the connection list from graph explored for local_node_idx
                local_node_connections = self.graph_explored[local_node_idx]
                if debug_print:
                    print ("\t\tGraph_explored[{}] = {}".format(local_node_idx,local_node_connections))
                connection_required = (node, local_node_cost)
                if debug_print:
                    print ("\t\tconnection required: {}".format(connection_required))
                if connection_required in local_node_connections:
                    continue
                else:
                    if debug_print:
                        print ("\t\tAdding connection...")
                    self.graph_explored[local_node_idx].append(connection_required)
            else:
                # -- add the connection to graph_explored as a new node, which points to the current node
                if debug_print:
                    print ("\t\tLocal Node : {} NOT found in graph_explored".format(local_node_idx))
                self.graph_explored[local_node_idx] = [(node, local_node_cost)]
        
        # -- update seen nodes
        current_seen_nodes = self._get_neighboring_nodes(node)
        
        # -- update the seen nodes
        self.seen_nodes.extend(self._get_neighboring_nodes(node))
        
        # -- make seen nodes unique
        self.seen_nodes = list(set(self.seen_nodes))
        
        if debug_print:
            print ("add_node_graph_explored:: Graph_updated_see_graph_here:")
            print (dict(self.graph_explored))
            
        return
    
    # --------------------------------------------------------------------------
    # Function: _get_neighboring_nodes
    # --------------------------------------------------------------------------
    # 1. For a input node, returns the neighboring nodes for the given node.
    # 2. Added the nodes to list 'seen_nodes' and returns the list
    # --------------------------------------------------------------------------
    def _get_neighboring_nodes(self, node, debug_print = True):
        # -- initialize nodes
        neighboring_nodes = deepcopy(self.graph_gt[node])
        seen_nodes = []
        
        # -- debug print
        if debug_print:
            print ("Function: _get_neighboring_nodes :: Neighboring nodes in GT: {}".format(neighboring_nodes))
            
        # -- fill up the seen nodes with the neighboring node indices
        for node in neighboring_nodes:
            seen_nodes.append(node[0])
        return seen_nodes
        
    
    # --------------------------------------------------------------------------
    # Function: _initialize_graph_explored
    # --------------------------------------------------------------------------
    # Initialize graph explored by initializing the explored graph vertices to
    # zero
    # --------------------------------------------------------------------------
    def _initialize_graph_explored(self):
        #self.graph_explored = defaultdict(lambda: [])
        self.graph_explored = dict()
        return True
    
    
    # --------------------------------------------------------------------------
    # Function: _initialize_graph
    # --------------------------------------------------------------------------
    # Initialize a graph from a given filename
    # --------------------------------------------------------------------------
    def _initialize_graph(self, filename, debug_print=True):
        count = 0
        self.edgeList=[]
        self.n_edges = 0
        with open(filename) as fp:
            while True:
                count += 1
                line = fp.readline()
                line = line.strip()
                if not line:
                    break
                if count == 1:
                    # the first line has the number of nodes
                    elems = line.split()
                    self.read_n_nodes = int(elems[0])
                    self.n_rows = int(elems[1])
                    self.n_cols = int(elems[2])
                else:
                    nodes = line.split()
                    v1 = int(nodes[0]) - 1 # minus 1 because input file has indices from 1
                    v2 = int(nodes[1]) - 1 # minus 1 because input file has indices from 1
                    weight = float(nodes[2])
                    self.edgeList.append((v1,v2, weight))
                    self.n_edges += 1

        if debug_print:
            for i in range(0,self.n_edges):
                print ("Idx: {}\tEdge: {}".format(i,self.edgeList[i]))
            
        # -- set up default dict for the edges
        graph_gt = defaultdict(lambda: [])
        
        # -- setup adjacency dictionary
        for edge in self.edgeList:
            v1 = edge[0]
            v2 = edge[1]
            w = edge[2]
            graph_gt[v1].append((v2, w))
            graph_gt[v2].append((v1, w))
        
        # -- readjust the number of nodes if there are nodes with no connectivity
        self.n_nodes = len(graph_gt)
        self.graph_gt = graph_gt
        
        # No adjList has been defined, the graph_gt is the whole information 
        # of the graph
        # -- print the choices
        print ("n_rows: {}\tn_cols: ".format(self.n_rows,self.n_cols))
        return
    
    
    def _make_pruned_graph(self, debug_print = False):
        """
        consider the nodes which have the least degree, and then start
        deleting them from the graph. Consider the data structure to be a 
        default dict
        """
        # -- make a function for deep copy
        prunedGraph = self._deep_copy_graph(self.graph_gt)
        
        # -- print the prunedGraph
        if debug_print:
            print ("Function: _make_pruned_graph :: Initialized Pruned Graph (Should contain all the nodes and edges)")
            self._print_graph(prunedGraph, "Pruned")
            print ("-"*50)
        
        # -- prune graph
        """
        deleting the nodes during the loop throws an error about the the 
        number of nodes changed within a loop.
        """
        delete_nodes = [] #to save the node indices to be deleted
        while(True):
            # -- initialize number of nodes deleted
            n_deleted = 0
            current_delete_nodes = [] # to store the nodes to be deleted in one iteration
            
            # -- run through the list of nodes in 
            for nodeId, connections in prunedGraph.items():
                # connections are 4 for each node, but the connections leading
                # to the same node means that the connectivity is less than 4
                node_degree = len(connections)
                if debug_print:
                    print ("Function: _make_pruned_graph :: Node: {} \t Connections: {}".format(nodeId, connections))
                    print ("Function: _make_pruned_graph :: Node: {} \t Degree: {}".format(nodeId, node_degree))
                    print ("- " * 30)
                
                # -- go through each of the connections from the node, to ensure that the
                # current node has no references to previous deleted nodes
                delete_connections = []
                for connection_idx, connection_node in enumerate(connections):
                    if connection_node[0] in delete_nodes:
                        delete_connections.append(connection_idx)
                
                # -- delete the nodes in delete connections
                for connection_idx in sorted(delete_connections, reverse=True):
                    del(connections[connection_idx])
                    
                # -- update node degree
                node_degree = len(connections)
                
                
                if debug_print:
                    print ("Function: _make_pruned_graph :: Node: {} \t Connections: {}".format(nodeId, connections))
                    print ("Function: _make_pruned_graph :: Node: {} \t Degree: {}".format(nodeId, node_degree))
                    print ("==" * 30)
                    
                if node_degree <= 1:
                    delete_nodes.append(nodeId)
                    current_delete_nodes.append(nodeId)
                    n_deleted += 1
            
            # -- print the nodes to be deleted
            if debug_print:
                print ("Function: _make_pruned_graph :: Nodes with degree 1: {}".format(current_delete_nodes))
                print ("Function: _make_pruned_graph :: Current Delete Nodes: {}".format(current_delete_nodes))
                print ("Function: _make_pruned_graph :: All Delete Nodes: {}".format(current_delete_nodes))
                print ("- "*30)
            # -- delete the nodes marked in current_delete_nodes
            for nodeIdx in current_delete_nodes:
                if nodeIdx in prunedGraph:
                    del(prunedGraph[nodeIdx])
                if debug_print:
                    print ("Function: _make_pruned_graph :: Delete Node : {}".format(nodeIdx))
                    print ("-" * 30)
            
            # -- print the remaining nodes
            if debug_print:
                print ("Function: _make_pruned_graph :: Pruned Graph after one loop:")
                print ("Function: _make_pruned_graph :: Total pruned nodes: ", n_deleted)
                print ("Function: _make_pruned_graph :: Current Graph: ")
                self._print_graph(prunedGraph, "Pruned")
                print ("-"*100)
            
            # -- exit condition
            if n_deleted == 0:
                break
        if debug_print:
            print ("Function: _make_pruned_graph :: Graph pruned")
            for nodeIdx, connections in prunedGraph.items():
                print ("Vertex: ", nodeIdx, "Connections: ", connections)
        
        # -- return the pruned graph
        return prunedGraph
        
    # --------------------------------------------------------------------------
    # Function: _deep_copy_graph
    # --------------------------------------------------------------------------
    # Copies a graph, ensures new data allocation for this graph
    # --------------------------------------------------------------------------
    # Input:
    #   source: Input Graph
    # Output
    #   sink: New Graph as a copy of the input graph 'source'
    # --------------------------------------------------------------------------
    def _deep_copy_graph(self, source):
        """
        The graph would be copied from source to sink
        """
        # -- identify data structure of source (dict/defaultdict)
        if isinstance(source, dict):
            sink = {}
        elif isinstance(source, defaultdict):
            sink = defaultdict(lambda: [])
        
        # -- copy the lists
        for key, items in source.items():
            sink[key] = items.copy()
        
        return sink
        
    def _read_node_tags(self, nodeTagFileName):
        count = 0
        self.node_tags = {}
        with open(nodeTagFileName) as fp:
            while True:
                count += 1
                line = fp.readline()
                line = line.strip()
                if not line:
                    break
                # -- split line
                elems = line.split()
                node_idx = int(elems[0])
                node_type = elems[1]
                # -- add to nodeTags
                self.node_tags[node_idx] = node_type
        return


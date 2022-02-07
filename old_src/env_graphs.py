from collections import defaultdict
from queue import PriorityQueue
import heapq
import numpy
from copy import deepcopy
import sys
import numpy as np



"""
Data elements:
==============
graph_explored:  <defaultdict> of the nodes explored. Each node has 
                 connectivity information along with it.
discovered_vertices: <list> list of discovered vertices
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
    including all visited or discovered vertices

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
                 nodeTagFileName='testVertexTagFile.txt'):
        """ Initialize the ground truth graph and the explored graph
        """
        # -- set up filenames
        self.nodeTagFileName = nodeTagFileName
        self.filename = edgeFileName
        
        # -- read the node tags
        self._read_node_tags(self.nodeTagFileName)
        
        # -- read the graph to ground truth graph
        self._initialize_graph(edgeFileName)
        
        # -- initialize the explored graph
        self._initialize_graph_explored()
        
        # -- return
        return


    def _get_local_nodes_gt(self, node):
        """ Returns neighboring vertices of a vertex from the ground truth graph
        ------------------------------------------------------------------------
        Input:
        node: A node of the graph environment
        ------------------------------------------------------------------------
        Return:
        a list of local nodes
        """
        if node not in self.graph_gt:
            print ("Function: get_local_nodes :: Error: Node <{}> not in graph".format(node))
            raise ValueError
        # -- get local nodes from graph_gt
        local_nodes = deepcopy(self.graph_gt[node])
        print ("Local Nodes: ", local_nodes)
        return local_nodes
    

    def get_local_unvisited_nodes_gt(self, node, debug_print = False):
        """ Returns a set of unvisited nodes from the local neighborhood
        ------------------------------------------------------------------------
        Input:
        node: A node of the graph environment
        debug_print: Prints on screen if true
        ------------------------------------------------------------------------
        Returns a list of local unvisited nodes
        ------------------------------------------------------------------------
        TODO: What do you mean by unvisited? 
        """
        # -- get all the neighboring vertices
        local_nodes = self._get_local_nodes_gt(node)
        
        # -- check the vertices that have been visited
        # if visited add them to a list (del_idx)
        del_idx = []
        for local_nodes_idx, node in enumerate(local_nodes):
            if node[0] in self.visited_vertices:
                del_idx.append(local_nodes_idx)
        
        if debug_print:
            print ("ENV GRAPH: Function: get_local_unvisited_nodes_gt :: local_nodes: {}".format(local_nodes))
            print ("ENV GRAPH: Function: get_local_unvisited_nodes_gt :: del_idx: {}".format(del_idx))
        
        # -- remove all the vertices in del_idx from the local_nodes
        for idx in sorted(del_idx, reverse=True):
            del(local_nodes[idx])
        
        # print the unvisited local nodes
        if debug_print:
            print ("ENV_GRAPH: GET_LOCAL_UNVISITED_NODES :: Unvisited Local Nodes ::: {}".format(local_nodes))
        return local_nodes
    

    def get_discovered_nodes(self, node, debug_print = False):
        """ Get all nodes 
        """
        pass

    def get_remote_nodes(self, node, debug_print = True):
        """ Get remote nodes (not neighborhood nodes) for a given node.
        Input:
            node: A node in the graph environment
            debug_print: Prints on screen if True
        Output:
            distance_all_nodes: 
            source_all_nodes: 
        """
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
        distance_all_nodes, source_all_nodes = self._get_cost_to_all_nodes_dijkstra(node, debug_print)
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
            print ("ENV GRAPHS: get_remote_nodes:: visited vertices: {}".format(self.visited_vertices))
            print ("\tLoop over the distance_all_nodes...")
            input("Continue?")
        # END debug print
        
        # -- filter the nodes which are explored but not visited
        del_node = []
        for node, cost in distance_all_nodes.items():
            # BEGIN debug_print
            if debug_print:
                print ("\t\tNode: {}\tCost:{}".format(node, cost))
            # END debug_print
            if node in self.visited_vertices:
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
    

    def get_all_nodes_with_path_cost(self, node, debug_print = True):
        """ Get all nodes in the explored graph with the path cost from input node
        Output:
            distance_all_nodes: 
        """
        if debug_print:
            print ("Function: get_all_remote_nodes:: Begin here")
            print ("Function: get_all_remote_nodes:: Node: {}".format(node))
        distance_all_nodes, source_all_nodes = self._get_cost_to_all_nodes_dijkstra(node, debug_print)
        return distance_all_nodes, source_all_nodes
    
    
    def get_all_unvisited_nodes_with_path_cost(self, node, debug_print = True):
        """ Get unvisited nodes path cost from input node.
        Input:
            node: <int> node_id of a node in the explored graph
            debug_print: <bool> boolean to print the values of computation
        Output:

        """
        if debug_print:
            print ("ENV GRAPHS: get_all_unvisited_nodes_with_path_code:: Start.")
        distance_all_nodes, source_all_nodes = self._get_cost_to_all_nodes_dijkstra(node, debug_print)
        
        # -- filter the nodes which are seen but not visited
        del_node = []
        for node, cost in distance_all_nodes.items():
            if debug_print:
                print ("\t\tNode: {}\tCost:{}".format(node, cost))
            if node in self.visited_vertices:
                del_node.append(node)
        
        if debug_print:
            print ("ENV GRAPHS: get_all_unvisited_nodes_with_path_cost:: nodes to be deleted: {}".format(del_node))
        
        # -- get unique elements by converting list to set and back to list
        del_node = list(set(del_node))
        # as distance_all_nodes is dict, we can delete nodes without sorting 
        # indices in reverse order
        for node in del_node:
            del(distance_all_nodes[node])
            del(source_all_nodes[node])
        
        return distance_all_nodes, source_all_nodes
    
    def go_local_node(self, node_idx, debug_print = True):
        """ Update the graph explored if a local node is visited
        TODO: merge this with go_remote_node
        -----------------------------------------------------------------------
        Input:
            node_idx: <int> node_idx of the new visited node
            debug_print: <bool> if True, display the debugging print statements
        -----------------------------------------------------------------------
        Output:
            node_idx: the same node idx that was input TODO: Why?
            distance_all_nodes: distance to all nodes
        """
        if debug_print:
            print ("ENV GRAPHS:  GO LOCAL NODE: visited vertices before update: {}".format(self.visited_vertices))
            print ("ENV GRAPH: GO LOCAL NODE: update explored graph with node: {}".format(node_idx))
        self.update_graph_explored(node_idx)
        if debug_print:
            print ("ENV GRAPHS:  GO LOCAL NODE: visited vertices before update: {}".format(self.visited_vertices))
            print ("ENV GRAPHS: GO LOCAL NODE: distance_all_nodes: {}".format(self.distance_all_nodes))
        # return the current node and the cost incurred to reach there
        return node_idx, self.distance_all_nodes[node_idx]
    

    def go_remote_node(self, current_node, node_idx, step_flag=True, debug_print=True):
        """ Update the explored graph when the robot visits a remote node.
        ------------------------------------------------------------------------
        Input:
            current_node: <int> current position of robot
            node_idx: <int> index of remote node
            step_flag: <bool> if true, then the robot only moves a step towards
                       the target vertex (node_idx) from the current_node. If 
                       false, then the robot directly moves to the target node.
            debug_print: <bool> to print the debug statements
        ------------------------------------------------------------------------
        Output:
            node_idx: the same node_idx as input
            distance_all_nodes: dijkstra's algorithm distance to all nodes.
        ------------------------------------------------------------------------
        Description:

        """
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
        

    def _get_cost_to_all_nodes_dijkstra(self, from_node, debug_print = False):
        """ Calculates cost to all vertices in the explored graph. Uses dijkstra's
        algorithm to get the distance to all the vertices.
        ------------------------------------------------------------------------
        Input:
        from_node: Distance from the source node to where all the node distances
                   are calculated
        debug_print: print the nodes to debug
        ------------------------------------------------------------------------
        Step 1: 
            Initialize two default dictionaries. Minimum Distance and Source.
            min_distances_to_v: stores the current minimum distance from the 
                                source vertex to any vertex in the graph
            source_of_v: stores the previous vertex to reach that particular 
                         vertex
        Step 2:
            Mark visited nodes
        Step 3:
        ------------------------------------------------------------------------
            
        """
        if debug_print:
            print ("ENV GRAPH: _get_cost_to_all_nodes_dijkstra :: from_node: {}".format(from_node))
        
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
            print ("ENV_GRAPH: _get_cost_to_all_nodes_dijkstra:: Visited Nodes: ", visited_nodes)
            print ("ENV_GRAPH: _get_cost_to_all_nodes_dijkstra:: Seen Nodes: ", self.discovered_vertices)
        nsteps = 0
        last_node = None
        cur_node = from_node
        cost_to_cur_node = 0
        # --  loop
        if debug_print:
            print ("ENV_GRAPH: _get_cost_to_all_nodes_dijkstra:: While Loop -- ")
        while True:
            if debug_print:
                print ("\tBack to start of loop...")
            # -- get the nodes nearby
            # if the node is discovered node, and not visited, do not give details of its connections
            if cur_node not in self.visited_vertices:
                if debug_print:
                    print ("\tNode: {} is not in visited vertices".format(cur_node))
                    print ("\tExplored Nodes: {}".format(self.visited_vertices))
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
            print ("ENV_GRAPH: _get_cost_to_all_nodes_dijkstra:: All remote nodes visited!")
            print ("ENV_GRAPH: _get_cost_to_all_nodes_dijkstra:: Minimum distances are: ", dict(min_distances_to_v))
            print ("ENV_GRAPH: _get_cost_to_all_nodes_dijkstra:: Sources of vertices are: ", dict(source_of_v))
            print ("ENV_GRAPH: _get_cost_to_all_nodes_dijkstra:: Return back to control")
            print ("="*100)
        # END debug print
        return dict(min_distances_to_v), dict(source_of_v)
        
    
    def _get_local_nodes_explored(self, node):
        if node not in gt_explored:
            return None
        else:
            return deepcopy(self.graph_explored[node])
        
    
    def set_up_initial_robot_position(self, position, debug_print = False):
        if debug_print:
            print ("Function: set_up_initial_robot_position :: Position Selected = {}".format(position))
        
        if position is not None and position not in self.graph_gt:
            print ("Function: set_up_initial_robot_position :: Position NOT found in dictionary, selecting random node")
            position = None
        
        if position is None:
            position = random.choice(list(self.graph_gt.keys()))
            if debug_print:
                print ("Function: set_up_initial_robot_position :: Position Selected = {}".format(position))
        
        # -- initialize visited vertices
        self.visited_vertices = []
        
        # -- initialize discovered vertices
        self.discovered_vertices = [position]
        
        # -- add node to explored graph (and update explored and discovered vertices)
        self._add_node_graph_explored(position)
        
        # -- return the explored graph_explored, explored_nodes, and discovered vertices
        return self.graph_explored, self.visited_vertices, self.discovered_vertices
        
    def update_graph_explored(self, node):
        """ Updates the graph explored with the new vertex.
        """
        self._add_node_graph_explored(node)
    
    def _print_graph(self, graph, graph_name=""):
        """ Prints the graph as std output.
        Input: graph <dict> key: vertex index, <val>: neighboring vertices
        Output: Screen
        Return: None
        """
        # The input should be in form of a dictionary
        print ("--"*50, "\t {} Graph: ".format(graph_name))
        for index,  connections in graph.items():
            print ("\tNode: ", index)
            print ("\tConnections: ", connections)
            print ("-"*30)
        print ("##"*50)
        return 

    def _add_node_graph_explored(self, node, debug_print=False):
        if node not in self.graph_gt:
            print ("Function: _add_node_graph_explored :: ERROR! Node {} does not exist in graph_gt".format(node))
            self._print_graph(self.graph_gt, "Ground Truth")
        # -- update the graph
        if debug_print:
            print ("ENV GRAPH: _add_node_graph_explored :: ", node)
            print ("\tself.graph_explored: {}".format(dict(self.graph_explored)))
        if node in self.visited_vertices:
            return
        else:
            self.graph_explored[node] = deepcopy(self.graph_gt[node])
        
        # -- update the visited vertices
        self.visited_vertices.append(node)
        if debug_print:
            print ("add_node_graph_explored:: Graph_updated_see_graph_here:")
            print (dict(self.graph_explored))
        
        # -- create the node from the discovered vertices to the visited node
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
        
        # -- update discovered vertices
        current_discovered_vertices = self._get_neighboring_nodes(node)
        
        # -- update the discovered vertices
        self.discovered_vertices.extend(self._get_neighboring_nodes(node))
        
        # -- make discovered vertices unique
        self.discovered_vertices = list(set(self.discovered_vertices))
        
        if debug_print:
            print ("add_node_graph_explored:: Graph_updated_see_graph_here:")
            print (dict(self.graph_explored))
        return
    

    def _get_neighboring_nodes(self, node, debug_print = False):
        """ Get neighboring vertices for a given vertex.
        Input: node <int> (should be a key in the ground truth graph)
        Output: discovered_vertices <list<int>> vertices in thee graph
        """
        # -- initialize nodes
        neighboring_nodes = deepcopy(self.graph_gt[node])
        discovered_vertices = []
        
        # -- debug print
        if debug_print:
            print ("Function: _get_neighboring_nodes :: Neighboring nodes in GT: {}".format(neighboring_nodes))
            
        # -- fill up the discovered vertices with the neighboring node indices
        for node in neighboring_nodes:
            discovered_vertices.append(node[0])
        return discovered_vertices
        
    
    def _initialize_graph_explored(self):
        """ Initialize a graph explored by initializing the explored graph to empty
        dictionary.
        """
        self.graph_explored = dict()
        return
    
    
    def _initialize_graph(self, filename, debug_print=False):
        """ Initialize a graph from a filename
        """
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
        
        # No adjacency list has been defined, the graph_gt is the whole information 
        # of the graph
        return
    
        
    def _deep_copy_graph(self, source):
        """ Deep copy a graph with new memory allocation.
        Input: source: input graph
        Output: sink: output graph
        Note: The graph would be copied from source to sink
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

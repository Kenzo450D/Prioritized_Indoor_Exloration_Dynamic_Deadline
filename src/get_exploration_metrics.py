from copy import deepcopy
from collections import defaultdict

"""

Values Required:
    node_type_percent
    keys: 'corridors', 'small_room', 'large_room', 'rooms', 'total'
"""

class ExplorationMetrics:
    def __init__(self, nodeTagFileName='testVertexTagFile.txt', visited_nodes= []):
        # -- read the node tags to a dictionary
        self.nodeTagFileName = nodeTagFileName
        self._read_node_tags(self.nodeTagFileName)
        self._calc_percent_explored(visited_nodes)
        # self._print_percentages()
        
    def _calc_percent_explored(self, nodeList):
        # -- initialize count
        node_type_count =  defaultdict(lambda: 0)
        for node in nodeList:
            node_type_count[self.node_tags[node]] += 1
        
        node_type_percent = defaultdict(lambda: 0)
        for key, val in node_type_count.items():
            node_type_percent[key] = float(val/self.tag_count[key])
            
        node_type_percent['total'] = float(len(nodeList)/self.total_nodes)
        
        node_type_percent['rooms'] = float((node_type_count['small_room'] + node_type_count['large_room'])/ (self.tag_count['small_room'] + self.tag_count['large_room']))
        
        self.node_type_percent = node_type_percent
        self.node_type_count = node_type_count
    
    def _print_percentages(self):
        # -- print the node type
        for key, val in self.node_type_percent.items():
            print ("Type: {}\tPercent: {}".format(key, val))
        
        print ("-" * 100)
        
        # -- print the node count
        for key, val in self.node_type_count.items():
            print ("Type: {}\Count: {}".format(key, val))
        
        print ("-" * 100)
        print ("Ground Truth: ")
        for key, val in self.tag_count.items():
            print ("Type: {}\Count: {}".format(key, val))
        print ("-" * 100)
        
        
        
    def _read_node_tags(self, nodeTagFileName):
        count = 0
        self.node_tags = {}
        self.tag_count =  defaultdict(lambda: 0)
        self.total_nodes = 0
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
                self.tag_count[node_type] += 1
                self.total_nodes += 1
        return

    

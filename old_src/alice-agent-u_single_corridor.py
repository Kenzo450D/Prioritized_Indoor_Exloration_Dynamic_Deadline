#!/usr/bin/env python3
import sys
from copy import deepcopy
from priority_exploration_agent import PriorityExplorationAgent
from get_files_for_environment_agent import get_env_params
from get_files_for_environment_agent import get_agent_files

# -- set up parameters
environment_name = 'looped_corridor'
agent_name = 'Priority_Greedy'
env_params = get_env_params(environment_name)
agent_priority_filename = get_agent_files(agent_name)


t_a = 20
t_r0 = 10
outfile = 'tmp.txt'


# -- parameters for plotting
bg_img = 'environments/single_corridor/rooms.png'
vertex_map = 'environments/single_corridor/sc-env-mapping.txt'

pe_agent = PriorityExplorationAgent(t_a, t_r0, agent_priority_filename, env_params['edges'], env_params['vertex_tag'], env_params['init_pose'], outfile)
pe_agent.explore()

# -- print the parameters
print ("{} {} {}".format(t_a, t_r0, env_params['init_pose']))
print ("Visited Nodes: ", pe_agent.visited_nodes)
print ("All time limits:", pe_agent.all_time_limits)
visited_nodes = deepcopy(pe_agent.visited_nodes)
print ("Visited Nodes: ", visited_nodes)

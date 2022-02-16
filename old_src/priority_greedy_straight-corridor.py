#!/usr/bin/env python3
import sys
from copy import deepcopy
from priority_exploration_agent_greedy import PriorityExplorationAgent
from get_files_for_environment_agent import get_env_params
from get_files_for_environment_agent import get_agent_files

# -- set up parameters
environment_name = 'straight_corridor'
agent_name = 'Priority_Greedy'
env_params = get_env_params(environment_name)
agent_priority_filename = get_agent_files(agent_name)


t_a = 20
t_r0 = 30
outfile = 'tmp.txt'


pe_agent = PriorityExplorationAgent(t_a, t_r0, agent_priority_filename, env_params['edges'], env_params['vertex_tag'], env_params['init_pose'], outfile)
pe_agent.explore()

# -- print the parameters
print ("t_a: {}\tt_r0: {}\tinit_position: {}".format(t_a, t_r0, env_params['init_pose']))
print ("Visited Nodes: ", pe_agent.visited_nodes)
print ("All time limits:", pe_agent.all_time_limits)

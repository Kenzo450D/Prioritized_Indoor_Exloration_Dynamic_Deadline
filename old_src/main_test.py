# Author: Sayantan Datta
# Email: sayantan <dot> knz <at> gmail <dot> com
# Date: October 27, 2021
# -- Global imports
import numpy as np
import sys
import heapq
from datetime import datetime
from copy import deepcopy
# -- Local imports
from exploration_env import GraphExplorationEnvironment
from get_files_for_environment_agent import get_env_params
from get_files_for_environment_agent import get_agent_files
from prioritized_greedy_exploration_algorithm import PriorityExplorationAlgorithm

"""
# class PriorityExploration:
#     def __init__(self, env_name, agent_name, t_a, t_r0, n_robots=1):
#         # -- get the environment files
#         self.env_params = get_env_params(env_name)
#         agent_priority_filename = get_agent_files(agent_name)
#         # -- print the environment files
#         print(f"Environment name: {env_name}")
#         for key, filename in self.env_params.items():
#             print(f"{key}: {filename}")
#         print("-" * 50)

#         # -- set up initial robot positions
#         if n_robots == 1:
#             init_pos = self.env_params['init_pose']
#         if n_robots < 1:
#             print("n_robots should be integer and at least 1")
#             raise ValueError
#         else:
#             print("Multiple robots")
#             init_pose = [0 for x in range(0, n_robots)]


#         # -- load the environment
#         self.env = GraphExplorationEnvironment(self.env_params['edges'],
#                                         self.env_params['vertex_tag'],
#                                         agent_priority_filename,
#                                         t_a, t_r0, init_pose)
        
#         # -- initialize visited nodes
#         self.visited_nodes = []
#         self.all_time_limits = []

#         # -- initialize completed exploration
#         self.flag_completed = True
#         self.flag_reached_home = True
#         self.flag_go_straight_home = True
        
#         # -- initialize file output to check results
#         self.exploration_step = 0 # To print steps in exploration
#         current_time = datetime.now()
#         self.exploration_file = current_time.strftime("output_%d_%m_%Y_%H_%M_%S.txt")
        
#         # -- set current state
#         self.current_position = self.env_params['init_pose']
        
#         return
    
#     def explore(self):
#         while True:
#             # -- get two types of output:
#             #    one for the greedy algorithm, one for the MILP solver.
#             # Part 1: Greedy algorithm
#             observation = self.env.get_state()
#             explored_graph = self.get_explored_graph()
#             # -- update visited nodes
#             self.visited_nodes.append(observation[0])

#             #TODO: Remove print statement from final code.
#             # -- print the visited nodes
#             print("Visited nodes: ", self.visited_nodes)
            
#             # --  save the details in the file
#             with open(self.exploration_file, 'a') as file_object:
#                 file_object.write("-"*50+"\n")
#                 file_object.write(f"Exploration_step: {self.exploration_step}\n")
                
#             # -- check for time remaining
#             time_remaining = observation[3]
#             if time_remaining is not None and time_remaining <= 0:
#                 break
            
#             # -- check if exploration is complete
#             if len(observation[1]) == 0 and len(observation[2]) == 0:
#                 # we do not have places to move
#                 print("AGENT: Region explored completely as local and remote free indices are empty")
#                 # -- if robot is not in starting node, go to starting node
#                 if observation[0] == self.init_position:
#                     break
#                 else:
#                     print("AGENT: Agent not in starting position, go back to starting position")
#                     observation = self.env.step(self.init_position)
#                     # input("Check")
#             else:
#                 # -- if time remaining is unknown
#                 if time_remaining is None:
#                     # to replace solver when time is known, change code here
#                     action = self._choose_greedy_action(observation)
#                     action_cost = action[0]
#                     action_node = action[1]
#                     with open(self.exploration_file, 'a') as file_object:
#                         file_object.write(f"\tTime unknown: Using greedy solver")
#                         file_object.write(f"\tCurrent Vertex       : {observation[0]}\n")
#                         file_object.write(f"\tTarget Vertex        : {action_node}\n")
#                 elif time_remaining <= 0:
#                     break
#                 else:
#                     # -- if time limit is known
#                     # To replace solver when time is unknown, change code here
#                     action = self._choose_greedy_action_time(observation)
#                     action_node = action[1]
#                     with open(self.exploration_file,'a') as file_object:
#                         file_object.write("\tPreviously calculated target not reached\n")
#                         file_object.write(f"\tCurrent Vertex       : {observation[0]}\n")
#                         file_object.write(f"\tTarget Vertex        : {action_node}\n")
#                         file_object.write(f"\tTime Remaining       : {time_remaining}\n")
#                 # -- take the action
#                 print("Action Node: ", action_node)
#                 print("Current node before step: ", self.env.current_robot_position)
#                 observation = self.env.step(action_node)
#                 # update exploration step for file print
#                 self.exploration_step += 1
            
#             #TODO: Remove print when code is final
#             # -- print the explored graph
#             explored_graph = self.get_explored_graph()
"""    

    
def main(env_name, agent_name, t_a, t_r0):
    # -- get the environment files
    env_params = get_env_params(env_name)
    agent_priority_filename = get_agent_files(agent_name)

    # -- print the environment files
    print(f"Environment name: {env_name}")
    for key, filename in env_params.items():
        print(f"{key}: {filename}")
    print("-" * 50)


    # -- load the environment
    env = GraphExplorationEnvironment(env_params['edges'],
                                      env_params['vertex_tag'],
                                      agent_priority_filename,
                                      t_a, t_r0, env_params['init_pose'])
    
    print("Initialized exploration environment")
    print("Environment Visited Nodes:", env.env.visited_vertices)
    print("Environment Discovered Vertices: ", env.env.discovered_vertices)
    print("Explored graph: ", env.env.graph_explored)
    print("-"*50)

    # -- set up priority exploration algorithm
    pea = PriorityExplorationAlgorithm(agent_priority_filename, output_filename='out.txt', init_position=env_params['init_pose'], env=env.env)


    # -- while loop
    while (True):
        # In each iteration, the output
        # -- initialize visited vertices
        # -- initialize flags
        # -- initialize file output to check results
        # -- initialize flags for solver
        
        # -- get the target position to visit
        current_deadline = env.get_time_limit()
        current_robot_position = env.current_robot_position
        visited_vertices = env.env.visited_vertices
        graph_explored = env.env.graph_explored
        
        print ("Main Test: ","--"*20)
        print ("Main Test: Current Deadline: ", current_deadline)
        print ("Main Test: Current Robot Position: ", current_robot_position)
        print ("Main Test: Visited vertices: ", visited_vertices)
        print ("Main Test: Graph explored: ", graph_explored)
        print ("Main Test: ","--"*20)
        print ("Main Test: Find target vertex...")
        target = pea.getTargetVertex(env.get_time_limit(), env.current_robot_position, env.env.visited_vertices, env.env.graph_explored)
        print ("Main Test: Target vertex selected: ", target)
        
        # -- get the new state
        state = env.step(target)
        print ("State: ", state)
        
        # break TODO: remove this
        break


    
    
    


if __name__ == '__main__':
    environment_name = 'looped_corridor'
    main(environment_name, "Priority_Greedy", 20, 10)

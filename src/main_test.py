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


def main(env_name, agent_name, t_a, t_r0):
    # -- get the environment files
    env_params = get_env_params(env_name)
    agent_priority_filename = get_agent_files(agent_name)
    # -- print the environment files
    print(f"Environment name: {env_name}")
    for key, filename in env_params.items():
        print(f"{key}: {filename}")
    print("-" * 50)

    # print(f"Agent name: {agent_name}")
    # for key, filename in agent_filenames.items():
    #     print(f"{key}: {filename}")
    # print("-" * 50)

    # -- set parameters
    # initial position
    # time before whistle
    # time after whistle

    # -- load the environment
    env = GraphExplorationEnvironment(env_params['edges'],
                                      env_params['vertex_tag'],
                                      agent_priority_filename,
                                      t_a, t_r0, env_params['init_pose'])
    # -- while loop
    # In each iteration, the output
    # -- initialize visited vertices

    # -- initialize flags

    # -- initialize file output to check results

    # -- initialize flags for solver


if __name__ == '__main__':
    environment_name = 'looped_corridor'
    main(environment_name)

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
from get_files_for_environment_agent import get_env_files


def main(env_name):
    # -- get the environment files
    env_filenames = get_env_files(env_name)
    # -- print the environment files
    print(f"Environment name: {env_name}")
    for key, filename in env_filenames.items():
        print(f"{key}: {filename}")
    print("-" * 50)

    # -- set parameters
    # initial position
    # time before whistle
    # time after whistle

    # -- load the environment
    env = GraphExplorationEnvironment(edges_filename,
                                      vertextag_filename,
                                      priority_filename,
                                      time_before_whistle,
                                      time_after_whistle,
                                      init_position)
    # -- initialize visited vertices

    # -- initialize flags

    # -- initialize file output to check results

    # -- initialize flags for solver


if __name__ == '__main__':
    environment_name = 'looped_corridor'
    main(environment_name)

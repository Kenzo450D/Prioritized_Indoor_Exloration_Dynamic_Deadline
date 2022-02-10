# Guide

## Example

execute `python3 priority_greedy_single-corridor_single-corridor.py`

## Parameters

#### Environment names
1. straight_corridor
2. looped_corridor
3. branched_corridor

#### Greedy algorithm options
1. `Priority_Greedy`
2. `Cost_Greedy`


## Executing exploration in an environment

1. Set up the environment.
2. Set up the exploration algorithm (named as `agent` in the example code)
3. Set up time:
    1. `t_a` refers to the time when the deadline is not available.
    2. `t_r0` refers to the deadline.
4. Set up the exploration agent by using an instance of the class `PriorityExplorationAgent`. 
5. Run the `explore()` function to explore the environment.

## Files in the code

1. `exploration_env.py` - Handles the exploration graph and time remaining and converts it to formats that can be used by the exploration algorithm. 
2. `env_graphs.py` - Handles the exploration graph and has functions to update it from the ground truth graph.
3. `env_clock.py` - Handles exploration time. Returns time when queried. Can update time remaining.
4. `priority_exploration_agent_greedy.py` - Is the priority exploration algorithm. Uses `node_tags` to identify importance of vertices.
    1. Has two functions to create greedy actions: `_choose_greedy_action` and `_choose_greedy_action_time`.
    2. Function `_choose_greedy_action` - choose a greedy action when deadline is not available.
    3. Function `_choose_greedy_action_time` - choose a greedy action when deadline is imposed.
5.  `int_wrapper.py` - Wraps integer node indexes to a wrapper class so that node indices are not compared against one another.
6.  `get_files_for_environment_agent.py` - Get input files for environment and exploration agent. 
7.  `priority_greedy_single-corridor_single-corridor.py` - Example code to run for a single environment.

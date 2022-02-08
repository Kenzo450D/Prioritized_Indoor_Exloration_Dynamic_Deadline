Files:
1. exploration_env.py
2. env_graphs.py
3. env_clock.py
4. priority_exploration_agent.py

# TODO
1. Remove remote and local notes and just keep one. At least for the interface that is available to others. 
2. What's the best way to consider the remote nodes from Up top? Write wrapper codes till basic ones are changed (2 hours)
3. Make a function in exploration_env to create a adjacency matrix for the explored graph. 
4. Write down clear documentation for:
    1. How do get the adjacency list as a dictionary for the explored graph?
    2. How to get the
5. Correct the priority values to reflect the changes make and make sure this works (Priority) DONE

## Get adjacency list of explored graph


## Remove remote and local nodes
1. Removed the visited vertices check in PEA. Lines 172-178
2. Created function `get_states_consolidated`.
3. Created file priority_exploration_agent_greedy to use the `get_states_consolidated` instead of `get_states`
4. Commented out `get_states` function as it was not being used.

from os.path import exists


def get_env_params(env_name):
    # -- initialize
    # vertex map has coordinate of every vertex on the image
    # each line in file is <vertex_id> <x_coord> <y_coord>
    vertex_map_filename = None  # map vertices to the figure
    edges_filename = None  # edge information of graph
    vtag_filename = None  # vertex tag information "corridor/large_room/small_room"
    bg_img_filename = None  # background image to plot

    # -- get vertex_map_filename
    if env_name == 'straight_corridor' or env_name == 'sc':
        vertex_map_filename = '../resources/environments/single_corridor/sc-env_mapping.txt'
        edges_filename = '../resources/environments/single_corridor/sc-weighted-edges.txt'
        vtag_filename = '../resources/environments/single_corridor/sc_vertex_tags.txt'
        bg_img_filename = '../resources/environments/single_corridor/rooms.png'
        init_pose = 16

    elif env_name == 'looped_corridor' or env_name == 'circular_corridor' or env_name == 'cc':
        vertex_map_filename = '../resources/environments/looped_corridor/looped_corridor-env_mapping.txt'
        edges_filename = '../resources/environments/looped_corridor/looped_corridor-edges_weighted.txt'
        vtag_filename = '../resources/environments/looped_corridor/looped_corridor-vertex_tags.txt'
        bg_img_filename = '../resources/environments/looped_corridor/rooms.png'
        init_pose = 40

    elif env_name == 'branched_corridor' or env_name == 'u_corridor' or env_name == 'bc' or env_name == 'uc':
        vertex_map_filename = '../resources/environments/u_corridor_large_room/bc-env_mapping.txt'
        edges_filename = '../resources/environments/u_corridor_large_room/bc-edges_weighted.txt'
        vtag_filename = '../resources/environments/u_corridor_large_room/bc-vertex_tags.txt'
        bg_img_filename = '../resources/environments/u_corridor_large_room/rooms.png'
        init_pose = 40

    else:
        print("Error: environment name is invalid")
        raise ValueError

    # -- check if the files exist
    if not exists(vertex_map_filename):
        print("File does not exist: ", vertex_map_filename)
        raise FileNotFoundError
    if not exists(edges_filename):
        print("File does not exist: ", edges_filename)
        raise FileNotFoundError
    if not exists(vtag_filename):
        print("File does not exist: ", vtag_filename)
        raise FileNotFoundError
    if not exists(bg_img_filename):
        print("File does not exist: ", bg_img_filename)
        raise FileNotFoundError

    # -- return the set variables
    env_params = {'vertex_map': vertex_map_filename,
                  'edges': edges_filename,
                  'vertex_tag': vtag_filename,
                  'background_image': bg_img_filename,
                  'init_pose': init_pose}
    return env_params


def get_agent_files(agent_name):
    priority_file = None
    # Note: When adding a new agent, add it in the 'else' condition.
    if agent_name == "alice" or agent_name == "Alice" or agent_name == "Priority_Greedy":
        priority_file = '../resources/greedy_agent/vertex_priority_prioritized.txt'
    elif agent_name == "bob" or agent_name == "Bob" or agent_name == "Cost_Greedy":
        priority_file = '../resources/greedy_agent/vertex_priority_equal.txt'
    else:
        print("Error: Agent name is invalid")
        print("Available Agents: (Use any one)")
        print("\tPriority_Greedy/alice/Alice")
        print("\tCost_Greedy/Bob/bob")
        raise ValueError

    if not exists(priority_file):
        print("Priority file does not exist, filename: ", priority_file)
        raise FileNotFoundError
    return priority_file

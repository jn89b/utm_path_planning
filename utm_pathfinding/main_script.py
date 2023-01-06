# -*- coding: utf-8 -*-
"""
Created on Mon Aug 22 13:39:48 2022

@author: jnguy
"""

"""

Map -> Grid -> Graph 

Allow adjustment for reconfigurable clusters 

Specify grid size
Break into clusters 

"""

#% Import stuff here
import numpy as np
import matplotlib.pyplot as plt
import math
import itertools
import random
from itertools import permutations 
from timeit import default_timer as timer


from mpl_toolkits.mplot3d import axes3d, Axes3D 

#from src import PathFinding
from utm_pathfinding import PathFinding, Map

#% Classes 

def generate_random_uav_coordinates(radius, x_bounds, y_bounds,  z_bounds, n_coords, obst_set):
    """generates random coordinates for uavs based on x bound, y bound, z bound
    and how many uavs you want and their proximity to each other"""
    # Generate a set of all points within 200 of the origin, to be used as offsets later
    # There's probably a more efficient way to do this.
    deltas = set()
    for x in range(-radius, radius+1):
        for y in range(-radius, radius+1):
            for z in range(-radius, radius+1):
                if x*x + y*y+ z*z <= radius*radius:
                    if (x,y,z) in obst_set:
                        continue
                    else:
                        deltas.add((x,y,z))
                        
    randPoints = []
    excluded = set()
    i = 0
    while i<n_coords:
        x = random.randrange(x_bounds[0], x_bounds[1])
        y = random.randrange(y_bounds[0], y_bounds[1])
        z = random.randint(z_bounds[0], z_bounds[1])
        if (x,y,z) in excluded or (x,y,z) in obst_set: 
            continue
        randPoints.append([x,y,z])
        i += 1
        excluded.update((x+dx, y+dy, z+dy) for (dx,dy,dz) in deltas)
    
    return randPoints
            
def get_coordinate(x_coord:float, y_coord:float, z_coord:float,
                   x_mesh:np.ndarray, y_mesh:np.ndarray, 
                   z_mesh:np.ndarray) -> tuple:
    """get coordinates"""
    
    return (x_mesh[x_coord,y_coord, z_coord], 
            y_mesh[x_coord,y_coord, z_coord], 
            z_mesh[x_coord,y_coord, z_coord])
    
def unravel_meshgrid(mesh_map:np.ndarray) ->np.ndarray:
    """unravel meshgrid and retruns array of [3, xlen*ylen*zlen] of matrices
    """
    return np.vstack(map(np.ravel, mesh_map))


def inflate_time(uav_vel:float, time_inflation:float, waypoints:list) -> list:
    """inflate time bubble, assuming dt is 1 second"""
    inflated_wp_time = []
    for i, wp in enumerate(waypoints):
        inflated_time = []
        for t in range(0, time_inflation):
            inflated_time.append((wp[0], wp[1], wp[2], wp[3]+t))
        
        inflated_wp_time.extend(inflated_time)
            
    return inflated_wp_time

def inflate_waypoints(waypoints:list, bubble_range:list) -> list:
    """inflates list of waypoints based on bound range"""
    inflated_waypoints = []
    for wp in waypoints:
        inflated_location = inflate_location(wp, bubble_range)
        inflated_waypoints.extend(inflated_location)
        
    return inflated_waypoints

def inflate_location(position:tuple, bubble_range:list) -> list:
    """inflate x,y,z locaiton position based on some bounds"""
    inflated_location = []
    """calculate bounds"""
    for i in bubble_range:
        for j in bubble_range:
            for k in bubble_range:
                new_position = [int(position[0]+i), 
                                int(position[1]+j), 
                                int(position[2]+k),
                                position[3]]
                inflated_location.append(tuple(new_position))

    return inflated_location

def create_bubble_bounds(col_radius:float)-> list:
    """create bubble size for UAS"""
    return list(np.arange(-col_radius, col_radius+1, 1))

def unpack_tuple_coords(tuple_coord_list:tuple)-> tuple:
    """unpacks list of tuples """
    x,y,z,t = zip(*tuple_coord_list) 
    return x,y,z,t

def smooth_high_path(path:list, path_to_append:list) -> list:
    """pass"""
    ## this smooths the path
    for k, p in enumerate(path): 

        if k+1 >= len(path):
            path_to_append.append(p)
            break                    
        
        if p in path_to_append:
            continue
        
        dist = math.dist(p, path[k+1])
        if abs(dist) <= 1.5:
            continue    
        
        path_to_append.append(p)
        
    return path_to_append


def get_refine_path(graph:np.meshgrid, abstract_path:list, 
                    reservation_table:set, 
                    col_bubble:int,weight_factor:int, curr_time:int, curr_vel:int) -> tuple:
    """
    get refined path for locations -> should use a set for obst coords
    returns tuple of waypoints, iteration count, and search count 
    """
    waypoint_coords = []
    iteration_cnt = 0
    search_cnt = 0
    
    if isinstance(abstract_path , int):
        return [],iteration_cnt,search_cnt
    
    if abstract_path is None:
        return [],iteration_cnt,search_cnt

    for i in range(len(abstract_path)):
        if i+1>= len(abstract_path):
            #print("final iteration", iteration_cnt)
            return waypoint_coords, iteration_cnt, search_cnt
        
        # print("going from", abstract_path[i], abstract_path[i+1])
        lowastar = PathFinding.AstarLowLevel(graph, 
                              reservation_table,
                              abstract_path[i], abstract_path[i+1], curr_vel, 
                              col_bubble, weight_factor, curr_time)
    
        waypoints= lowastar.main()
        
        
        if waypoints[0] == 0:
            return waypoints[2],iteration_cnt,search_cnt

        
        if waypoints is not None:
            #get time complexity and space complexity
            curr_time = waypoints[0][-1][-1]
            iteration_cnt += waypoints[1]
            search_cnt += len(waypoints[2])
            #print("length of dictionary is", len(waypoints[2]))
        
        if not waypoints:
            return waypoints[0],iteration_cnt,search_cnt
        
        if isinstance(waypoints[0], list): 
            waypoint_coords.extend(waypoints[0])
        else:
            print("final iteration", iteration_cnt)
            return waypoint_coords, iteration_cnt, search_cnt


def generate_random_uav_coordinates(radius, x_bounds, y_bounds,  z_bounds, n_coords, obst_set):
    """generates random coordinates for uavs based on x bound, y bound, z bound
    and how many uavs you want and their proximity to each other"""
    # Generate a set of all points within 200 of the origin, to be used as offsets later
    # There's probably a more efficient way to do this.
    n_coords
    deltas = set()
    for x in range(-radius, radius+1):
        for y in range(-radius, radius+1):
            for z in range(-radius, radius+1):
                if x*x + y*y+ z*z <= radius*radius:
                    if (x,y,z) in obst_set:
                        continue
                    else:
                        deltas.add((x,y,z))
                        
    randPoints = []
    excluded = set()
    i = 0
    while i<n_coords:
        x = random.randrange(x_bounds[0], x_bounds[1])
        y = random.randrange(y_bounds[0], y_bounds[1])
        z = random.randint(z_bounds[0], z_bounds[1])
        if (x,y,z) in excluded or (x,y,z) in obst_set: 
            continue
        randPoints.append((x,y,z))
        i += 1
        excluded.update((x+dx, y+dy, z+dy) for (dx,dy,dz) in deltas)
    
    return randPoints

def compute_actual_euclidean(position, goal):
    distance =  (((position[0] - goal[0]) ** 2) + 
                       ((position[1] - goal[1]) ** 2) +
                       ((position[2] - goal[2]) ** 2))**(1/2)
    
    return distance

def prioritize_uas(starting_list, goal_list, min_first = False):
    """Takes in start list, and goal list and 
    prioritizes UAS based on highest distance to be traversed"""
    
    dist_list = []
    for i, (start,goal) in enumerate(zip(starting_list,goal_list)):
        dist_val = compute_actual_euclidean(start,goal)
        dist_list.append((dist_val, start, goal))
    
    ##setting reverse to false sets to min first, true max first
    final_list = sorted(dist_list, key=lambda x: x[0], reverse=min_first)
    sorted_start = [start[1] for i, start in enumerate(final_list)]
    sorted_goal = [goal[2] for i, goal in enumerate(final_list)]

    return final_list, sorted_start, sorted_goal

def print_connections(position_key:tuple, level_graph:dict) -> None:
    connections = level_graph[str(position_key)]
    print("starting at", position_key)
    for con in connections:
        print(con.position)
        # print(con.node_type)
        
def evaluate_failure(path_dict):
    #vals = [v for x,v in path_dict.items()]
    position_list = [v.position for x,v in path_dict.items()]
    x = [x[0] for x in position_list]
    y = [x[1] for x in position_list]
    z = [x[2] for x in position_list]
    
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(x,y,z)


#%%
#% Main 
if __name__ == '__main__':
    plt.close('all')
    x_config = 200
    y_config = 200
    z_config = 50
    x_bounds = [0, x_config-1]
    y_bounds = [0, y_config-1]
    z_bounds = [0, z_config-1]
    
    #works with 4
    gs = 10
    
    """regions only work with even square roots"""
    map_area = Map.Map(x_config, y_config, z_config, gs)    
    
    n_regions = 4
    z_step = 10
    max_level = 1
    
    map_area.break_into_square_regions(4, z_step, 1)
    map_area.find_neighbor_regions(1)

    map_area.break_into_square_regions(4, z_step, 2)
    map_area.find_neighbor_regions(2)
    
    #%% 
    map_area.plot_regions(1, False)
    map_area.plot_regions(2, False)
    
    #position = map_area.unravel_meshgrid()
    #test = unravel_meshgrid(map_area.grid)
#%% Create abstract graph
    ab_graph = Map.AbstractGraph(map_area)
    ab_graph.build_corridors(1)
    ab_graph.build_airways(1)
    
    ab_graph.build_corridors(2)
    ab_graph.build_airways(2)
    graph_levels = ab_graph.graph_levels['2']
    region_levels = map_area.level_regions['2']
    
    #%% uav set up
    n_uavs = 40
    obst_set = set() 
    random_coords = generate_random_uav_coordinates(6, x_bounds, y_bounds, z_bounds, n_uavs*2, obst_set)
    begin_list = random_coords[0::2]
    end_list = random_coords[1::2]
    

    info_list, sorted_start, sorted_goal = prioritize_uas(begin_list, end_list)
    
    start_list = []
    for begin, end in zip(sorted_start, sorted_goal):
        start_list.append([begin,end])        

    
#%% High level search with low level search
    uas_bubble = 6
    uas_radius = uas_bubble/2     
    bubble = create_bubble_bounds(uas_radius)

    """
    2 UAS with opposite start and end position, 
    images instead of points for traversal 
    """
    uas_paths = []
    curr_vel = 2 #m/Ss # this
    vel_2 = 1 #m/s
    curr_time = 0 #s
    time_inflate = 5
    col_bubble = 4
    weight_factor = 5
    reserved_table = set()
    all_high_paths = []
    failures = []
        
    start_time = timer()
        
    """I need to refactor this"""
    for i,start in enumerate(start_list):
        if i % 5 == 0:
            print("at iteration", i)
        """high path search"""
        high_paths = []        
        if i % 1 == 0:
            set_vel = curr_vel
        else:
            set_vel = vel_2
        
        #loop from max level to low level search
        for i in reversed(range(max_level+1)):
            if i == 0:
                break
            
            ##start level
            if i == max_level:
                
                #determine starting location 
                region_start = map_area.find_which_region(start[0], i)
                region_end = map_area.find_which_region(start[1], i)
                
                ## if on same locationdo low level search instead
                if region_start == region_end:
                    high_paths = [start[0], start[1]]
                    refined_path, time, iter_count = get_refine_path(map_area.meshgrid, high_paths, 
                                                    reserved_table,col_bubble, weight_factor, 
                                                    curr_time, curr_vel)

                    if isinstance(refined_path, dict): 
                        #waypoint_coords.extend(waypoints[0])
                        failures.append((refined_path, time, start[0], start[1]))
                        continue

                    #check if we have correct path
                    if refined_path[-1][0:3] == start[1]:
                        t_inflate = inflate_time(uav_vel=curr_vel, 
                                                  time_inflation=time_inflate, 
                                                  waypoints=refined_path)
                        #remove this 
                        uas_paths.append(refined_path)
                        
                        inflated_list = inflate_waypoints(t_inflate, bubble)
                        reserved_table.update(inflated_list)                                    
                        continue
                    
                else:
                    ab_graph.insert_temp_nodes(start[0], 20, i)
                    ab_graph.insert_temp_nodes(start[1], 20, i)
                    
                    high_level_path = PathFinding.AstarGraph(ab_graph.graph_levels[str(i)], 
                                                             reserved_table, 
                                                             start[0], start[1],
                                                             set_vel, curr_time)
        
                    
                    high_path, time, iter_count  = high_level_path.main()
                    
                    if high_path == 0:
                        refined_path, time, iter_count = get_refine_path(map_area.meshgrid, [start[0], start[1]], 
                                                        reserved_table,col_bubble, weight_factor, 
                                                        curr_time, curr_vel)
                    else:
                        refined_path, time, iter_count = get_refine_path(map_area.meshgrid, high_path, 
                                                        reserved_table,col_bubble, weight_factor, 
                                                        curr_time, curr_vel)
                        
                    if isinstance(refined_path, dict): 
                        #waypoint_coords.extend(waypoints[0])
                        failures.append((refined_path, time, start[0], start[1]))
                        continue
                        
                    #check if we have correct path
                    if refined_path[-1][0:3] == start[1]:
                        t_inflate = inflate_time(uav_vel=curr_vel, 
                                                  time_inflation=time_inflate, 
                                                  waypoints=refined_path)
                        #remove this 
                        uas_paths.append(refined_path)
                        
                        inflated_list = inflate_waypoints(t_inflate, bubble)
                        reserved_table.update(inflated_list)                                    
                        continue
            
            print("\n")

    end_time = timer()
    time_diff = end_time - start_time
    print("time difference is ", time_diff)
                    
#%% animate
    plt.close('all')
                    
    animate_uas = PathFinding.AnimateMultiUAS(uas_paths=uas_paths, 
                                  method_name= str(len(uas_paths)/len(start_list)) + " UAS")
    
    animate_uas.plot_path(x_bounds=[0, x_config], 
                                  y_bounds=[0, y_config], 
                                  z_bounds=[0, z_config])    

    animate_uas.animate_multi_uas(x_bounds=[0, x_config], 
                                  y_bounds=[0, y_config], 
                                  z_bounds=[0, z_config],
                                  axis_on=True)

# #%% Testing out time appendices for weighted astar
#     uas_bubble = 6
#     uas_radius = uas_bubble/2     
#     bubble = create_bubble_bounds(uas_radius)

#     """
#     2 UAS with opposite start and end position, 
#     images instead of points for traversal 
#     """
#     curr_vel = 17 #m/Ss
#     vel_2 = 17 #m/s
#     curr_time = 0 #s
#     time_inflate = 10
#     col_bubble = 4
#     weight_factor = 10
    
#     start_list = [[(0,0,10), (x_config,y_config,10)],
#                   [(x_config,0,10), (0,y_config,10)],
#                   [(0,y_config/2,10), (x_config,y_config/2,10)],
#                   [(x_config/2,0,10), (x_config/2,y_config,10)]]
    
#     reserved_table = set()
    
#     paths = [] 
#     for start in start_list:
#         astar = PathFinding.AstarLowLevel(map_area.meshgrid, 
#                               reserved_table,
#                               start[0], start[1], curr_vel, 
#                               col_bubble, weight_factor, curr_time)
        
#         path, closed, visited = astar.main()
        
#         if path != 0:
#             t_inflate = inflate_time(uav_vel=curr_vel, 
#                                       time_inflation=time_inflate, 
#                                       waypoints=path)
            
#             inflated_list = inflate_waypoints(t_inflate, bubble)
#             reserved_table.update(inflated_list)
#             paths.append(path)
        
#         astar = PathFinding.AstarLowLevel(map_area.meshgrid, 
#                               reserved_table,
#                               start[1], start[0], vel_2, 
#                               col_bubble, weight_factor, curr_time)
        
#         path_2, closed_2, visited_2 = astar.main()
#         if path_2 != 0:
#             t_inflate = inflate_time(uav_vel=curr_vel, 
#                                       time_inflation=time_inflate, 
#                                       waypoints=path_2)
            
#             inflated_list = inflate_waypoints(t_inflate, bubble)
#             reserved_table.update(inflated_list)
#             paths.append(path_2)
    
    
#     """
#     Create an animation plot generator 
    
#     Discretizes time step based on the resolution desired
#     Discretizes positions based on this as well
    
#     """
        
#     plt.close('all')
    
#     animate_uas = PathFinding.AnimateMultiUAS(uas_paths=paths, 
#                                   method_name= str(len(paths)) + " UAS")
    
#     animate_uas.animate_multi_uas(x_bounds=[0, x_config], 
#                                   y_bounds=[0, y_config], 
#                                   z_bounds=[5, 15],
#                                   axis_on=True)
    

            
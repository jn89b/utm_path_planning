#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 17 22:18:48 2023

@author: justin
"""
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
import pickle


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


def generate_random_uav_coordinates(radius, x_bounds, y_bounds,  z_bounds, n_coords, obst_set,
   max_z_dist = 20, min_lateral_dist = 50, max_lateral_dist = 100, excluded=set()):
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
                        
    start_points = []
    end_points = []
    # excluded = set()
    i = 0

    while i<n_coords:
        
        start_x = random.randrange(x_bounds[0], x_bounds[1])
        start_y = random.randrange(y_bounds[0], y_bounds[1])
        start_z = random.randrange(z_bounds[0], z_bounds[1])

        #set end point based on min lateral distance
        end_x = random.randrange(start_x - min_lateral_dist, start_x + min_lateral_dist)        
        end_y = random.randrange(start_y - min_lateral_dist, start_y + min_lateral_dist)
        end_z = random.randrange(start_z - max_z_dist, start_z + max_z_dist)

        # x = random.randrange(x_bounds[0], x_bounds[1])
        # y = random.randrange(y_bounds[0], y_bounds[1])
        # z = random.randint(z_bounds[0], z_bounds[1])

        if (start_x, start_y, start_z) in excluded or (start_x, start_y, start_z) in obst_set:
            continue

        if (end_x, end_y, end_z) in excluded or (end_x, end_y, end_z) in obst_set:
            continue
        
        #check if end point is within max lateral distance
        if math.dist((start_x, start_y, start_z), (end_x, end_y, end_z)) <= min_lateral_dist:
            continue

        #check if end point is out of bounds
        if end_x < x_bounds[0] or end_x > x_bounds[1]:
            continue
        if end_y < y_bounds[0] or end_y > y_bounds[1]:
            continue
        if end_z < z_bounds[0] or end_z > z_bounds[1]:
            continue

    
        start_points.append((start_x, start_y, start_z))
        end_points.append((end_x, end_y, end_z))

        #randPoints.append((x,y,z))
        i += 1
        excluded.update((start_x+dx, start_y+dy, start_z+dz) for (dx,dy,dz) in deltas)
        excluded.update((end_x+dx, end_y+dy, end_z+dz) for (dx,dy,dz) in deltas)
        
        #excluded.update((x+dx, y+dy, z+dy) for (dx,dy,dz) in deltas)

    
    return start_points, end_points

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

#function to save dictionary to pickle
def save_to_pickle(pickle_file_name, data):
    with open(pickle_file_name+'.pkl', 'wb') as f:
        pickle.dump(data, f, pickle.HIGHEST_PROTOCOL)
        

#%%
#% Main 
if __name__ == '__main__':
    plt.close('all')
    x_config = 500
    y_config = 500
    z_config = 12 #100
    x_bounds = [0, x_config-1]
    y_bounds = [0, y_config-1]
    z_bounds = [0, z_config-1]
    
    #works with 4
    gs = 50
    
    """regions only work with even square roots"""
    map_area = Map.Map(x_config, y_config, z_config, gs)    
    
    n_regions = 25
    z_step = 1
    max_level = 1
    
    map_area.break_into_square_regions(n_regions, z_step, max_level)
    map_area.find_neighbor_regions(1)
    
    
    #%% 
    map_area.plot_regions(1, False)
    # map_area.plot_regions(2, False)
    
    #position = map_area.unravel_meshgrid()
    #test = unravel_meshgrid(map_area.grid)
#%% Create abstract graph
    ab_graph = Map.AbstractGraph(map_area)
    ab_graph.build_corridors(1)
    ab_graph.build_airways(1)
    
    # ab_graph.build_corridors(2)
    # ab_graph.build_airways(2)
    graph_levels = ab_graph.graph_levels['1']
    region_levels = map_area.level_regions['1']
    
    #%% 
    map_area.plot_regions(1)
    
    #%% uav set up
    n_uavs = 100
    obst_set = set() 
    
    col_bubble = np.random.randint(1, 3) # to 
    
    #round up to nearest even number
    col_radius = (col_bubble + col_bubble % 2)

    bubble = create_bubble_bounds(col_radius)
    max_z = 20

    #compute max lateral distance of map area
    max_lateral_distance = ((x_config**2 + y_config**2)**(1/2))/1.5
    min_lateral_distance = max_lateral_distance/1.5


    begin_list, end_list = generate_random_uav_coordinates(int(col_radius), 
                                                           x_bounds, y_bounds, 
                                                           z_bounds, n_uavs, 
                                                           obst_set, max_z,
                                                           int(min_lateral_distance),
                                                           int(max_lateral_distance))

    info_list, sorted_start, sorted_goal = prioritize_uas(begin_list, end_list)
    
    start_list = []
    for begin, end in zip(sorted_start, sorted_goal):
        start_list.append([begin,end])        

    
#%% High level search with low level search

    uas_paths = []
    uas_inflated_paths = []
    time_list = []
    
    #random velocity 
    curr_vel = np.random.randint(1,4)
    curr_time = 0 #s
    time_inflate = 5
    weight_factor = 5
    reserved_table = set()
    all_high_paths = []
    failures = []

    max_height_climb = 20
    start_time = timer()
    log_index_indicator = 5
    """I need to refactor this"""
    for i,start in enumerate(start_list):
        
        #round time to nearest second
        curr_time = int(round(timer() - start_time, 0))

        #loop through uas paths
        for uas in uas_inflated_paths:
            last_wp = uas[-1]
            if last_wp[-1]<= curr_time:
                curr_time = int(round(timer() - start_time, 0))
                # print("popping off uas from reserved table", last_wp)
                #remove all values from reservation table
                print("removing uas from reserved table")    
                for wp in uas:
                    #check if in reserved table
                    if wp in reserved_table:
                        reserved_table.remove(wp)
                        #remove from uas_inf_paths
                uas_inflated_paths.remove(uas)
                        
                
        if i % log_index_indicator == 0:
            print("current time", curr_time)
            print("at iteration", i)
        
        
        """high path search"""
        high_paths = []        
        
        #loop from max level to low level search
        for i in reversed(range(max_level+1)):
            if i == 0:
                break
            
            ##start level
            if i == max_level:
                
                #determine starting location 
                region_start = map_area.find_which_region(start[0], i)
                region_end = map_area.find_which_region(start[1], i)
                
                ## if on same location do low level search instead
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
                    ab_graph.insert_temp_nodes(start[0], max_height_climb, i)
                    ab_graph.insert_temp_nodes(start[1], max_height_climb, i)
                    
                    high_level_path = PathFinding.AstarGraph(ab_graph.graph_levels[str(i)], 
                                                             reserved_table, 
                                                             start[0], start[1],
                                                             curr_vel, curr_time)
        
                    
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
                        uas_inflated_paths.append(inflated_list)
                        reserved_table.update(inflated_list)                                    
                        continue
                    
            print("\n")
            #round time to nearest second
            dt = int(round(timer() - curr_time, 0))
            time_list.append(dt)



    end_time = timer()
    time_diff = end_time - start_time
    print("time difference is ", time_diff)
    
    #%% 
    info_dict = {
        "time": time_list,
        "iterations": iter_count,
        "failures": failures,
        "uas_paths": uas_paths,
        "start_list": start_list,
        "end_list": end_list,
        # "sorted_radius": sorted_radius,
        # "time_inflate_list": time_inflate_list,
        # "velocity_list": velocity_list,
    }

    pickle_name = 'test'
    #save to pickle to monte_carlo_data
    save_to_pickle("monte_carlo_data/"+pickle_name, 
        info_dict)

                    
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
    

            
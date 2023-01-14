#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 13 19:04:10 2023

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
import pickle
import os 

#import garbage collector
import gc

from itertools import permutations 
from timeit import default_timer as timer


from mpl_toolkits.mplot3d import axes3d, Axes3D 

#from src import PathFinding
from utm_pathfinding import PathFinding, Map

#% Classes 
def generate_random_uav_coordinates(radius_bounds, x_bounds, y_bounds,  z_bounds, n_coords, obst_set,
   max_z_dist = 20, min_lateral_dist = 50, max_lateral_dist = 100):
    """generates random coordinates for uavs based on x bound, y bound, z bound
    and how many uavs you want and their proximity to each other"""
    # Generate a set of all points within 200 of the origin, to be used as offsets later
    # There's probably a more efficient way to do this.
    n_coords
    radius_info = []
    deltas = set()
    #random radius 
    for i in range(n_coords):   
        radius = random.randrange(radius_bounds[0], radius_bounds[1])
        for x in range(-radius, radius+1):
            for y in range(-radius, radius+1):
                for z in range(-radius, radius+1):
                    if x*x + y*y+ z*z <= radius*radius:
                        if (x,y,z) in obst_set:
                            continue
                        else:
                            deltas.add((x,y,z))
        radius_info.append(radius)
                        
    start_points = []
    end_points = []
    excluded = set()
    i = 0

    while i<n_coords:
        
        start_x = random.randrange(x_bounds[0], x_bounds[1])
        start_y = random.randrange(y_bounds[0], y_bounds[1])
        start_z = random.randrange(z_bounds[0], z_bounds[1])

        #set end point based on min lateral distance
        end_x = random.randrange(start_x - min_lateral_dist, start_x + min_lateral_dist)        
        end_y = random.randrange(start_y - min_lateral_dist, start_y + min_lateral_dist)
        end_z = random.randrange(start_z - max_z_dist, start_z + max_z_dist)

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
        
    return start_points, end_points, radius_info

            
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



def compute_actual_euclidean(position, goal):
    distance =  (((position[0] - goal[0]) ** 2) + 
                       ((position[1] - goal[1]) ** 2) +
                       ((position[2] - goal[2]) ** 2))**(1/2)
    
    return distance

def prioritize_uas(starting_list, goal_list, radius_uav_list, min_first = False):
    """Takes in start list, and goal list and 
    prioritizes UAS based on highest distance to be traversed"""
    
    dist_list = []
    for i, (start,goal,radius) in enumerate(zip(starting_list,goal_list, radius_uav_list)):
        dist_val = compute_actual_euclidean(start,goal)
        dist_list.append((dist_val, start, goal, radius))
    
    ##setting reverse to false sets to min first, true max first
    final_list = sorted(dist_list, key=lambda x: x[0], reverse=min_first)
    sorted_start = [start[1] for i, start in enumerate(final_list)]
    sorted_goal = [goal[2] for i, goal in enumerate(final_list)]
    sorted_radius = [radius[3] for i, radius in enumerate(final_list)]

    return final_list, sorted_start, sorted_goal, sorted_radius

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

def get_map_from_pickle(pickle_file):
    with open(pickle_file+'.pkl', 'rb') as f:
        map_area = pickle.load(f)
    return map_area

#function to save dictionary to pickle
def save_to_pickle(pickle_file_name, data):
    with open(pickle_file_name+'.pkl', 'wb') as f:
        pickle.dump(data, f, pickle.HIGHEST_PROTOCOL)
        
#%%
#% Main 
if __name__ == '__main__':
    
    #load in map from pickle 
    folder_dir = os.getcwd()
    folder_name = 'maps'
    map_type = 'medium'

    # base_map = get_map_from_pickle(folder_name+'/base')
    # small_map = get_map_from_pickle(folder_name+'/small')
    # medium_map = get_map_from_pickle(folder_name+'/medium')
    map_info= get_map_from_pickle(folder_name+'/'+map_type)
    map_area = map_info['map_area']
    ab_graph = map_info['graph']
    
    max_level = 1

    #%% Set up configs
    x_config = map_area.x_array[-1]
    y_config = map_area.y_array[-1]
    z_config = map_area.z_array[-1]

    x_bounds = [map_area.x_array[0], map_area.x_array[-1]]
    y_bounds = [map_area.y_array[0], map_area.y_array[-1]]
    z_bounds = [map_area.z_array[0], map_area.z_array[-1]]
    
    
    #%% SET UP SIMULATION
    
    #list of 50 to 100 uavs
    number_sims = 500
    uavs = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
    
    min_max_vel = [10, 20]
    min_max_time_inflate = [5, 15]

    # n_uavs = 10
    obst_set = set() 
    max_z_climb = 20
    log_index_indicator = 5
    uav_number_indicator = 10 #number of uavs to print out
    
    for uav in uavs:
        for k in range(number_sims+1):

            #garbage collection
            gc.collect()
            
            #check iteration number modulus
            if k % log_index_indicator == 0:
                print("iteration", k)

            uas_paths = []
            all_high_paths = []
            failures = []
            col_radius_list = []
            velocity_list = []
            time_inflate_list = []

            curr_time = 0 #s
            # time_inflate = 5
            weight_factor = 20
            reserved_table = set()

            #compute max lateral distance of map area
            max_lateral_distance = ((x_config**2 + y_config**2)**(1/2))/1.5
            min_lateral_distance = max_lateral_distance/1.5
            uav_radius_bounds = [2, 5]
            begin_list, end_list, radius_info = generate_random_uav_coordinates(uav_radius_bounds, 
                                                                x_bounds, y_bounds, 
                                                                z_bounds, uav, 
                                                                obst_set, max_z_climb,
                                                                int(min_lateral_distance),
                                                                int(max_lateral_distance))

            info_list, sorted_start, sorted_goal, sorted_radius = prioritize_uas(
                begin_list, end_list, radius_info)
            
            start_list = []
            for begin, end, radius in zip(sorted_start, sorted_goal, sorted_radius):
                start_list.append([begin,end, radius])        

            
            #begin simulation
            start_time = timer()

            """I need to refactor this"""
            for i,start in enumerate(start_list):
                
                if i % uav_number_indicator == 0:
                    print("uav", i)

                #set random stuff
                col_radius = start[2]
                bubble = create_bubble_bounds(col_radius)
                col_bubble = create_bubble_bounds(col_radius*2)

                time_inflate = random.randint(min_max_time_inflate[0], 
                                min_max_time_inflate[1])
                curr_vel = random.randint(min_max_vel[0], min_max_vel[1])

                # col_radius_list.append(col_radius)
                time_inflate_list.append(time_inflate)
                velocity_list.append(curr_vel)


                #round time to nearest second
                curr_time = int(round(timer() - start_time, 0))
                
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
                            
                            ab_graph.insert_temp_nodes(start[0], max_z_climb, i)
                            ab_graph.insert_temp_nodes(start[1], max_z_climb, i)
                            
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
                                reserved_table.update(inflated_list)                                    
                                continue
                            

            end_time = timer()
            time_diff = end_time - start_time
            
            #%% 
            info_dict = {
                "time": time_diff,
                "map_type": map_type,
                "iterations": iter_count,
                "failures": failures,
                "uas_paths": uas_paths,
                "start_list": start_list,
                "end_list": end_list,
                "sorted_radius": sorted_radius,
                "time_inflate_list": time_inflate_list,
                "velocity_list": velocity_list,
            }

            pickle_name = map_type+"_sim_"+str(k)+"_uavs_"+str(uav)
            #save to pickle to monte_carlo_data
            save_to_pickle("monte_carlo_data/"+map_type+"/"+pickle_name, 
                info_dict)



#%% animate
    # plt.close('all')
                    
    # animate_uas = PathFinding.AnimateMultiUAS(uas_paths=uas_paths, 
    #                               method_name= str(len(uas_paths)/len(start_list)) + " UAS")
    
    # animate_uas.plot_path(x_bounds=[0, x_config], 
    #                               y_bounds=[0, y_config], 
    #                               z_bounds=[0, z_config])    

    # animate_uas.animate_multi_uas(x_bounds=[0, x_config], 
    #                               y_bounds=[0, y_config], 
    #                               z_bounds=[0, z_config],
    #                               axis_on=True)


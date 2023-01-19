"""
Each grid point is a hub: 
    - Each grid is 8m x 8m x 1m 
    - So 500 x 500 grid means:
        4km x 4km grid
    - Speeds vary from:
        8 m/s to 32 m/s
    - Bubble size varies:
        8m to 24m diameter

Break up from hub spawns and random spawns

Hub spawns spawn from some known location area 

"""
from utm_pathfinding import data_utils, PathFinding, Map, data_utils
from config import config
import numpy as np
import math
import random
import os
import pickle
from timeit import default_timer as timer
import matplotlib.pyplot as plt


x_config = 500
y_config = 500

def get_map_from_pickle(pickle_file):
    with open(pickle_file+'.pkl', 'rb') as f:
        map_area = pickle.load(f)
    return map_area


def gen_random_coords(radius, x_bounds, y_bounds,  z_bounds, n_coords, obst_set,
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
        #choose to randomize end point or not

        # if rand_num == 1:
        #     end_x = random.randrange(start_x - min_lateral_dist, start_x + min_lateral_dist)        
        #     end_y = start_y 
        #     end_z = random.randrange(start_z - max_z_dist, start_z + max_z_dist)

        # elif rand_num == 2:
        #     end_x = start_x
        #     end_y = random.randrange(start_y - min_lateral_dist, start_y + min_lateral_dist)
        #     end_z = random.randrange(start_z - max_z_dist, start_z + max_z_dist)

        # else:
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
        if end_x < x_bounds[0] or end_x > x_config:
            continue
        if end_y < y_bounds[0] or end_y > y_config:
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
        
        # #check if end point is within max lateral distance
        # if math.dist((start_x, start_y, start_z), (end_x, end_y, end_z)) <= min_lateral_dist:
        #     continue

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

def create_bubble_bounds(col_radius:float)-> list:
    """create bubble size for UAS"""
    return list(np.arange(-col_radius, col_radius+1, 1))

if __name__=='__main__':
    
    
    #load in map from pickle 
    folder_dir = os.getcwd()
    folder_name = 'maps'
    map_type = 'sim_world'

    map_info= get_map_from_pickle(folder_name+'/'+map_type)
    map_area = map_info['map_area']
    ab_graph = map_info['graph']

    max_level = 1
    unit_per_cell = 8 #8m per cell
    n_cells = 500 

    x_config = 500
    y_config = 500
    z_config = 100
    x_map_bounds = [0, x_config-1]
    y_map_bounds = [0, y_config-1]
    z_map_bounds = [0, z_config-1]
    
    #works with 4
    gs = 25

    #hub coordinates
    #lets say hub is 80m x 80 x 20m -> convert this 
    hub_x_size_meter = 520
    hub_y_size_meter = 520
    hub_z_size_meter = 80

    hub_x= [0, hub_x_size_meter/unit_per_cell]
    hub_y = [0, hub_y_size_meter/unit_per_cell]
    hub_z = [0, 40]

    radius = 10
    max_z = 20
    
    n_hub_uavs = config.NUMBER_UAV_HUBS

    #need to see if I can reroute the uavs to the hub back
    #generate random uav coordinates based on the hub coordinates
    #compute max lateral distance of map area
    max_lateral_distance = ((x_config**2 + y_config**2)**(1/2))/2
    min_lateral_distance = max_lateral_distance/1.5
    reserved_table = set()
    excluded_set = set()



#%% Plot  
    map_area.plot_regions(1, True)

    
#%% High level search with low level search

    uas_paths = []
    uas_inflated_paths = []
    time_list = []
    
    #random velocity 
    curr_time = 0 #s
    time_inflate = 3
    weight_factor = 5
    reserved_table = set()
    all_high_paths = []
    failures = []

    max_height_climb = 20
    start_time = timer()
    log_index_indicator = 5

    overall_start_list = []
    overall_end_list = []
    
    #%% 
    #SIMULATE 
    while len(uas_paths) <= config.NUMBER_TOTAL_OPERATIONS+1:
         
        n_hub_uavs = random.randint(config.MIN_SPAWN, config.NUMBER_UAV_HUBS)
        n_random_uavs = random.randint(config.MIN_SPAWN, config.NUMBER_RANDOM_UAVS)

        random_set = set()
        print("current number of uas paths", len(uas_paths))
        uav_hub_start, uav_hub_end = gen_random_coords(int(radius), 
                                    hub_x, hub_y, hub_z, 
                                    n_hub_uavs, random_set, 
                                    int(min_lateral_distance),
                                    int(max_lateral_distance))

        random_begin_list, random_end_list = generate_random_uav_coordinates(
            int(radius), x_map_bounds, y_map_bounds, z_map_bounds,  n_random_uavs,
            random_set, int(min_lateral_distance), int(max_lateral_distance)
        )

        total_begin = uav_hub_start + random_begin_list + uav_hub_end
        total_end = uav_hub_end + random_end_list + uav_hub_start
        
        info_list, sorted_start, sorted_goal = prioritize_uas(total_begin, total_end)
        
        start_list = []
        end_list = []

        for begin, end in zip(sorted_start, sorted_goal):
            start_list.append([begin,end])

        overall_start_list.extend(start_list)
        overall_end_list.extend(sorted_goal)

        """I need to refactor this"""
        for i,start in enumerate(start_list):
            curr_vel = np.random.randint(1,4)
            col_bubble = np.random.randint(1, 3) # to 
            col_radius = col_bubble/2

            bubble = create_bubble_bounds(col_radius)
            #round time to nearest second
            curr_time = int(round(timer() - start_time, 0))

            #loop through uas paths
            for uas in uas_inflated_paths:
                last_wp = uas[-1]
                if last_wp[-1]+time_inflate <= curr_time:
                    #remove all values from reservation table
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
        "start_list": overall_start_list,
        "end_list": overall_end_list,
        # "sorted_radius": sorted_radius,
        # "time_inflate_list": time_inflate_list,
        # "velocity_list": velocity_list,
    }

    pickle_name = 'hub_sim_01'
    #save to pickle to monte_carlo_data
    data_utils.save_to_pickle("monte_carlo_data/"+pickle_name, 
        info_dict)


#%% animate
    plt.close('all')
                    
    animate_uas = PathFinding.AnimateMultiUAS(uas_paths=uas_paths, 
                                  method_name= str(len(uas_paths)/len(overall_start_list)) + " UAS")
    
    animate_uas.plot_path(x_bounds=[0, x_config], 
                                  y_bounds=[0, y_config], 
                                  z_bounds=[0, z_config])    

    animate_uas.animate_multi_uas(x_bounds=[0, x_config], 
                                  y_bounds=[0, y_config], 
                                  z_bounds=[0, z_config],
                                  axis_on=True)






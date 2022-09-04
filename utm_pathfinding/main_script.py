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

from mpl_toolkits.mplot3d import axes3d, Axes3D 

#from src import PathFinding
from utm_pathfinding import PathFinding, Map

#% Classes 
class UAV():
    def __init__(self, id_name, start, goal, vel, size, config):
        pass
            

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
        if abs(dist) <= 0.75:
            continue    
        
        path_to_append.append(p)
        
    return path_to_append


def get_refine_path(graph:np.meshgrid, abstract_path:list, 
                    reservation_table:set, 
                    col_bubble:int,weight_factor:int, curr_time:int) -> tuple:
    """get refined path for locations -> should use a set for obst coords"""
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
        
        # lowastar = PathFinding.AstarLowLevel(
        #     graph, reservation_table, obstacle_coords,
        #     abstract_path[i], abstract_path[i+1], col_bubble, weighted_h
        #     )
        
        lowastar = PathFinding.AstarLowLevel(graph, 
                              reservation_table,
                              abstract_path[i], abstract_path[i+1], curr_vel, 
                              col_bubble, weight_factor, curr_time)
        

        waypoints= lowastar.main()
        if waypoints is not None:
            #get time complexity and space complexity
            iteration_cnt += waypoints[1]
            search_cnt += len(waypoints[2])
            #print("length of dictionary is", len(waypoints[2]))
        
        if not waypoints:
            return [],iteration_cnt,search_cnt
        
        if isinstance(waypoints[0], list): 
            waypoint_coords.extend(waypoints[0])
        else:
            #print("final iteration", iteration_cnt)
            return waypoint_coords, iteration_cnt, search_cnt

#%%
#% Main 
if __name__ == '__main__':
    plt.close('all')
    x_config = 200
    y_config = 200
    z_config = 75
    #works with 4
    gs = 10
    
    """regions only work with even square roots"""
    map_area = Map.Map(x_config, y_config, z_config, gs)    
    
    n_regions = 16
    z_step = 5
    max_level = 2
    
    map_area.break_into_square_regions(n_regions, z_step, 1)
    map_area.find_neighbor_regions(1)

    map_area.break_into_square_regions(4, z_step, 2)
    map_area.find_neighbor_regions(2)
    
    map_area.plot_regions(1, True)
    map_area.plot_regions(2, True)
    
    #position = map_area.unravel_meshgrid()
    #test = unravel_meshgrid(map_area.grid)
#%% Create abstract graph
    ab_graph = Map.AbstractGraph(map_area)
    ab_graph.build_corridors(1)
    ab_graph.build_airways(1)
    
    ab_graph.build_corridors(2)
    ab_graph.build_airways(2)
    graph_levels = ab_graph.graph_levels['1']
    region_levels = map_area.level_regions['1']
#%% Testhing this out, I want to create all permutations, for graph should have 3 for each node
    # graph = {}    
    # for key, region in region_levels.items():
    #         all_sides = []
    #         for side_key, side in region.region_sides.items():
    #             if side:
    #                 all_sides.extend(side)
                    
                    
    #         for position in all_sides:
    #             if position not in graph:
    #                 neighbors = [x for x in all_sides if x != position]
                    
    #                 for neighbor in neighbors:
    #                     print(neighbor)
                    
    
#%% astar graph test
    start = (21,1,25)
    end = (193, 179, 15)
    
    #this will loop and add the temp nodes in each level    
    for i in range(1,max_level+1):
        ab_graph.insert_temp_nodes(start, 20, i)
        ab_graph.insert_temp_nodes(end, 20, i)

    reservation_table = {}
    curr_vel = 1 #m/Ss
    curr_time = 0

    high_paths = []
    
    for i in reversed(range(max_level+1)):
        if i == 0:
            break
        
        ##start level
        if i == 2:            
            level_graph = ab_graph.graph_levels[str(i)]
            astar_graph = PathFinding.AstarGraph(level_graph, reservation_table,
                                                start, end, curr_vel, curr_time)
            path = astar_graph.main()
            
        else:
            for j, coords in enumerate(path):
                if j+1 >= len(path):
                    print("im done", i, coords)
                    break
                
                ab_graph.insert_temp_nodes(coords, 20, i)
                ab_graph.insert_temp_nodes(path[j+1], 20, i)
                
                level_graph = ab_graph.graph_levels[str(i)]
                astar_graph = PathFinding.AstarGraph(level_graph, reservation_table,
                                                    coords, path[j+1], curr_vel, curr_time)                
                some_path = astar_graph.main()
                            
                smooth_high_path(some_path, high_paths)

                
    #%% animte high level path
    plt.close('all')
    paths = [path]
        
    animate_uas = PathFinding.AnimateMultiUAS(uas_paths=paths, 
                                  method_name= str(len(paths)) + " UAS")
    
    animate_uas.plot_path(x_bounds=[0, x_config], 
                                  y_bounds=[0, y_config], 
                                  z_bounds=[0, z_config])


    animate_uas = PathFinding.AnimateMultiUAS(uas_paths=[high_paths], 
                                  method_name= str(len(paths)) + " UAS")
    
    animate_uas.plot_path(x_bounds=[0, x_config], 
                                  y_bounds=[0, y_config], 
                                  z_bounds=[0, z_config])
    
#%% High level search with low level search
    uas_bubble = 6
    uas_radius = uas_bubble/2     
    bubble = create_bubble_bounds(uas_radius)

    """
    2 UAS with opposite start and end position, 
    images instead of points for traversal 
    """
    curr_vel = 10 #m/Ss
    vel_2 = 3 #m/s
    curr_time = 0 #s
    time_inflate = 15
    col_bubble = 4
    weight_factor = 10
    reserved_table = {}
    
    start_list = [[(0,0,10), (x_config-1,y_config-1,10)]]
                  #[(x_config,0,10), (0,y_config-1,10)]]
    
    for start in start_list:
        print("starting at", start[1]) 
        
        for index in range(1,max_level+1):
            ab_graph.insert_temp_nodes(start[0], 20, index)
            ab_graph.insert_temp_nodes(start[1], 20, index)
    
        reservation_table = {}
        
        """high path search"""
        high_paths = []
        for i in reversed(range(2+1)):
            if i == 0:
                break
            
            ##start level
            if i == 2:            
                level_graph = ab_graph.graph_levels[str(i)]
                astar_graph = PathFinding.AstarGraph(level_graph, reservation_table,
                                                    start[0], start[1], curr_vel, curr_time)
                path = astar_graph.main()
                
            else:
                for j, coords in enumerate(path):
                    if j+1 >= len(path):
                        break
                    
                    ab_graph.insert_temp_nodes(coords, 20, i)
                    ab_graph.insert_temp_nodes(path[j+1], 20, i)
                    
                    level_graph = ab_graph.graph_levels[str(i)]
                    astar_graph = PathFinding.AstarGraph(level_graph, reservation_table,
                                                        coords, path[j+1], curr_vel, curr_time)                
                    some_path = astar_graph.main()
                                
                    ## this smooths the path
                    high_paths = smooth_high_path(some_path, high_paths)

                        
        """low search"""
        # for i,wp in enumerate(high_paths):
        #     if i+1>=len(high_paths):
        #         print("done")
        refined_path = get_refine_path(map_area.meshgrid, high_paths, 
                                       reserved_table,
                                       col_bubble, weight_factor, curr_time)
            
    
#%% Testing out time appendices for weighted astar
    uas_bubble = 6
    uas_radius = uas_bubble/2     
    bubble = create_bubble_bounds(uas_radius)

    """
    2 UAS with opposite start and end position, 
    images instead of points for traversal 
    """
    curr_vel = 10 #m/Ss
    vel_2 = 3 #m/s
    curr_time = 0 #s
    time_inflate = 15
    col_bubble = 4
    weight_factor = 10
    
    start_list = [[(0,0,10), (x_config,y_config,10)],
                  [(x_config,0,10), (0,y_config,10)],
                  [(0,y_config/2,10), (x_config,y_config/2,10)],
                  [(x_config/2,0,10), (x_config/2,y_config,10)]]
    
    reserved_table = set()
    
    paths = [] 
    for start in start_list:
        astar = PathFinding.AstarLowLevel(map_area.meshgrid, 
                              reserved_table,
                              start[0], start[1], curr_vel, 
                              col_bubble, weight_factor, curr_time)
        
        path, closed, visited = astar.main()
        
        if path != 0:
            t_inflate = inflate_time(uav_vel=curr_vel, 
                                      time_inflation=time_inflate, 
                                      waypoints=path)
            
            inflated_list = inflate_waypoints(t_inflate, bubble)
            reserved_table.update(inflated_list)
            paths.append(path)
        
        astar = PathFinding.AstarLowLevel(map_area.meshgrid, 
                              reserved_table,
                              start[1], start[0], vel_2, 
                              col_bubble, weight_factor, curr_time)
        
        path_2, closed_2, visited_2 = astar.main()
        if path_2 != 0:
            t_inflate = inflate_time(uav_vel=curr_vel, 
                                      time_inflation=time_inflate, 
                                      waypoints=path_2)
            
            inflated_list = inflate_waypoints(t_inflate, bubble)
            reserved_table.update(inflated_list)
            paths.append(path_2)
    
    
    """
    Create an animation plot generator 
    
    Discretizes time step based on the resolution desired
    Discretizes positions based on this as well
    
    """
        
    plt.close('all')
    
    animate_uas = PathFinding.AnimateMultiUAS(uas_paths=paths, 
                                  method_name= str(len(paths)) + " UAS")
    
    animate_uas.animate_multi_uas(x_bounds=[0, x_config], 
                                  y_bounds=[0, y_config], 
                                  z_bounds=[5, 15],
                                  axis_on=True)
    

                
            
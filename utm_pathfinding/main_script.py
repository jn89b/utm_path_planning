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

#%%
#% Main 
if __name__ == '__main__':
    plt.close('all')
    x_config = 100
    y_config = 100
    z_config = 100
    gs = 5
    map_area = Map.Map(x_config, y_config, z_config, gs)    
    
    n_regions = 4
    z_step = 25
    map_area.break_into_square_regions(n_regions, z_step)
    map_area.find_neighbor_regions()
    
    #create abstract graph
    #do this by 
    #    
    map_area.plot_regions(True)
    
    #position = map_area.unravel_meshgrid()
    #test = unravel_meshgrid(map_area.grid)
#%% Create abstract graph
    ab_graph = Map.AbstractGraph(map_area)
    ab_graph.build_corridors()
    ab_graph.build_airways()
    test = ab_graph.graph
    
#%% astar graph test
    start = (250,250,75)
    end = (485, 369, 62)
    height_bound = 30
    
    ab_graph.insert_temp_nodes(start, height_bound )
    ab_graph.insert_temp_nodes(end, height_bound )
    
    reservation_table = {}
    astar_graph = PathFinding.AstarGraph(ab_graph.graph, reservation_table,
                                         start, end)
    path = astar_graph.main()
    
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
    for i,start in enumerate(start_list):
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
    

                
            
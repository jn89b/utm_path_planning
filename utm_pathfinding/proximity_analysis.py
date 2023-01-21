# -*- coding: utf-8 -*-
"""
Created on Fri Jan 20 16:37:17 2023

@author: jnguy
"""



#% Import stuff here
import pickle as pkl
import seaborn as sns
import numpy as np
import matplotlib.pyplot as plt
from utm_pathfinding import data_utils as data_analysis, PathFinding

#% Classes 

def scale_paths(paths:list, unit_value=1):
    
    all_scaled_paths = []
    for path in paths:
        
        scaled_path = []
        
        for point in path:
            x = point[0] * unit_value
            y = point[1] * unit_value
            z = point[2] * unit_value
            
            t = point[3]
            
            scaled_path.append((x, y, z, t))
        
        all_scaled_paths.append(scaled_path)

    return all_scaled_paths

def interpolate_paths(paths:list, time_step=1):
    
    all_interpolated_paths = []
    for path in paths:
        
        interpolated_path = []
        
        for i, point in enumerate(paths):

            if i == len(paths) - 1:
                break
            
            x_traverse = np.linspace(point[0], paths[i+1][0], time_step)
            y_traverse = np.linspace(point[1], paths[i+1][1], time_step)
            z_traverse = np.linspace(point[2], paths[i+1][2], time_step)
            t_traverse = np.linspace(point[3], paths[i+1][3], time_step)            
            
            # print(x_traverse)
            # print(y_traverse)
            # print(z_traverse)
            
            for x,y,z,t in zip(x_traverse, y_traverse, z_traverse, t_traverse):
                interpolated_path.append((x, y, z, t))
            
        all_interpolated_paths.append(interpolated_path)
    
    return all_interpolated_paths
                    
#%% 
#% Main 
if __name__ == '__main__':
    
    #define pickle data
    pickle_dir = 'monte_carlo_data/'
    pickle_name = 'random_sim_01'
    
    #load pickle data
    with open(pickle_dir + pickle_name + '.pkl', 'rb') as f:
        data = pkl.load(f)
        

    uas_paths = data['uas_paths'][0:100]
    start_list = data['start_list'][0:100]

    #scale paths
    unit_cell = 8
    scaled_paths = scale_paths(uas_paths, unit_cell)
    
    x_config = 500 * unit_cell
    y_config = 500 * unit_cell
    z_config = 12 * 8
    x_init = 0
    y_init = 0
    z_init = 0


    #evaluate proximity of paths to each other
    #convert path in scale paths as numpy array for easier computation
    
    #search for close time intervals and compute points
    """
    If area is within margin compute the distance between them
    """
    overall_uas_time_vectors = []
    time_margin = 10     
    
    for i, path in enumerate(scaled_paths):
        
        #check modulus 
        if i % 10 == 0:
            print(i)
        
        uas_time_vector = []

        for next_path in scaled_paths:
            next_path_vector = []
            if path == next_path:
                uas_time_vector.append(0)
                continue
            
            for point in path:
                
                for next_point in next_path:
                    
                    if abs(point[3] - next_point[3]) <= time_margin:
                        #compute distance between points
                        distance = np.linalg.norm(np.array(point[0:3]) - np.array(next_point[0:3]))
                        next_path_vector.append(distance)
            
            # check if next_path_vector is empty
            if next_path_vector == []:
                uas_time_vector.append(0)
            else:
                min_distance = min(next_path_vector) 
                uas_time_vector.append(min_distance)
    
        overall_uas_time_vectors.append(uas_time_vector)                            

            
#%% Save data to pickle file
    # pickle_dir = 'monte_carlo_data/'
    # heat_map_name = 'heat_map_01'
    
    # data_analysis.save_to_pickle(pickle_dir+heat_map_name, overall_uas_time_vectors)
    
#%% Plot this data as a heat map
    pickle_dir = 'monte_carlo_data/'
    heat_map_name = 'heat_map_01'
    
    
    # plt.close('all')
    #load pickle data
    with open(pickle_dir + heat_map_name + '.pkl', 'rb') as f:
        overall_uas_time_vectors = pkl.load(f)
        
    #%% 
    #convert overall uas time vectors to numpy array
    overall_matrix = np.array(overall_uas_time_vectors)
    plt.close('all')
    #plot as heat map
    sns.heatmap(overall_matrix, vmin=0, vmax=24)
    
    #compute number of paths are within 10 meters of each other
    locations = np.where(overall_matrix < 5.0)
    
    n_dangerous_manuevers = len(locations[0]) - len(start_list)
    
    print(n_dangerous_manuevers/2)

    #%% 
    
    # plt.close('all')
    # # plt.rcParams['animation.ffmpeg_path'] = '/opt/local/bin/ffmpeg'
    
    # name = str(len(overall_interpolated_path)) + '/' + str(len(start_list)) + ' Paths Planned On First Attempt'
    # animate_uas = PathFinding.AnimateMultiUAS(uas_paths=overall_interpolated_path, 
    #                                 method_name= name)
    # # animate_uas.plot_path(x_bounds=[x_init, x_config], 
    # #                                 y_bounds=[y_init, y_config], 
    # #                                 z_bounds=[z_init, z_config])    
    
    # animate_uas.animate_multi_uas_traffic(x_bounds=[x_init, x_config], 
    #                                 y_bounds=[y_init, y_config], 
    #                                 z_bounds=[z_init, z_config],
    #                                 axis_on=True, save=False)
     
    
    
    

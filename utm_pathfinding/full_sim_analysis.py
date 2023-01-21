from utm_pathfinding import data_utils, PickleDataParser, PathFinding

import pickle as pkl
import matplotlib.pyplot as plt

import pandas as pd
from matplotlib import animation
from matplotlib.animation import FFMpegWriter


"""
Took time difference is  345.85243617499873

"""

                                                    #load pickle file
monte_carlo_data_dir = 'monte_carlo_data/'
pickle_file_name = 'random_sim_01'

#pickle_file_name = '24_hour_sim_0'

unit_per_cell = 8
x_config = 500 * unit_per_cell
y_config = 500 * unit_per_cell
z_config = 12 * 8
x_init = 0
y_init = 0
z_init = 0


with open(monte_carlo_data_dir + pickle_file_name + '.pkl', 'rb') as f:
    test = pkl.load(f)


#%% Assign data
uas_paths = test['uas_paths'][0:1350]
start_list = test['start_list'][0:1350]

t_min = uas_paths[0][0][-1]
t_max = uas_paths[-1][-1][-1]

updated_paths = []  

for i, path in enumerate(uas_paths):
    time_init = path[0][-1]
    time_final = path[-1][-1]
    
    time_init_idx = time_init - t_min
    time_final_idx = time_final - t_min
    
    init_x = -100 
    init_y = -100
    init_z = -100

    final_x = path[-1][0]
    final_y = path[-1][1]
    final_z = path[-1][2]

    new_path = []

    for j in range(time_init_idx+1):
        
        #check if index is out of bounds
        if j >= time_final_idx:
            break

        new_path.append((init_x, init_y, init_z, t_min))

    new_path.extend(path)

    for k in range(time_init_idx+1, time_final_idx+1):

        if k >= time_final_idx:
            break
            
        #new_path.append((path[i][0], path[i][1], path[i][2], t_max))
        
    updated_paths.append(new_path)


high_resolution_paths = []
for path in updated_paths:
    
    full_path = []
    
    for point in path:
        x = point[0] * unit_per_cell
        y = point[1] * unit_per_cell
        # z = point[2] * unit_per_cell
        z = point[2] * unit_per_cell
        t = point[3]
        
        full_path.append((x, y, z, t))
        
    high_resolution_paths.append(full_path)


#%% 
plt.close('all')
# plt.rcParams['animation.ffmpeg_path'] = '/opt/local/bin/ffmpeg'

name = str(len(updated_paths)) + '/' + str(len(start_list)) + ' Paths Planned On First Attempt'
animate_uas = PathFinding.AnimateMultiUAS(uas_paths=high_resolution_paths, 
                                method_name= name)
# animate_uas.plot_path(x_bounds=[x_init, x_config], 
#                                 y_bounds=[y_init, y_config], 
#                                 z_bounds=[z_init, z_config])    

animate_uas.animate_multi_uas_traffic(x_bounds=[x_init, x_config], 
                                y_bounds=[y_init, y_config], 
                                z_bounds=[z_init, z_config],
                                axis_on=True, save=False)


# animate_uas.animate_multi_uas_traffic(x_bounds=[2000, 3000], 
#                                 y_bounds=[2000, 3000], 
#                                 z_bounds=[z_init, z_config],
#                                 axis_on=True, save=False)


# writervideo = FFMpegWriter(fps=120)
# animate_uas.ani.save('hub_sim.mp4', 
#                 writer=writervideo)
        
# plt.show()

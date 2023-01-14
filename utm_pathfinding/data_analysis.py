from utm_pathfinding import data_utils
from utm_pathfinding import PathFinding

import matplotlib.pyplot as plt

monte_carlo_data_dir = 'monte_carlo_data/medium'
pickle_file_name = 'medium_sim_0_uavs_5'
monte_data = data_utils.get_pickle_data(monte_carlo_data_dir + 
    '/' + pickle_file_name)


uas_paths = monte_data['uas_paths']
start_list = monte_data['start_list']

x_config = 2000
y_config = 2000
z_config = 100

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


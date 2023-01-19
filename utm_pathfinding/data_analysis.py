from utm_pathfinding import data_utils
from utm_pathfinding import PathFinding

import matplotlib.pyplot as plt

monte_carlo_data_dir = 'monte_carlo_data/good_base_batch/'
pickle_file_name = 'base_sim'



uas_paths = test['uas_paths']
start_list = test['start_list']

x_config = 100
y_config = 100
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


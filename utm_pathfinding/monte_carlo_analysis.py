import numpy as np


"""
Define metrics 

Plot metrics

Tell Copilot to do that

https://github.com/jn89b/hierarchy_astar_uavs/tree/main/hierarchy_astar_uavs/hierarchy_astar_uavs
""" 

from utm_pathfinding import data_utils, PickleDataParser
import matplotlib.pyplot as plt
import pandas as pd

if __name__ == '__main__':
    
    data_parser = PickleDataParser.DataParser()
    
    #monte_carlo_data_dir = 'monte_carlo_data/good_base_batch/'
    monte_carlo_data_dir = 'monte_carlo_data/good_base_batch/'


    #get all pkl files in monte_carlo_data directory
    pkl_file_dirs = data_parser.get_all_pkl_names(monte_carlo_data_dir)
    
    DICT_KEYS = ["abstract_paths", 
                 "goal_list", 
                 "iter_list", 
                 "success", 
                 "time_list"]

    pkl_list, pkl_filenames = data_parser.return_pkl_files(pkl_file_dirs)
    
    #%% Convert overall dictionaries to dataframes for easier formatting and parsing
    
    keys = list(pkl_list[0].keys())

    n_uav_dictionary = data_parser.return_uav_dict(pkl_list, pkl_filenames)
    
    df_list = PickleDataParser.return_df_list(n_uav_dictionary)
    
    test = pkl_list[0]
    
    uav_range = np.arange(10,110,10)
    
    for i, df in enumerate(df_list):
        df['N_UAVS']= uav_range[i]
        
    df_range = df_list[0:10]
    
    overall_df = pd.concat(df_range)
    
    #%% Plot
    plt.close('all')
    from matplotlib.lines import Line2D
    import seaborn as sns

    def get_mean_std(df, column_name):
        """returns mean and standard deviation from dataframe and column"""
        return df[column_name].mean(), df[column_name].std()
    
    
    def plot_mean_std(df_list, col_name, titlename, xlabel, ylabel, x_range):
        """plot mean and std dev"""
        fig = plt.figure()
        #set fig size

        sns.set_style("darkgrid")
        #sns.set_theme(style="whitegrid", rc={"axes.facecolor": (0, 0, 0, 0), 'axes.linewidth':2})    #sns.set_theme(style="white")
        fontsize = 14
        colors= sns.color_palette("Paired", n_colors=len(df_list))
        #colors.reverse()
        custom_lines = [Line2D([0], [0], color=colors[-1], lw=4),
                Line2D([0], [0], color=colors[3], lw=4), 
                Line2D([0], [0], color=colors[1], lw=4)]
        
        
        for i, df in enumerate(df_list):
            mean, std = get_mean_std(df, col_name)
            plt.errorbar(int(i+1)*10, mean, std/2, linestyle='-', marker='o',
                         color=colors[-1], capsize=1)
 
        
        #set yaxis to 0-100
        # plt.ylim(0.7,1.01)

        plt.legend(custom_lines, ['4D Planning'])
        plt.xticks(x_range)
        plt.xlabel(xlabel, fontsize=fontsize)
        plt.ylabel(ylabel, fontsize=fontsize)
        #plt.title(titlename,fontsize=fontsize)
        plt.tight_layout()
        
        # save_image(titlename, fig)
         
    
    
    plot_mean_std(df_range, "Success", "success_rate", "Number of UAS", "Percent Success",uav_range)
    
    plot_mean_std(df_range, "Total Time","time_complexity" ,"Number of UAS", "Time (seconds)",uav_range)
    
    
 
    #%% Compare solution path
    plt.close('all')
    
    import math
    
    def truncate(number, digits) -> float:
        stepper = 10.0 ** digits
        return math.trunc(stepper * number) / stepper
       
    def compute_total_distance(path):
        """compute total sum of distance travelled from path list"""
        path_array = np.diff(np.array(path), axis=0)
        segment_distance = np.sqrt((path_array ** 2).sum(axis=1))
        return np.sum(segment_distance)    
    
    def compute_actual_euclidean(position, goal):
        print("position", position)
        position = position[0]
        print("goal", goal)
        distance =  (((position[0] - goal[0]) ** 2) + 
                           ((position[1] - goal[1]) ** 2))**(1/2)
                           #((position[2] - goal[2]) ** 2))**(1/2)
        
        return distance
       
    def compute_quality(best, measured):
        """returns the quality of the solution as twice"""
        quality = abs(measured/best)
        
        return quality
    
    def compute_quality_percent(best,measured):
        percent_quality = abs(best/measured)
        
        return percent_quality
    
    def plot_subplots(num_plots,df_list, column_name,x_label, main_title):
        """plots histogram"""
        n_rows = int(num_plots/2)
        n_cols = int(num_plots/2) 
        c = 1
        fig, axes = plt.subplots(n_rows,n_cols,sharey=False)        
        axes = axes.flat 
       
        for i, df in enumerate(df_list[0:4]):
            """FREQUENCY -> """
            sns.histplot(df[column_name], bins='auto', color='steelblue', ax=axes[i], 
                         stat='probability', kde=True)
            #sns.kdeplot(data=df[column_name], ax=axes[i],shade=True)
            axes[i].set_title(str((i+1)*10) + " for " + str(len(df)) + " simulations")
            axes[i].set_xlabel(x_label)
            axes[i].set_ylabel("Percent Frequency")
            
        fig.suptitle(main_title)
        
    def get_mission_info(uav_dict, uav_key):
        """returns mission info from pkl file in dictionary"""
        return uav_dict[uav_key]["Location"]
       
    def get_quality_solution(mission_index_list, pkl_list):
        """find the quality of the solution path length"""
        solution_list = []
        percent_solution = []
        for i in range(len(mission_index_list)):
            mission_dict = pkl_list[mission_index_list[i]]
            all_waypoints = mission_dict["uas_paths"]
            if "start_list" in mission_dict:
                start_list = mission_dict["start_list"]
                goal_list = mission_dict["end_list"]
            else:
                continue
            
            """best waypoints"""
            for i, waypoints in enumerate(all_waypoints):
                if waypoints:
                    twod_coords = [list(wp[:-1]) for wp in waypoints]
                    act_dist = compute_total_distance(twod_coords)
                    best_dist = compute_actual_euclidean(start_list[i], goal_list[i])
                    if compute_quality(best_dist, act_dist) <= 1:
                        solution_list.append(1)
                        percent_solution.append(1)
                    elif math.isnan(compute_quality(best_dist, act_dist)) or math.isinf(compute_quality(best_dist, act_dist)):
                        continue
                    else:
                        solution = compute_quality(best_dist, act_dist)
                        solution_list.append(truncate(solution, 3))
                        
                        percent_sol = compute_quality_percent(best_dist, act_dist)
                        percent_solution.append(percent_sol)
                        
                else:
                    solution_list.append(0)
                    
        quality_sol= [i for i in solution_list if i != 0]
        percent_qual = [i for i in percent_solution if i != 0]
        return quality_sol, percent_qual
    
            
    ## open up nuav dictionary 
    ## key in some number of uavs
    uav_keys = list(n_uav_dictionary.keys())
    quality_sol_list = []
    percent_qual_list = []
    
    ## get list of indexes from index
    for uav in uav_keys[1:]:
        mis_index_list = get_mission_info(n_uav_dictionary, uav)
        qual, percent_qual = get_quality_solution(mis_index_list, pkl_list)
        quality_sol_list.append(qual)
        percent_qual_list.append(percent_qual)
        
#%% Quality of solution
    import statistics    
    plt.close('all')

    def my_mean(x):
        return np.average(x, weights=np.ones_like(x) / x.size)
    
    def get_mean_std(info):
        """return mean and std from list"""
        #info = info[np.logical_not(np.isnan(info))]                
        #mean = statistics.mean(info)
        #std = statistics.stdev(info)
        info = np.array(info, dtype=float)
        mean = np.nanmean(info)
        std = np.nanstd(info)
        print("mean", mean)
        return mean, std
    
    def plot_mean_std_list(some_list, titlename, xlabel, ylabel,offset, n_uavs):
        """plot mean and std dev"""
        fig = plt.figure()
        sns.set_style("darkgrid")
        fontsize = 16
        colors= sns.color_palette("mako", n_colors=len(df_list))
        #colors.reverse()
        custom_lines = [Line2D([0], [0], color=colors[4], lw=4),
                Line2D([0], [0], color=colors[8], lw=4)]
        
        mean_list = []
        std_list = []
        for i, vals in enumerate(some_list):
            #print(vals)
            mean, std = get_mean_std(vals)
            mean_list.append(mean)
            std_list.append(std)
            plt.errorbar(int(i+offset)*10, mean, std/2, linestyle='None', marker='o',
                         color=colors[i], capsize=3)
            
        # for i, vals in enumerate(some_list2):
        #     #print(vals)
        #     mean, std = get_mean_std(vals)
        #     plt.errorbar(int(i+offset)*10, mean, std/2, linestyle='None', marker='o',
        #                  color=colors[i], capsize=3)
            
        x_range = n_uavs
        #x_range = np.arange(offset*10,(len(some_list)*(10*offset)), 10)
        print("x_range", x_range)
        plt.xticks(x_range[0:-1])
        plt.xlabel(xlabel, fontsize=fontsize)
        plt.ylabel(ylabel, fontsize=fontsize)
        plt.title(titlename,fontsize=fontsize)
        plt.tight_layout()
        # save_image(titlename, fig)
        
        return mean_list, std_list
    
    #plot_mean_std_list(quality_sol_list, "Quality of Solution Unsorted", "Number of UAS", "Distance Factor",1, n_uavs)
    unsort_mean, unsort_std = plot_mean_std_list(percent_qual_list, "Percent Quality Unsorted", 
                                                 "Number of UAS", "Distance Factor",1, uav_range)

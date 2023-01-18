import pickle
import glob 
import os
import pandas as pd
from collections import defaultdict


class DataParser():

    def __init__(self) -> None:
        pass


    def load_pkl_file(self,pkl_file_name):
        """opens pkl file name"""
        with open(pkl_file_name, 'rb') as f:
            return pickle.load(f)
        
    def get_all_pkl_names(self,path_directory):
        """get all csv files based on some path directory"""
        return glob.glob(path_directory + "/*.pkl")
    
    def get_filename(self,filename_dir):
        """return base file names of csv and removes the directory path"""
        return os.path.basename(os.path.normpath(filename_dir))
    
    # https://stackoverflow.com/questions/4529815/saving-an-object-data-persistence
    def save_object(self,obj, filename):
        with open('replay_fails/'+filename, 'wb') as outp:  # Overwrites any existing file.
            pickle.dump(obj, outp, pickle.HIGHEST_PROTOCOL)

    def init_uavs_dictionary(self, start, stop, steps):
        """returns a dictionary of keys for number of uavs """
        n_uavs_keys = [i for i in range(start,stop+steps, steps)]
    
        new_dict = defaultdict(list)
        for n_uav in n_uavs_keys:
            new_dict[n_uav] = None
        
        return new_dict
        
    def init_nuav_df(self):
        """initializes an overall dataframe"""
        df = pd.DataFrame(data = None, 
                        columns = ['SimNum','Heuristic','Bubble','Total Time',
                                    'Iter Time', 'Success'],
                        )
        
        return df

    def insert_df_to_dict(self, uav_dict):
        """inserts a dataframe into the dictionary"""
        for n_uav in uav_dict:
            df = self.init_nuav_df()
            uav_dict[n_uav] = df
        
    def insert_overall_dict(self):
        """adds an overall dictionary inside key of input dictionary"""
        overall_keys = ['SimNum','Heuristic','Bubble','Total Time',
                'Iter Time', 'Success']
        
        new_dict = defaultdict(list)
        for n_uav in overall_keys:
            new_dict[n_uav] = []
        
        return new_dict

    def return_uav_dict(self, pkl_list, pkl_filenames):
        keys = list(pkl_list[0].keys())
        
        n_uav_dictionary = self.init_uavs_dictionary(10, 100, 10)
        for key in n_uav_dictionary:
            n_uav_dictionary[key] = self.insert_overall_dict()
        
        for i,(pkl,file_name) in enumerate(zip(pkl_list, pkl_filenames)):
            # num_uavs = len(pkl["overall_paths"])
            num_uavs = len(pkl["start_list"])
            n_uav_dictionary[num_uavs]["Bubble"] = 4
            n_uav_dictionary[num_uavs]["Heuristic"] = 10
            n_uav_dictionary[num_uavs]["SimNum"].append(file_name)
            success_rate = len(pkl["uas_paths"])/num_uavs 
            n_uav_dictionary[num_uavs]["Success"].append(success_rate)
            n_uav_dictionary[num_uavs]["Total Time"].append(pkl["time"])
            n_uav_dictionary[num_uavs]["Iter Time"].append(pkl["iterations"])
            n_uav_dictionary[num_uavs]["Location"].append(i) ## refers to the list inside pkl_list
            
        return n_uav_dictionary


    def return_pkl_dirs(self, folder_dir_name, index_val):
        """return pkl files from directory"""
        pkl_dirs = []
        for i in range(1,index_val):
            folder_name = folder_dir_name+str(i)
            path = os.getcwd() + "/"+ folder_name
            pkl_file_dirnames = self.get_all_pkl_names(path)
            pkl_dirs.extend(pkl_file_dirnames)
            
        return pkl_dirs

    def return_pkl_files(self, pkl_dirs):
        """return pkl filenames"""
        pkl_list = []
        pkl_filenames = []
        for pkl in pkl_dirs:
            
            pkl_list.append(self.load_pkl_file(pkl))
            pkl_filenames.append(self.get_filename(pkl))

        return pkl_list, pkl_filenames


def save_image(image_name, fig):
    """saves image"""
    image_format = 'svg' # e.g .png, .svg, etc.
    # image_name = 'myimage.svfg'
    
    fig.savefig('images/'+image_name+'.svg', format=image_format, dpi=1200)
    


def return_df_list(uav_dictionary):
    """return list of dataframes for n uavs"""
    df_list = []
    
    for key,overall_dict in uav_dictionary.items():
        overall_df = pd.DataFrame.from_dict(overall_dict)
        df_list.append(overall_df)

    return df_list
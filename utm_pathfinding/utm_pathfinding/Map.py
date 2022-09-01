import numpy as np
import math as m
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import cnames
from matplotlib import animation
import seaborn as sns
#https://stackoverflow.com/questions/36013063/what-is-the-purpose-of-meshgrid-in-python-numpy

class Region(object):
    def __init__(self, region_coord:tuple, limits:list) -> None:
        self.region_coordinate = region_coord
        self.region_sides = {'left_side': None,
                             'right_side': None,
                             'bottom_side': None,
                             'top_side': None}
        
        #this will be appended to the same side of the region
        self.neighbor_sides = {'left_side': None,
                             'right_side': None,
                             'bottom_side': None,
                             'top_side': None}

        self.opposite_direction = {
                    'left_side': 'right_side',
                    'right_side':'left_side',
                    'bottom_side': 'top_side',
                    'top_side': 'bottom_side'}

        self.limits = limits

    def get_opposite_side(self, entrance_key: str) -> str:
        """return the index of the opposite direction"""
        return self.opposite_direction[entrance_key]
    
    def set_neighbor_side(self, adjacent_key:str, neighbor_coords:list) -> None:
        self.neighbor_sides[adjacent_key] = neighbor_coords
   
class AbstractGraph(object):
    def __init__(self, map_area:object) -> None:
        """map and graph data"""
        self.map = map_area
        self.graph = set()
    
    def build_graph(self) -> None:
        """build graph given map area"""
        self.build_corridors()
        
    def build_corridors(self) -> None:
        """
        find all sides or entrances, call out map region dictionary
        in each region dictionary pull out the neighbors of that side
        create abstract nodes 
        """
        for reg_key, region in self.map.regions.items():
             
            pass
    

class Map(object):
    """
    Config map overall size 
    """
    def __init__(self, x_max:float, y_max:float, z_max:float) -> None:
        self.x_array = np.arange(0, x_max)     
        self.y_array = np.arange(0, y_max)
        self.z_array = np.arange(25, z_max)
        
        self.meshgrid = self.generate_grid()
        self.regions = {}
        
        self.grid_space = 1
        self.offset_val = 1
        
    def generate_grid(self) -> list:
        return np.meshgrid(self.x_array, self.y_array, self.z_array, 
                           indexing='xy')
        
    def unravel_meshgrid(self) -> np.ndarray:
        """unravel meshgrid and retruns array of [3, xlen*ylen*zlen] of matrices
        """
        return np.vstack(map(np.ravel, self.meshgrid))
    
    
    def __get_cluster_bounds(self,index, step_size) -> list:
        """get min and max bounds of the cluster"""
        #print(index, index+step_size)
        return [index, index+step_size-self.offset_val] 
    
    def __create_region_sides(self, start_val:float, end_val:float, 
                              const_val:float, z_steps:list, 
                              lat_or_long= "lat") -> list:
        """"return side of square"""
        side_list = []    
        for z in z_steps:            
            for i in range(start_val, end_val+self.offset_val):
                
                # if lateral then left or right side, so y will change                    
                if lat_or_long == "lat":
                    #pruning off outer edges
                    if const_val <= self.x_array[0] or const_val >= self.x_array[-1]:
                        continue
                    side_list.append((const_val,i,z))
                
                else:
                    #pruning off outer edges
                    if const_val <= self.y_array[0] or const_val >= self.y_array[-1]:
                        continue
                    side_list.append((i,const_val,z))
            
        return side_list
    
    def find_which_region(self, coordinates:tuple) -> tuple:
        """find which region the coordinate belongs to"""
        for reg_coord, region in self.regions.items():
            x_bounds, y_bounds = region.limits
       
            if coordinates[0] in range(x_bounds[0], x_bounds[1]+self.offset_val) and \
                (coordinates[1] in range(y_bounds[0], y_bounds[1]+self.offset_val)):
                
                return region.region_coordinate
                 
    def break_into_square_regions(self, num_regions:int, z_step:int) -> None:
        """
        break map into regions based on number of regions specified
        need to refactor the setup of this system 
        """
        
        n_row = m.sqrt(num_regions)
        region_length = round(len(self.x_array)/n_row)
        print("region length: ",region_length)
        z_steps = list(np.arange(self.z_array[0],
                                 self.z_array[-1]+self.offset_val, 
                                 z_step))
         
        for i in range(0, len(self.x_array), region_length):
            for j in range(0, len(self.y_array), region_length):
                        
                x_min_max = self.__get_cluster_bounds(i,region_length) #[x_min, x_max]
                y_min_max = self.__get_cluster_bounds(j, region_length) #[y_min, y_max]
                
                cluster_coords = (i//region_length, j//region_length)
                region_name = str(cluster_coords)
                
                bottom_side = self.__create_region_sides(
                    x_min_max[0], x_min_max[1], y_min_max[0], z_steps,"lon")
                top_side = self.__create_region_sides(
                    x_min_max[0], x_min_max[1], y_min_max[1],z_steps, "lon")
                
                right_side = self.__create_region_sides(
                    y_min_max[0], y_min_max[1], x_min_max[1], z_steps)
                left_side = self.__create_region_sides(
                    y_min_max[0], y_min_max[1], x_min_max[0],z_steps)
                  
                limits = (x_min_max, y_min_max) 
                
                region = Region(cluster_coords, limits)
                region.region_sides['left_side'] = left_side
                region.region_sides['right_side'] = right_side
                region.region_sides['top_side'] = top_side
                region.region_sides['bottom_side'] = bottom_side
                
                self.regions[region_name] = region

                print("i and j are  " + str(i//region_length) + " and " + str(j//region_length))
                # print("region name is " + region_name)
                print("limits are " + str(limits))
                print("\n")
                
        self.region_bound_x = (0,cluster_coords[0])
        self.region_bound_y = (0,cluster_coords[1])
                
    def __is_out_bounds(self, region_coord:list) -> bool:
        if (region_coord[0] > (self.region_bound_x[1]) or 
            region_coord[0] < 0 or 
            region_coord[1] > (self.region_bound_y[1]) or 
            region_coord[1] < 0 ):
            return True
        
        return False    
    
    def find_neighbor_regions(self) -> None:
        """Find neighbors of each region and connect their sides
        -from current region call out which side the neighbor is at ,
        -from this side access the neighbor region and get the opposite side
        -use this opposite side to access the location
        -link these two together  
        """
        ss = 1
        
        move_dict = {'top_side': [0,ss],
                     'bottom_side': [0,-ss],
                     'left_side': [-ss,0],
                     'right_side': [ss,0]}
        
        for region_key, region in self.regions.items():
            region_coord = region.region_coordinate
            
            for move_key, move in move_dict.items():
                neighbor_pos = (region_coord[0]+move[0], 
                                region_coord[1]+move[1])
                
                if self.__is_out_bounds(neighbor_pos) == True:
                    continue
                           
                neighbor_reg = self.regions[str(neighbor_pos)]
                neighbor_side = neighbor_reg.get_opposite_side(move_key)
                neighbor_coords = neighbor_reg.region_sides[neighbor_side]
    
                region.neighbor_sides[neighbor_side] = neighbor_reg
                region.set_neighbor_side(neighbor_side, neighbor_coords)
                        
                
    def main(self,num_regions:int, z_step:int):
        self.break_into_square_regions(num_regions, z_step)
        self.find_neighbor_regions()
                
    def plot_regions(self, threed=False) -> None:
        """plot regions, threed plot is defaulted to false"""
        if not self.regions:
            print("no regions currently")
        else:
            self.fig = plt.figure(figsize=(8, 8))
            self.ax = self.fig.add_subplot(111, projection="3d")
            self.ax.set_xlim3d(self.x_array[0], self.x_array[-1])
            self.ax.set_ylim3d(self.y_array[0], self.y_array[-1])
            self.ax.set_zlim3d(self.z_array[0], self.z_array[-1])
            self.title = self.ax.set_title("Regions",fontsize=18)
            
            color_list = sns.color_palette("hls", len(self.regions))
            
            if threed == False:
                for i, (k, v) in enumerate(self.regions.items()):
                    for r_k, r_v in v.region_sides.items():
                        if r_v:
                            x,y,z = zip(*r_v)
                            self.ax.plot(x,y,color=color_list[i])
            else:
                print("plotting 3d")
                for i, (k, v) in enumerate(self.regions.items()):
                    for r_k, r_v in v.region_sides.items():
                        if r_v:
                            x,y,z = zip(*r_v)
                            self.ax.scatter(x,y,z,color=color_list[i])
                    
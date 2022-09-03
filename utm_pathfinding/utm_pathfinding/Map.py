from re import RegexFlag
import numpy as np
import math as m
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import cnames
from matplotlib import animation
import seaborn as sns

import itertools
from itertools import combinations, permutations, product

#https://stackoverflow.com/questions/36013063/what-is-the-purpose-of-meshgrid-in-python-numpy

class AbstractNode():
    def __init__(self, global_location, region_coord):
        self.location = global_location
        self.position = global_location
        self.region_coord = region_coord
        self.level = None
        self.node_type = None
    
    def set_cost(self,cost:float) -> None:
        """this is the edge cost"""
        self.cost = cost
        
    def set_node_type(self,node_type:str) -> None:
        """set type of node, inter connects regions, intra connects within"""
        if node_type == "INTER":
            self.node_type = "INTER"
        if node_type == "INTRA":
            self.node_type = "INTRA"
                            
    def set_node_level(self,node_level:int) -> None:
        self.level = node_level
                            
    def __get_comparison(self) -> bool:
        return (tuple(self.location), self.node_type)
            
    def __eq__(self, other) -> bool:
        return (self.location, self.node_type) == (other.location, other.node_type)
        
    def __ne__(self, other) -> bool:
        return (not self.__eq__(other))
    
    def __hash__(self) -> dict:
        return hash(self.__get_comparison())

class Region(object):
    def __init__(self, region_coord:tuple, limits:list,
                 region_level:int) -> None:
        
        self.region_coordinate = region_coord
        self.region_sides = {'left_side': None,
                             'right_side': None,
                             'bottom_side': None,
                             'top_side': None}

        self.region_level = region_level
        
        #this will be appended to the same side of the region
        self.neighbor_sides = {'left_side': None,
                             'right_side': None,
                             'bottom_side': None,
                             'top_side': None}
        
        #this appends to the coordinate of the neighbor 
        self.neighbor_coords = {'left_side': None,
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
   
    def set_neighbor_coords(self, adjacent_key:str, neighbor_reg_coord:list) -> None:
        """insert the region coordinate of neighbor from which side"""
        self.neighbor_coords[adjacent_key] = neighbor_reg_coord


class AbstractGraph(object):
    def __init__(self, map_area:object) -> None:
        """map and graph data"""
        self.map = map_area
        self.graph_levels = {}
        self.graph = {}
        self.weight = 1
        self.intra_type = "INTRA"
        self.inter_type = "INTER"
        
    def build_graph(self) -> None:
        """build graph given map area"""
        self.build_corridors()
        
    def build_corridors(self, level:int) -> None:
        """
        find all sides or entrances, call out map region dictionary
        in each region dictionary pull out the neighbors of that side
        check if obstacle in this
        if not:
            create abstract nodes
            connect nodes together 
        """
        regions = self.map.level_regions[str(level)]
        
        #set graph level 
        self.graph_levels[str(level)] = {}

        for reg_key, region in regions.items():
            for (side_key,sides), (nei_key,nei_sides) in \
                zip(region.region_sides.items(), region.neighbor_sides.items()):

                    if sides == None or nei_sides == None:
                        continue
                    
                    for (side_coord,nei_coord) in zip(sides,nei_sides):
                        reg_node = AbstractNode(side_coord, region.region_coordinate)
                        
                        nei_reg_coord = self.map.find_which_region(nei_coord, level)
                        nei_node = AbstractNode(nei_coord, nei_reg_coord)
                        
                        self.__add_node(reg_node, level)
                        self.__add_node(nei_node, level)
                        self.__add_edge(
                            reg_node, nei_node, level, self.weight, self.inter_type)

    def add_level_to_graph(self, level:int) -> None:
        """"""
        for reg_key, region in self.map.regions.items():
            pass

    def build_airways(self, level:int):
        """
        within each region look at all sides and connect them to each combination
        - loop through reach regions 
        - create a list of inner_connections
            - loop through each side of region 
            - append side coordinates to inner_connections 
            - get all combinations for same side 
        
        number of connectsion = entrace_con + (m-1)(n) 
        where m is the number of nodes in the current location and n is the 
        number of nodes in the adjacent side
        """
        regions_dict = self.map.level_regions[str(level)]
        
        for idx, (reg_key, region) in enumerate(regions_dict.items()):
                        
            region_coord = region.region_coordinate
            inner_connections = []
            inner_sets = []
            
            for edge_side, location in region.region_sides.items():
                
                if not location:
                    print("no location", edge_side)
                    continue
                
                inner_connections.append(location)
                
                #get all permuations for nodes on the same side
                same_side_combos = list(combinations(location, 2))
                for same_side in same_side_combos:
                    inner_sets.append(same_side)
            
            #add all the intra nodes between the inner sides so left and right, etc
            for r in itertools.chain(product(inner_connections[0], inner_connections[1])
                                     , product(inner_connections[1], inner_connections[0])):
                inner_sets.append(r)
                
            intra_connections_list = inner_sets
            
            #this is a test to reduce the grid
            if idx == 4:
                return intra_connections_list, inner_sets

            #add all the intra nodes between the adjacent nodes need to do Astar to find distance
            for intra_connections in intra_connections_list:
                intra_node1 = AbstractNode(intra_connections[0], region_coord)
                intra_node1.set_node_level(level)
                
                intra_node2 = AbstractNode(intra_connections[1], region_coord)
                intra_node2.set_node_level(level)
                
                distance = self.compute_actual_euclidean(intra_node1.location, intra_node2.location)
                #distance = self.__search_for_distance(intra_node1, intra_node2, config_space, config_bounds)
                self.__add_edge(intra_node1, intra_node2, level, distance, self.intra_type)
    
    def compute_actual_euclidean(self, position:list, goal:list) -> float:
        distance =  (((position[0] - goal[0]) ** 2) + 
                           ((position[1] - goal[1]) ** 2) +
                           ((position[2] - goal[2]) ** 2))**(1/2)
        
        return distance
            
    def insert_temp_nodes(self, location:tuple, height_bound:float, level:int) -> None:
        """insert temp nodes into the graph takes in the AbstractNode, level, 
        and hash key name to be inserted 
        - use methods to determine cluster
        - connect to border
        - set level
        """
        region_coords = self.map.find_which_region(location, level)
        print("region coordinates of location", location, region_coords )
        temp_node = AbstractNode(location , region_coords)        
        self.connect_to_border(temp_node, str(location), height_bound, level)
        
    def connect_to_border(self,node:object, key_name:str, height_limit:float, level:int) -> None:
        """connect borders to the map, I should have this in the graph class but define the key value
        so set key to start and goal to make it temporary"""
        offset_limit = height_limit #this is dumb should have this parameterized
        height_bounds = [node.location[2]-offset_limit, node.location[2]+offset_limit]
        print("height bounds are", height_bounds)
        
        regions = self.map.level_regions[str(level)]
        mapped_entrances_start = regions[str(node.region_coord)].region_sides
        
        for entrance, entrance_list in mapped_entrances_start.items():
            for entrance_loc in entrance_list:
                """do a check for ranges don't want to connect to areas at higher places"""
                if entrance_loc[2] >= height_bounds[0] and entrance_loc[2] <= height_bounds[1]:
                    intra_node2 = AbstractNode(entrance_loc, node.region_coord)
                    intra_node2.set_node_level(level)
                    
                    distance = self.compute_actual_euclidean(node.location, intra_node2.location)
                    self.__add_temp_edges(
                        node, intra_node2, level, distance, self.intra_type, key_name)
                else:
                    continue
                     
    def __add_node(self, node:object, level:int) -> None:
        """add node to search graph"""
        if node.level == None:
            node.set_node_level(level)
        
        graph = self.graph_levels[str(level)]
                
        if str(node.location) in graph:
            graph[str(node.location)].add(node)
        else:
            graph[str(node.location)] = set([node])
                
                  
    def __add_temp_edges(self, temp_node:AbstractNode, node2:AbstractNode, level:int, 
                         weight:float,  node_type:str, key_name:str) -> None:
        """adds the temporary node and connects it with other nodes in the hashtable"""
        temp_node.set_cost(weight)
        node2.set_cost(weight)
        
        temp_node.set_node_type(node_type)
        node2.set_node_type(node_type)
        
        graph = self.graph_levels[str(level)]

        #check if intra node connections exists
        if not str(key_name) in graph:
            graph[str(key_name)] = set([temp_node])
            
        if not str(node2.location) in graph:
            graph[str(node2.location)] = set([node2])
        
        graph[str(key_name)].add(node2)
        graph[str(node2.location)].add(temp_node)
            
    def __add_edge(self, node1:object, node2:object,level:int, 
                   weight:int, node_type:str) -> None:
        """makes edge between two nodes inside the graph at specified level"""
        node1.set_cost(weight)
        node2.set_cost(weight)
        
        node1.set_node_type(node_type)
        node2.set_node_type(node_type)
        
        graph = self.graph_levels[str(level)]
        
        #check if intra node connections exists
        if not str(node1.location) in graph:
            graph[str(node1.location)] = set([node1])
            
        if not str(node2.location) in graph:
            graph[str(node2.location)] = set([node2])
        
        graph[str(node1.location)].add(node2)
        graph[str(node2.location)].add(node1)


class Map(object):
    """
    Config map overall size 
    """
    def __init__(self, x_max:float, y_max:float, z_max:float, 
                 grid_space:int) -> None:
        self.x_array = np.arange(0, x_max)     
        self.y_array = np.arange(0, y_max)
        self.z_array = np.arange(0, z_max)
        
        self.meshgrid = self.generate_grid()
        
        self.level_regions = {}
        self.regions = {}
        self.level_region_bounds = {}
        
        self.grid_space = int(grid_space)
        self.offset_val = 1
            
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
            for i in range(start_val, end_val+self.offset_val, self.grid_space):
                
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
    
    def __is_out_bounds(self, region_coord:list, level:int) -> bool:
        """checks if out of bounds of the grid coordinates of each region
        so if we have 3 x 3 we if beyond -4 or 4 
        """
        bounds = self.level_region_bounds[str(level)]
        region_bound_x = bounds[0]
        region_bound_y = bounds[1]
        
        if (region_coord[0] > (region_bound_x[1]) or 
            region_coord[0] < 0 or 
            region_coord[1] > (region_bound_y[1]) or 
            region_coord[1] < 0 ):
            return True
        
        return False    
    
    def generate_grid(self) -> list:
        return np.meshgrid(self.x_array, self.y_array, self.z_array, 
                           indexing='xy')
        
    def unravel_meshgrid(self) -> np.ndarray:
        """unravel meshgrid and retruns array of [3, xlen*ylen*zlen] of matrices
        """
        return np.vstack(map(np.ravel, self.meshgrid))

    def find_which_region(self, coordinates:tuple, level:int) -> tuple:
        """find which region the coordinate belongs to"""
        
        regions = self.level_regions[str(level)]
        
        for reg_coord, region in regions.items():
            x_bounds, y_bounds = region.limits
       
            if coordinates[0] in range(x_bounds[0], x_bounds[1]+self.offset_val) and \
                (coordinates[1] in range(y_bounds[0], y_bounds[1]+self.offset_val)):
                
                return region.region_coordinate
                 
    def break_into_square_regions(self, num_regions:int, z_step:int, level:int) -> None:
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
        print("z height: ",z_steps)

        regions_dict = {}
        
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
                
                region = Region(cluster_coords, limits, level)
                region.region_sides['left_side'] = left_side
                region.region_sides['right_side'] = right_side
                region.region_sides['top_side'] = top_side
                region.region_sides['bottom_side'] = bottom_side
                
                """remove this later"""                                
                self.regions[region_name] = region
                
                regions_dict[region_name] = region

                print("i and j are  " + str(i//region_length) + " and " + str(j//region_length))
                # print("region name is " + region_name)
                print("limits are " + str(limits))
                print("\n")
                
        self.region_bound_x = (0,cluster_coords[0])
        self.region_bound_y = (0,cluster_coords[1])
        
        self.level_region_bounds[str(level)] = [(0,cluster_coords[0]), (0,cluster_coords[1])]
        self.set_regions_level(level, regions_dict)

    def set_regions_level(self, level:int, regions_dict:dict) -> None:
        self.level_regions[str(level)] = regions_dict 
                
    def find_neighbor_regions(self, level) -> None:
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
        
        regions = self.level_regions[str(level)]
        
        for region_key, region in regions.items():
            region_coord = region.region_coordinate
            
            for move_key, move in move_dict.items():
                neighbor_pos = (region_coord[0]+move[0], 
                                region_coord[1]+move[1])
                
                """should parameterize this based on the map config"""
                if self.__is_out_bounds(neighbor_pos, level) == True:
                    continue
                           
                neighbor_reg = regions[str(neighbor_pos)]
                neighbor_side = neighbor_reg.get_opposite_side(move_key)
                neighbor_coords = neighbor_reg.region_sides[neighbor_side]
    
                # region.neighbor_sides[neighbor_side] = neighbor_reg
                region.set_neighbor_side(move_key, neighbor_coords)
                region.neighbor_coords[move_key] = neighbor_pos 
                
    def main(self,num_regions:int, z_step:int):
        self.break_into_square_regions(num_regions, z_step)
        self.find_neighbor_regions()
                
    def plot_regions(self, level:int, threed=False) -> None:
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

            regions = self.level_regions[str(level)]
            color_list = sns.color_palette("hls", len(regions))
            
            if threed == False:
                for i, (k, v) in enumerate(regions.items()):
                    for r_k, r_v in v.region_sides.items():
                        if r_v:
                            x,y,z = zip(*r_v)
                            self.ax.plot(x,y,color=color_list[i])
            else:
                print("plotting 3d")
                for i, (k, v) in enumerate(regions.items()):
                    for r_k, r_v in v.region_sides.items():
                        if r_v:
                            x,y,z = zip(*r_v)
                            self.ax.scatter(x,y,z,color=color_list[i])
                    
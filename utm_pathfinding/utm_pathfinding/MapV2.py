import numpy as np 
import math as m


class Map():
    """
    config map overall size
    """
    def __init__(self, map_params:dict) -> None:
        
        self.map_params = map_params
        self.x_array = np.arange(0, map_params["x_config"])
        self.y_array = np.arange(0, map_params["y_config"])
        self.z_array = np.arange(0, map_params["z_config"])

        self.regions_dict = {}

    def break_into_regions(self) -> None:
        """
        break map into regions based on number of regions specified
        Grid visualization is as follows say we have 3 rows and 4 columns 

                 COL
            R    A1 B1 C1 D1
            O    A2 B2 C2 D2
            W    A3 B3 C3 D3 

        """
        number_columns = self.map_params["number_cols"]
        number_rows = self.map_params["number_rows"]
        level = self.map_params["level"]

        inter_cols = self.map_params["inter_cols"]
        inter_rows = self.map_params["inter_rows"]
        inter_level = self.map_params["inter_level"]

        x_step_size = int(m.ceil(len(self.x_array)/number_columns)) 
        y_step_size = int(m.ceil(len(self.y_array)/number_rows))
        z_step_size = int(m.ceil(len(self.z_array)/level))

        #calculate intermediate step size
        x_inter_step_size = int(m.ceil(x_step_size/inter_cols))
        y_inter_step_size = int(m.ceil(y_step_size/inter_rows))
        z_inter_step_size = int(m.ceil(z_step_size/inter_level))

        #list of letters from A to Z based on number of columns
        letters = [chr(i) for i in range(65, 65+number_columns)]
        
        #list of lat_numbers from 1 to number of rows
        lat_numbers = [str(i) for i in range(1, number_rows+1)]
        lat_numbers.reverse()

        #list of alt_numbers from 1 to number of z_steps
        alt_numbers = [str(i) for i in range(1, level+1)]
        alt_numbers.reverse()

        #create 2d adjacency matrix
        self.adjacency_matrix = []

 
        #break into regions based on number of columns and rows
        for z in range(0,  len(self.z_array), z_step_size):
            for i in range(0, len(self.x_array), x_step_size):
                for j in range(0, len(self.y_array), y_step_size):
                    
                    letter = letters[int(i/x_step_size)]
                    lat_number = lat_numbers[int(j/y_step_size)]
                    alt_number = alt_numbers[int(z/z_step_size)]

                    #get region points
                    region_points = self.__get_region_points(i, j, z, 
                                                            x_step_size, 
                                                            y_step_size,
                                                            z_step_size,                                                               
                                                            x_inter_step_size,
                                                            y_inter_step_size,
                                                            z_inter_step_size)

                    coordinate_grid = letter+lat_number+alt_number
                    print(letter+lat_number+alt_number)
                    print(i,j,z)
                    print(region_points)
                    print("\n")
                    self.regions_dict[coordinate_grid] = region_points

                    #store letter and lat number in adjacency matrix
                    self.adjacency_matrix.append([letter+lat_number])

    def __get_region_points(self, 
        min_region_x:int, min_region_y:int, min_region_z:int,
        x_step_size:int, y_step_size:int, z_step_size:int,
        x_inter_step_size:int, y_inter_step_size:int, z_inter_step_size:int) -> list:
        """
        return coordinate points for region
        """
        
        region_points = []

                
        for z in range(min_region_z, min_region_z+z_step_size+1, z_inter_step_size-1):
            print(z)
            #check if z is out of bounds
            # if z <= self.z_array[0] or z >= self.z_array[-1]:
            #     continue

            for x in range(min_region_x, min_region_x+x_step_size+1, x_inter_step_size):
                #check if x is out of bounds
                if x <= self.x_array[0] or x >= self.x_array[-1]:
                    continue 
                
                for y in range(min_region_y, min_region_y+y_step_size+1, y_inter_step_size):
                    #check if y is out of bounds
                    if y <= self.y_array[0] or y >= self.y_array[-1]:
                        continue

                    region_points.append([x,y,z])


        return region_points


if __name__ == "__main__":
    
    map_params = {
        "x_config": 10,
        "y_config": 10,
        "z_config": 20,
        "number_rows": 2, #related to y 
        "number_cols": 2, #related to x 
        "level": 1,
        "inter_rows": 3,
        "inter_cols": 3,
        "inter_level": 1
    }
    
    map_area = Map(map_params)
    map_area.break_into_regions()

    regions = map_area.regions_dict    

    #adjacent matrix
    adjacent_matrix = map_area.adjacency_matrix 
    #convert to numpy array
    adjacent_matrix = np.array(adjacent_matrix)
    #reshape to number_rows x number_cols
    adjacent_matrix = adjacent_matrix.reshape(map_params["number_rows"], 
                    map_params["number_cols"])

        
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    plt.close('all')

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    #set colors for each region with seaborn
    import seaborn as sns
    sns.set()
    sns.set_palette("husl", 8)

    for region in regions:
        points = regions[region]
        x = [point[0] for point in points]
        y = [point[1] for point in points]
        z = [point[2] for point in points]
        ax.scatter(x, y, z, marker='o', label=region)

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.legend()
    plt.show()



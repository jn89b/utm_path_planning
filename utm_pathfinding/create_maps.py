import pickle 

from utm_pathfinding import Map


"""
This script creates a map object and saves it to a pickle file.

"""

def create_map_config(x_size:int, 
    y_size:int, z_size:int, resolution:int, 
    map_name:str, region:int):
    """
    Creates a map object and saves it to a pickle file.
    """

    z_step = 20
    max_level = 1

    # Create a map object
    map_area =  Map.Map(x_size, y_size, z_size, resolution)
    map_area.break_into_square_regions(region, z_step, max_level)
    map_area.find_neighbor_regions(max_level)

    graph = create_graph(map_area, max_level)

    map_info = {
        'map_area': map_area,
        'graph': graph
    }

    # Save the map object to a pickle file
    with open(map_name + '.pkl', 'wb') as output:
        pickle.dump(map_info, output, pickle.HIGHEST_PROTOCOL)

        print("Map saved to " + map_name + ".pkl")

    return 

def create_graph(map_area, level:int):
    """
    Creates a graph object and saves it to a pickle file.
    """

    # Create a graph object
    graph = Map.AbstractGraph(map_area) 
    graph.build_corridors(level)
    graph.build_airways(level)

    return graph

# Create a map object
if __name__ == "__main__":

        
    x_size = [100, 1000, 1500, 2000]
    y_size = [100, 1000, 1500, 2000]
    z_size = [100, 100, 100, 100]
    resolution = [10, 50, 100, 150]
    map_name = ['base', 'small', 'medium', 'large']
    regions = [4, 100, 150, 200]

    idx = 3
    create_map_config(
        x_size[idx], 
        y_size[idx], 
        z_size[idx], 
        resolution[idx],
        map_name[idx], 
        regions[idx])

    # for x,y,z,res,name,region in zip(x_size, y_size, z_size, resolution, map_name, regions):
    #     create_map_config(x, y, z, res, name, region)
        







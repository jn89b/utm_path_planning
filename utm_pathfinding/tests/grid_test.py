# -*- coding: utf-8 -*-
"""
Created on Mon Aug 22 13:39:48 2022

@author: jnguy
"""

"""

Map -> Grid -> Graph 

Allow adjustment for reconfigurable clusters 

Lay out x,y dimensions then specify height and stack up

"""


#% Import stuff here
import numpy as np
import math as m
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D 


#% Classes 
class Map():
    def __init__():
        pass


def get_coordinate(x_coord:float, y_coord:float, z_coord:float,
                   x_mesh:np.ndarray, y_mesh:np.ndarray, 
                   z_mesh:np.ndarray) -> tuple:
    """get coordinates"""
    
    return (x_mesh[x_coord,y_coord, z_coord], 
            y_mesh[x_coord,y_coord, z_coord], 
            z_mesh[x_coord,y_coord, z_coord])
    
def unravel_meshgrid(mesh_map:np.ndarray) ->:
    """unravel meshgrid"""
    

#% Main 
if __name__ == '__main__':
    
    plt.close('all')
    
    x = np.arange(0, 25)     
    # numpy.linspace creates an array of
    # 9 linearly placed elements between
    # -4 and 4, both inclusive
    y = np.arange(0, 25)
    
    z = np.arange(0, 25)
    
    # The meshgrid function returns
    # two 2-dimensional arrays
    x_1, y_1, z_1 = np.meshgrid(x, y, z, indexing='xy')
    map_area =  np.meshgrid(x, y, z, indexing='xy')
    
    # ax.scatter(xv, yv, zv, c='g', marker='^')


    # assert np.all(x_1[:,0,0] == x)
    # assert np.all(y_1[0,:,0] == y)
    # assert np.all(z_1[0,0,:] == z)
    
    # coords = []
    # for a, b, c in  zip(x_1, y_1, z_1):
    #     for a1, b1, c1 in zip(a, b, c):
    #         for a2, b2, c2 in zip(a1, b1, c1):
    #             coords.append((a2, b2, c2,))
    
    
    some_coordinate = get_coordinate(2,3,4, x_1, y_1, z_1)
    print(some_coordinate)

    #unravel position            
    positions = np.vstack(map(np.ravel, test))

                
            
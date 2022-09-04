# -*- coding: utf-8 -*-
"""
Created on Sun Sep  4 07:36:58 2022

@author: jnguy
"""



#% Import stuff here




#% Classes 




#% Main 

import itertools
from itertools import combinations

stuff = [1, 2, 3]
for L in range(len(stuff) + 1):
    for subset in itertools.combinations(stuff, 3):
        print(subset)
        
        
perm = permutations(all_sides) 


vals = list(combinations(all_sides,len(all_sides)))
# Print the obtained permutations 
# for i in list(perm): 
#     print (i) 
    
    
    
from sympy.utilities.iterables import multiset_permutations

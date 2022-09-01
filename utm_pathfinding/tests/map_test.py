import os
import unittest 
import sys
sys.path.append('./utm_pathfinding')
print(sys)
# import utm_pathfinding
from utm_pathfinding import Map

class TestMap(unittest.TestCase):
    def setUp(self) -> None:
        x_config = 100
        y_config = 100
        z_config = 100
        self.map = Map.Map(x_config, y_config, z_config) 
        
    def test_find_which_region(self):
        """test and see if I can return correct values"""
        num_regions = 8
        self.map.break_into_square_regions(num_regions)
        
        #test quadrants
        coordinate = [0,35,0]
        correct_region = (0,1)
        region_coord = self.map.find_which_region(coordinate)
        self.assertEqual(region_coord, correct_region)
    
        coordinate = [35,35,0]
        correct_region = (1,1)
        region_coord = self.map.find_which_region(coordinate)
        self.assertEqual(region_coord, correct_region)

        coordinate = [85,85,0]
        correct_region = (3,3)
        region_coord = self.map.find_which_region(coordinate)
        self.assertEqual(region_coord, correct_region)
    
    
    # def tearDown(self) -> None:
    #     self.map.dispose()
    
if __name__ == "__main__":
    
    unittest.main()
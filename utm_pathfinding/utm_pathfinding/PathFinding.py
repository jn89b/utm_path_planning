import seaborn as sns
import numpy as np
import math as m

from queue import PriorityQueue
from scipy import spatial

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import cnames
from matplotlib import animation


class Node(object):
    """
    parent = parent of current node
    posiition = position of node right now it will be x,y coordinates
    g = cost from start to current to node
    h = heuristic 
    f = is total cost
    """
    def __init__(self, parent:list, position:list):
        self.parent = parent 
        self.position = position
        
        self.g = 0
        self.h = 0
        self.f = 0
        self.total_distance = 0
        self.total_time = 0
        
    def __lt__(self, other):
        return self.f < other.f
    
    # Compare nodes
    def __eq__(self, other):
        return self.position == other.position

    # Print node
    def __repr__(self):
        return ('({0},{1})'.format(self.position, self.f))


class AstarGraph(object):
    def __init__(self, graph:object, reservation_table:set, 
                 start_location:tuple, end_location:tuple) -> None:
        self.graph = graph
        self.reservation_table = reservation_table        
        self.start_location = start_location
        self.end_location = end_location
        self.openset = PriorityQueue() # priority queue
        self.closedset = {}
        self.iter_limit = 75000 #this is stupid should be a paramter
        
    def __init_nodes(self) -> None:
        """initialize start and end location nodes"""
        start_node = Node(None, self.start_location)
        start_node.g = start_node.h = start_node.f = 0
        self.openset.put((start_node.f, start_node))
        
        end_node = Node(None, self.end_location)
        end_node.g = end_node.h = end_node.f = 0
    
    def __check_at_goal(self, current_position:tuple) -> bool:
        """check if we have arrived at the goal"""
        if current_position == self.end_location:
            return True

    def __return_path_to_goal(self, current_node:tuple) -> list:
        """return path to starts"""
        path_goal  = []
        current = current_node 
        while current is not None:
            path_goal.append(current.position)
            current = current.parent
        #reverse path
        path_goal = path_goal[::-1]
        print("path to goal is", path_goal)
        
        return path_goal
    
    def __unpack_tuple_coordinates(self, tuple_coords:tuple) -> list:
        """return tuple coordinates as a list"""
        return [tuple_coords[0],tuple_coords[1],tuple_coords[2]]

    def __make_node(self, current_node:object, neighbor_node:object) -> object:
        """inserts a new potential node to my neighbor based on neighbor
        and references it to the current node as parent"""

        distance = self.__compute_euclidean(neighbor_node.location, self.end_location)
        extra_reward = 1
        
        if neighbor_node.node_type is not None and distance >= 30:
            if neighbor_node.node_type == "INTER":
                # print("neighbor_node", neighbor_node.region_coord)
                extra_reward == 1E-1
        elif distance <= 50:
            extra_reward == 1E-6
        
        new_node = Node(current_node, neighbor_node.location)
        new_node.g = current_node.g + neighbor_node.cost
        new_node.h = distance
        new_node.f = new_node.g + (new_node.h * extra_reward) 
        
        return new_node
    
    def __compute_euclidean(self,position, goal) -> float:
        """compute euclidean distance as heuristic"""
        distance =  m.sqrt(((position[0] - goal[0]) ** 2) + 
                            ((position[1] - goal[1]) ** 2))
                            #((position[2] - goal[2]) ** 2))
        
        return distance
    
    def main(self) -> tuple:
        """main implementation"""
        self.__init_nodes()
        
        #setting a iter count to prevent it from running forever
        iter_count = 0
        while not self.openset.empty():
            iter_count = iter_count + 1
            #pop current node off
            cost,current_node = self.openset.get()
            # print("current node is", current_node)
            if iter_count >= self.iter_limit:
                # iter count
                return iter_count,self.closedset
            
            if self.__check_at_goal(current_node.position):
                path_home = self.__return_path_to_goal(current_node)
                return path_home

            self.closedset[str(current_node.position)] = current_node
            
            current_node_position = current_node.position
            neighbors = self.graph[str(current_node_position)]

            for neighbor in neighbors:
                
                if neighbor.location == current_node_position:
                    continue
                
                if str(neighbor.location) in self.closedset:
                    continue
                
                # if neighbor.location[0:2] == current_node_position[0:2]:
                #     continue
                
                if tuple(neighbor.location) in self.reservation_table:
                    continue
                
                #make new node
                new_node = self.__make_node(current_node, neighbor)
            
                #put to open set
                self.openset.put((new_node.f, new_node))
            
            iter_count +=1
            
        if self.openset.empty():
            print("set is empty")

class AstarLowLevel(object):
    """might need to refactor the original Astar to include some set things
    or inherit from original Astar and use polymorphism for weaker post condition
    stronger postcondition constraints
    
    Improvmeents:
        refactor heuristics by the following:
            do a gradient iteration
            change delta z from start to goal:
                if negative then we want to go up so set cost to go down higher
                if positive then we want to go down so set cost to up higher
    
    """
    def __init__(self, grid, reservation_table,start, goal, vel, 
                 col_bubble, weight_factor, curr_time):
        self.grid = grid
        self.grid_x = len(grid[0])
        self.grid_y = len(grid[1])
        self.grid_z = len(grid[2])
        self.start = start
        self.goal = goal
        self.vel = vel  #had to add one 
        self.collision_bubble = col_bubble
        self.weight_factor = weight_factor
        self.reservation_table = reservation_table
        self.curr_time = curr_time
        
        self.time_bubble = 5 #have this inputted
        self.dt = 1 #seconds

        self.openset = PriorityQueue() # priority queue
        self.closedset = {}
    
        if self.start[2] - self.goal[2] < 0.0: 
            self._determine_penalty = "going down"
        else:
            self._determine_penalty = "going up"
        # else:
        #     self._determine_penalty = "level"
    
    def init_node(self):
        start_position = (self.start[0], 
                          self.start[1], 
                          self.start[2], 
                          self.curr_time)
        
        start_node = Node(None,start_position)
        start_node.g = start_node.h = start_node.f = 0
        self.openset.put((start_node.f, start_node))
        
        #self.openset.append(start_node)
        self.end_node = Node(None, tuple(self.goal))
        self.end_node.g = self.end_node.h = self.end_node.f = 0

    
    def is_move_valid(self, node_position):
        """check if move made is valid if so then return True"""
        if (node_position[0] > (self.grid_x) or 
            node_position[0] < 0 or 
            node_position[1] > (self.grid_y) or 
            node_position[1] < 0 or
            node_position[2] > self.grid_z  or
            node_position[2] < 0 ):
            return False
    
    def is_collision(self,distance:float) -> bool:
        """check if there is a collision if so return True"""
        if distance <= self.collision_bubble:
            return True
      
    def is_target_close(self, position:list, goal:list) -> bool:
        """check if we are close to target if so we remove the penalty heuristic for 
        flying high or low"""
        distance = self.compute_euclidean(position, goal)
        
        if distance <= 1.5:
            return True
        
    def return_path(self,current_node, grid):
        path = []
        no_rows = len(grid)
        no_columns = len(grid)
        # here we create the initialized result maze with -1 in every position
        result = [[-1 for i in range(no_columns)] for j in range(no_rows)]
        current = current_node
        
        while current is not None:
            path.append(current.position)
            current = current.parent
        # Return reversed path as we need to show from start to end path
        path = path[::-1]
        start_value = 0
        waypoints = []
        for points in path:
            waypoints.append(points)
            
        return waypoints
    
    def compute_euclidean(self,position:tuple, goal:tuple) -> float:
        """compute euclidean distance"""
        distance =  m.sqrt(((position[0] - goal.position[0]) ** 2) + 
                           ((position[1] - goal.position[1]) ** 2) +
                           ((position[2] - goal.position[2]) ** 2))
        
        return distance

    def get_moves(self, ss:int) -> list:
        """returns all 3d moves based on a step size"""
        bounds = list(np.arange(-ss, ss+1, 1))
        move_list = []
        for i in bounds:
            for j in bounds:
                for k in bounds:
                    move_list.append([i,j,k])
        
        #remove the value that doesnt move
        move_list.remove([0,0,0])
        return move_list
    
    def __determine_penalty(self):
        """determines the penalty of the cost for changing height
        if want to end at higher position -> penalize going down
        if want to end at lower position -> penalize going up """
        
        if self.start[2] - self.goal[2] <= 0: 
            return "going down"
        else:
            return "going up"
    
    def __compute_penalty(self, current_z, new_z):
        """compute penalty"""
        diff_z = current_z - new_z 
        if self._determine_penalty == "going down" and diff_z > 0:
            return 1
        if self._determine_penalty == "going up" and diff_z < 0:
            return 1
        # if self._determine_penalty =="level":
        #     return 1
            
        return 1.0
    
    def at_goal(self, current_node:Node) -> bool:
        curr_pos = (current_node.position[0], 
                    current_node.position[1], 
                    current_node.position[2])
        
        if curr_pos == self.goal:
            return True
    
    def main(self):
        ss = 1
        count = 0 
        
        move = self.get_moves(ss)
        self.init_node()
        rounded_dist_traveled = 0
        
        """main implementation"""
        while not self.openset.empty():
            count = count + 1
            
            if count >= 10000:
                print("iterations too much for low level")
                return 0, count, self.closedset
            
            #pop node off from priority queue and add into closedset
            cost,current_node = self.openset.get()
            self.closedset[current_node.position] = current_node
            # rounded_curr_dist = round(current_node.total_distance)
            
            #check if we hit the goal 
            #if current_node.position == self.end_node.position:
            if self.at_goal(current_node):
                #print("Goal reached", current_node.position)
                path = self.return_path(current_node, self.grid)
                print("success!", count)
                return path, count, self.closedset 
                  
            #move generation
            children = []
    
            for new_position in move:
                
                node_position = (current_node.position[0] + new_position[0], 
                                 current_node.position[1] + new_position[1], 
                                 current_node.position[2] + new_position[2],
                                 current_node.total_time)
                
                # Make sure within range (check if within maze boundary)
                if self.is_move_valid(node_position) == False:
                    #print("move is invalid", node_position)
                    continue
    
                if tuple(node_position) in self.reservation_table:
                    # print("its in the low level reservation table")
                    continue
                
                #create new node
                new_node = Node(current_node, node_position)
                
                # put to possible paths
                children.append(new_node)
                    
            #check each children 
            for child in children:
                #check if children is already visited
                if child.position in self.closedset:
                    #print("Exists", child.position)
                    continue
                
                """refactor this, determine if I am going up or down then 
                adjust the heuristic"""
                penalty = self.__compute_penalty(
                    current_node.position[2],  child.position[2]) 
                
                """Heuristic costs calculated here, this is using eucledian distance"""
                #print("child.position", child.position)
                if self.is_target_close(current_node.position, self.end_node):
                    cost = self.compute_euclidean(current_node.position, child)
                    child.g = current_node.g + 1#cost
                    child.h = self.compute_euclidean(child.position, self.end_node)
                    dynamic_weight = 0.75
                    child.f = child.g + (child.h *penalty*dynamic_weight)
                    child.total_distance = child.g + 1#cost 
                    child.total_time = round(child.total_distance/ self.vel)
                else:
                    dynamic_weight = self.weight_factor
                    cost = self.compute_euclidean(current_node.position, child)
                    child.g = current_node.g + cost
                    child.h = self.compute_euclidean(child.position, self.end_node)
                    child.f = child.g + (child.h *penalty*dynamic_weight)
                    child.total_distance = child.g + cost
                    child.total_time = round(child.total_distance/ self.vel)
                
                self.openset.put((child.f, child))
                
        if self.openset.empty():
            print("open set is empty")
            return 0, count, self.closedset 



class AnimateMultiUAS():
    def __init__(self, uas_paths:list, method_name:str):
        self.uas_paths = uas_paths
        self.method_name = method_name
        self.color_list = sns.color_palette("hls", len(uas_paths))
        self.bad_index = [] #bad indexes are index locations where paths cant be found
        
    def set_size_params(self,x_bounds:list, y_bounds:list, 
                        z_bounds:list):
        self.fig = plt.figure(figsize=(8, 8))
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_xlim3d(x_bounds[0], x_bounds[1])
        self.ax.set_ylim3d(y_bounds[0], y_bounds[1])
        self.ax.set_zlim3d(z_bounds[0], z_bounds[1])
        self.title = self.ax.set_title(self.method_name,fontsize=18)

    def plot_initial_final(self, start_pos, goal_pos, color):
        """plots the initial and final points of a UAS with a color"""
        self.graph = self.ax.scatter(start_pos[0], start_pos[1],
                        start_pos[2], color=color, s=100, marker='o')
        
        self.graph = self.ax.scatter(goal_pos[0], goal_pos[1], 
                        goal_pos[2], color=color, s=100, marker='x')
        
        return self.graph

    def plot_path(self, x_bounds:list,y_bounds:list, z_bounds:list) -> None:
        """plot paths"""
        self.set_size_params(x_bounds, y_bounds, z_bounds)
        for i, path in enumerate(self.uas_paths):
            x = [position[0] for position in path]
            y = [position[1] for position in path]
            z = [position[2] for position in path]
            
            self.graph = self.plot_initial_final(path[0], 
                                                    path[-1], 
                                                    self.color_list[i])
            
            self.ax.scatter(x,y,z, marker='o', color=self.color_list[i], s=100)
        
    def init(self):
        for line, pt in zip(self.lines, self.pts):
            line.set_data([], [])
            line.set_3d_properties([])

            pt.set_data([], [])
            pt.set_3d_properties([])
        return self.lines + self.pts

    def update_multi(self,i:int):
        # we'll step two time-steps per frame.  This leads to nice results.
        # i = (2 * i) % x_t.shape[0]

        # for line, pt, xi in zip(lines, pts, x_t):
        for j, (line,pt) in enumerate(zip(self.lines,self.pts)):
            time_span = 1
            if i < time_span:
                interval = 0
            else:
                interval = i - time_span
            
            #set lines 
            line.set_data(self.x_list[j][:i], self.y_list[j][:i])
            line.set_3d_properties(self.z_list[j][:i])

            # #set points
            pt.set_data(self.x_list[j][interval:i], self.y_list[j][interval:i])
            pt.set_3d_properties(self.z_list[j][interval:i])
        
            #changing views
            # self.ax.view_init(60, 0.3 * i)
            # self.fig.canvas.draw()
            
        # fig.canvas.draw()
        return self.lines + self.pts
    

    def animate_multi_uas(self, x_bounds:list, y_bounds:list, z_bounds:list, 
                            axis_on=True, save=False):
        """animate and simulate multiple uas"""
        #https://stackoverflow.com/questions/56548884/saving-matplotlib-animation-as-movie-movie-too-short
        #marker_size = 80
        self.set_size_params(x_bounds, y_bounds, z_bounds)
        
        if axis_on == False:
            self.ax.axis("off")
        
        self.x_list = []
        self.y_list = []
        self.z_list = []
        
        for i, uav_path in enumerate(self.uas_paths):
            
            #checking if we have a path
            if not uav_path:
                self.bad_index.append(i)
            else:
                self.curr_color = self.color_list[i]
                x_list = [position[0] for position in uav_path]
                y_list = [position[1] for position in uav_path]
                z_list = [position[2] for position in uav_path]
                
                start_pos = [x_list[0], y_list[0], z_list[0]]
                goal_pos  = [x_list[-1], y_list[-1], z_list[-1]]
                
                self.graph = self.plot_initial_final(start_pos, 
                                                        goal_pos, 
                                                        self.color_list[i])         
            
                self.x_list.append(x_list)
                self.y_list.append(y_list)
                self.z_list.append(z_list)
                
        # set up lines and points
        self.lines = [self.ax.plot([], [], [], linewidth=2)[0] 
                        for _ in range(len(self.uas_paths))]

        self.pts = [self.ax.plot([], [], [], 'o')[0] 
                        for _ in range(len(self.uas_paths))]
        
        for i, (line,pt) in enumerate(zip(self.lines,self.pts)):
            line._color = self.color_list[i]
            pt._color = self.color_list[i]
    
            
        #get which solution is the longest to use as frame reference
        max_list = max(self.uas_paths, key=len)

        self.ani = animation.FuncAnimation(
            self.fig, self.update_multi, frames=len(max_list),repeat=True, 
            interval=5,save_count=len(max_list))
        
        self.ani = animation.FuncAnimation(
            self.fig, self.update_multi, init_func=self.init,
            frames=len(max_list), interval=50, blit=True, 
            repeat=True)
        
        if save == True:
            print("saving")
            # writervideo = FFMpegWriter(fps=10)
            self.ani.save('videos/'+self.method_name+'.mp4', 
                            writer=writervideo)
                    
        plt.show()


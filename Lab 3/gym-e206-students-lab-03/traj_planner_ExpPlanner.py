# E206 Motion Planning

# Simple planner
# C Clark

import math
import dubins
import random
import matplotlib.pyplot as plt
from traj_planner_utils import *
import numpy as np
import time

class Node():

  def __init__(self, state, parent_node, edge_distance):
    self.state = state
    self.parent_node = parent_node
    self.edge_distance = edge_distance
    
  def manhattan_distance_to_node(self, node):
    return abs(self.state[1] - node.state[1]) + abs(self.state[2] - node.state[2])
  
  def manhattan_distance_to_state(self, state):
    return abs(self.state[1] - state[1]) + abs(self.state[2] - state[2])
    
  def euclidean_distance_to_state(self, state):
    return math.sqrt( (self.state[1] - state[1])**2 + (self.state[2] - state[2])**2 )

class Expansive_Planner():
  
  DISTANCE_DELTA = 1.5 #m
  DIST_TO_GOAL_THRESHOLD = 0.5 #m
  GRID_RESOLUTION = 0.5 #m
  EDGE_TIME = 10 #s
  LARGE_NUMBER = 9999999
  
  MAX_NUM_ITERATIONS = 1000
  MIN_RAND_DISTANCE = 1.0 #m
  MAX_RAND_DISTANCE = 5.0 #m
  MEAN_EDGE_VELOCITY = 0.75 #m
  PLAN_TIME_BUDGET = 0.5 #s
    
  def __init__(self):
    self.fringe = []

  def construct_traj(self, initial_state, desired_state, objects, walls):
    """ Construct a trajectory in the X-Y space and in the time-X,Y,Theta space.
        Arguments:
          traj_point_0 (list of floats): The trajectory's first trajectory point with time, X, Y, Theta (s, m, m, rad).
          traj_point_1 (list of floats): The trajectory's last trajectory point with time, X, Y, Theta (s, m, m, rad).
        Returns:
          traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
          traj_cost (float): The path length (m).
    """
    self.tree = []
    self.desired_state = desired_state
    self.objects = objects
    self.walls = walls
    self.grid = {}  # Dictionary with the keys being the indices of the normalized grid 
    
    start_node = Node(initial_state, None, 0)
    self.add_to_tree(start_node)
    start_time = time.perf_counter()

    while time.perf_counter() - start_time < 100:
      rand_node = self.sample_random_node()
      expansion_node = self.generate_random_node(rand_node)
      if not self.collision_found(rand_node, expansion_node):
        self.add_to_tree(expansion_node)
        goal_node = self.generate_goal_node(expansion_node, desired_state)
        if not goal_node == None:
          self.add_to_tree(goal_node)
          return self.build_traj(goal_node)
      
    return [], self.LARGE_NUMBER
        
  def construct_optimized_traj(self, initial_state, desired_state, objects, walls):
    """ Construct the best trajectory possible within a limited time budget.
        Arguments:
          traj_point_0 (list of floats): The trajectory's first trajectory point with time, X, Y, Theta (s, m, m, rad).
          traj_point_1 (list of floats): The trajectory's last trajectory point with time, X, Y, Theta (s, m, m, rad).
        Returns:
          best_traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
          best_traj_cost (float): The path lenght of the shortest traj (m).
    """
    start_time = time.perf_counter()
    current_time = start_time
    best_traj = []
    best_traj_cost = self.LARGE_NUMBER
    TIME_BUDGET = 5 #In seconds
    print("The time budget is: " + str(TIME_BUDGET))
    while (current_time - start_time) < TIME_BUDGET:
      traj, traj_cost = self.construct_traj(initial_state, desired_state, objects, walls)
      if traj_cost < best_traj_cost:
        best_traj_cost = traj_cost
        best_traj = traj
      current_time = time.perf_counter()
    return best_traj, best_traj_cost
     
  def add_to_tree(self, node):
    """ Add the node to the tree.
        Arguments:
          node (Node): The node to be added.
    """
    _, x, y, _ = node.state
    x_index = math.floor(x / self.GRID_RESOLUTION)
    y_index = math.floor(y / self.GRID_RESOLUTION)
    x_y = (x_index, y_index)

    if x_y in self.grid:
      self.grid[x_y].append(node)
    else:
      self.grid[x_y] = [node]

    self.tree.append(node)
    pass
    
  def sample_random_node(self):
    """ Randomly select a node from the tree and return it.
        Returns:
          node (Node): A randomly selected node from the tree.
    """
    key = random.choice(list(self.grid.keys()))
    node = random.choice(self.grid[key])

    return node

  def generate_random_node(self, node_to_expand):
    """ Create a new node by expanding from the parent node using.
        Arguments:
          node_to_expand (Node): The parent node.
        Returns:
          new_node (Node): The newly generated node.
    """
    # Add code here to make a new node #
    not_valid = True
    x = 0
    y = 0

    while not_valid:
      random_distance = random.randint(self.MIN_RAND_DISTANCE, self.MAX_RAND_DISTANCE)
      random_angle = random.uniform(-math.pi, math.pi)
      
      x = node_to_expand.state[1] + random_distance * math.cos(node_to_expand.state[3] + random_angle)
      y = node_to_expand.state[2] + random_distance * math.sin(node_to_expand.state[3] + random_angle)

      #  walls (list of lists): A list of walls defined by end points - X0, Y0, X1, Y1, length (m, m, m, m, m).
      x_min = self.walls[0][0]
      x_max = self.walls[0][2]
      y_min = self.walls[1][3]
      y_max = self.walls[1][1]

      if x_min <= x <= x_max and y_min <= y <= y_max:
        not_valid = False  

    time = node_to_expand.state[0] + random_distance/self.MEAN_EDGE_VELOCITY
    theta = node_to_expand.state[3] + random_angle
    state = (time, x, y, theta)

    return Node(state, node_to_expand, self.calculate_edge_distance(state, node_to_expand))

  def generate_goal_node(self, node, desired_state):
    """ Create a goal node by connecting from the parent node using.
        Arguments:
          node_to_expand: The parent node.
        Returns:
          goal_node: The newly generated goal node or None if there is not goal connection.
    """
    goal_node = Node(desired_state, node, self.calculate_edge_distance(desired_state, node))
    collision = self.collision_found(node, goal_node)
    if (not collision):
      return goal_node
    else:
      return None

  def calculate_edge_distance(self, state, parent_node):
    """ Calculate the cost of an dubins path edge from a parent node's state to another state.
        Arguments:
          state: The end state of the edge.
          parent_node: The initial state node of the edge.
        Returns:
          traj_distance: The length of the edge, or is the LARGE_NUMBER if collision exists (m).
    """
    traj, traj_distance = construct_dubins_traj(parent_node.state, state)
    if collision_found(traj, self.objects, self.walls):
      return self.LARGE_NUMBER

    return traj_distance

  def build_traj(self, goal_node):
    """ Build a traj via back tracking from a goal node.
        Arguments:
          goal_node: The node to back track from and create a traj of dubins paths with.
        Returns:
          traj (list of list of floats): The trajectory as a list of time, X, Y, Theta (s, m, m, rad).
          traj_cost (float): The length of the traj (m).
    """
    node_list = []
    node_to_add = goal_node
    while node_to_add != None:
      node_list.insert(0, node_to_add)
      node_to_add = node_to_add.parent_node
  
    traj = []
    traj_cost = 0
    for i in range(1,len(node_list)):
      node_A = node_list[i-1]
      node_B = node_list[i]
      traj_point_0 = node_A.state
      traj_point_1 = node_B.state
      traj_point_1 = list(traj_point_1)
      traj_point_1[3] = math.atan2(traj_point_1[2]-traj_point_0[2], traj_point_1[1]-traj_point_0[1])
      traj_point_1 = tuple(traj_point_1)
      edge_traj, edge_traj_distance = construct_dubins_traj(traj_point_0, traj_point_1)
      traj = traj + edge_traj
      traj_cost = traj_cost + edge_traj_distance
    
    return traj, traj_cost

  def collision_found(self, node_1, node_2):
    """ Return true if there is a collision with the traj between 2 nodes and the workspace
        Arguments:
          node_1 (Node): A node with the first state of the traj - Time, X, Y, Theta (s, m, m, rad).
          node_2 (Node): A node with the second state of the traj - Time, X, Y, Theta (s, m, m, rad).
          objects (list of lists): A list of object states - X, Y, radius (m, m, m).
          walls (list of lists): A list of walls defined by end points - X0, Y0, X1, Y1, length (m, m, m, m, m).
        Returns:
          collision_found (boolean): True if there is a collision.
    """
    traj, traj_distance = construct_dubins_traj(node_1.state, node_2.state)
    return collision_found(traj, self.objects, self.walls)

if __name__ == '__main__':
  minimum = 1000000
  maximum = 0
  mean = 0
  for i in range(0, 50):
    print("On iteration: " + str(i))
    maxR = 10
    tp0 = [0, -8, -8, 0]
    tp1 = [40, 8, 8, 0]
    planner = Expansive_Planner()
    walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
    num_objects = 7
    objects = []
    
    for j in range(0, num_objects): 
      obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0.6]
      while (abs(obj[0]-tp0[1]) < 1 and abs(obj[1]-tp0[2]) < 1) or (abs(obj[0]-tp1[1]) < 1 and abs(obj[1]-tp1[2]) < 1):
        obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 1.0]
      objects.append(obj)
    traj, traj_cost = planner.construct_optimized_traj(tp0, tp1, objects, walls)
    # traj, traj_cost = planner.construct_traj(tp0, tp1, objects, walls)
    if len(traj) > 0:
      #plot_traj(traj, traj, objects, walls)
      if traj_cost < minimum:
        minimum = traj_cost
      if traj_cost > maximum:
        maximum = traj_cost
      mean += traj_cost
      # print("The traj cost is: " + str(traj_cost))

  mean = mean/50
  print("The min traj_cost is: " + str(minimum))
  print("The max traj_cost is: " + str(maximum))
  print("The average traj_cost is: " + str(mean))
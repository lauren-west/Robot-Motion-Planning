# E206 Motion Planning

# Simple planner
# C Clark

import math
import dubins
import random
import matplotlib.pyplot as plt
from traj_planner_utils import *
import numpy as np

total_traj_distance = 0

class Node():

  def __init__(self, state, parent_node, g_cost, h_cost):
    self.state = state
    self.parent_node = parent_node
    self.g_cost = g_cost
    self.h_cost = h_cost
    self.f_cost = self.g_cost + self.h_cost

  def getState():
    return self.state
    
  def manhattan_distance_to_node(self, node):
    return abs(self.state[1] - node.state[1]) + abs(self.state[2] - node.state[2])
  
  def manhattan_distance_to_state(self, state):
    return abs(self.state[1] - state[1]) + abs(self.state[2] - state[2])
    
  def euclidean_distance_to_state(self, state):
    return math.sqrt( (self.state[1] - state[1])**2 + (self.state[2] - state[2])**2 )

class A_Star_Planner():
  
  DIST_TO_GOAL_THRESHOLD = 0.5 #m
  CHILDREN_DELTAS = [-0.5, -0.25, 0.0, 0.25, 0.5]
  #CHILDREN_DELTAS = [-0.3, -0.15, 0.0, 0.15, 0.3]
  #CHILDREN_DELTAS = [-0.6, -0.3, 0.0, 0.3, 0.6]
  #CHILDREN_DELTAS = [-0.53, -0.265, 0.0, 0.265, 0.53]
  #CHILDREN_DELTAS = [-0.47, -0.235, 0.0, 0.235, 0.47]
  #CHILDREN_DELTAS = [-0.4, -0.2, 0.0, 0.2, 0.4]
  #CHILDREN_DELTAS = [-0.7, -0.35, 0.0, 0.35, 0.7]
  DISTANCE_DELTA = 1.5 #m
  EDGE_TIME = 10 #s
  LARGE_NUMBER = 9999999

  def __init__(self):
    self.fringe = []

  def construct_traj(self, initial_state, desired_state, objects, walls):
    """ Construct a trajectory in the X-Y space and in the time-X,Y,Theta space.
        Arguments:
          traj_point_0 (list of floats): The trajectory's first trajectory point with time, X, Y, Theta (s, m, m, rad).
          traj_point_1 (list of floats): The trajectory's last trajectory point with time, X, Y, Theta (s, m, m, rad).
        Returns:
          traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
    """
    
    self.desired_state = desired_state
    self.objects = objects
    self.walls = walls
    self.fringe = []

    initial_node = self.create_initial_node(initial_state)
    self.fringe.append(initial_node)



    while self.generate_goal_node(self.fringe[0], desired_state) == None:
      newNode = self.get_best_node_on_fringe()
      children_list = self.get_children(newNode)
      for child in children_list:
        self.add_to_fringe(child)
    
    new_desired = (self.fringe[0].state[0] + self.EDGE_TIME, desired_state[1], desired_state[2], desired_state[3])
    goalNode = self.generate_goal_node(self.fringe[0], new_desired)

    return self.build_traj(goalNode)

  def add_to_fringe(self, node): 
    # student written
    if len(self.fringe) == 0:
      self.fringe.append(node)
    else:
      for i in range(len(self.fringe)):
        if self.fringe[i].f_cost > node.f_cost:
          self.fringe.insert(i, node)
          break 
      self.fringe.insert(-1, node)
      
    
  def get_best_node_on_fringe(self):
    return self.fringe.pop(0)
    
  def get_children(self, node_to_expand):
    children_list = []

    # Psuedocode from E206 site
    for i in range(len(self.CHILDREN_DELTAS)):
      CHILDREN_DELTA = self.CHILDREN_DELTAS[i]
      time = node_to_expand.state[0] + self.EDGE_TIME
      x = node_to_expand.state[1] + self.DISTANCE_DELTA * math.cos(node_to_expand.state[3] + CHILDREN_DELTA)
      y = node_to_expand.state[2] + self.DISTANCE_DELTA * math.sin(node_to_expand.state[3] + CHILDREN_DELTA)
      theta = node_to_expand.state[3] + 2 * CHILDREN_DELTA
      
      # student written
      state = (time, x, y, theta)
      child = self.create_node(state, node_to_expand)
      children_list.append(child)
  
    return children_list

  def generate_goal_node(self, node, desired_state):
    # student written
    
    # Add code here.
    collision, _ = self.collision_found(node.state, desired_state)
    if (not collision):
      goal_node = self.create_node(desired_state, node)
      return goal_node
    else:
      return None

    
  def create_node(self, state, parent_node):
    
    h_cost = self.estimate_cost_to_goal(self.desired_state)
    g_cost = parent_node.g_cost + self.calculate_edge_distance(state, parent_node)
    
    return Node(state, parent_node, g_cost, h_cost)


  def create_initial_node(self, state):
    h_cost = self.estimate_cost_to_goal(self.desired_state)
    g_cost = 0
    return Node(state, None, g_cost, h_cost)


  def calculate_edge_distance(self, state, parent_node):
    collision, traj_distance = self.collision_found(parent_node.state, state)
    
    # CAREFUL Collision found takes in two nodes, we pass in a state and a node
    if collision:
      return self.LARGE_NUMBER
    else:
      return traj_distance

  def estimate_cost_to_goal(self, state):
    return math.sqrt( (self.desired_state[1] - state[1])**2 + (self.desired_state[2] - state[2])**2 )

  def build_traj(self, goal_node):
    
    global total_traj_distance
    total_traj_distance = 0

    node_list = []
    node_to_add = goal_node
    while node_to_add != None:
      node_list.insert(0, node_to_add)
      node_to_add = node_to_add.parent_node
  
    traj = []
    for i in range(1,len(node_list)):
      node_A = node_list[i-1]
      node_B = node_list[i]
      traj_point_0 = node_A.state
      traj_point_1 = node_B.state
      traj_point_1 = list(traj_point_1)
      traj_point_1[3] = math.atan2(traj_point_1[2]-traj_point_0[2], traj_point_1[1]-traj_point_0[1])
      traj_point_1 = tuple(traj_point_1)
      edge_traj, edge_traj_distance = construct_dubins_traj(traj_point_0, traj_point_1)
      
      total_traj_distance += edge_traj_distance
      traj = traj + edge_traj

      ########## CHILDREN_DELTA = [-0.5, -0.25, 0.0, 0.25, 0.5] ################
      # traj length: 11.998502403593706

      ########## CHILDREN_DELTA = [-0.3, -0.15, 0.0, 0.15, 0.3] ################
      # traj length: 12.898756032515823
      
      ########## CHILDREN_DELTA = [-0.6, -0.3, 0.0, 0.3, 0.6]
      # traj length: 12.098711900557902

      ########## CHILDREN_DELTA = [-0.53, -0.265, 0.0, 0.265, 0.53]
      # traj length: 12.398229837927044

       ########## CHILDREN_DELTA = [-0.47, -0.235, 0.0, 0.235, 0.47]
       # traj length: 12.199023142374802

      ########## CHILDREN_DELTA = [-0.4, -0.2, 0.0, 0.2, 0.4]
      # traj length:11.998929287263906

      ########## CHILDREN_DELTA = [-0.7, -0.35, 0.0, 0.35, 0.7]
      # traj length: 12.097732792113147

    return traj

  # changed arguments from starter code collision_found(self, node_1, node_2) to what it is now
  def collision_found(self, state_1, state_2):
    """ Return true if there is a collision with the traj between 2 nodes and the workspace
        Arguments:
          node_1 (Node): A node with the first state of the traj - Time, X, Y, Theta (s, m, m, rad).
          node_2 (Node): A node with the second state of the traj - Time, X, Y, Theta (s, m, m, rad).
          objects (list of lists): A list of object states - X, Y, radius (m, m, m).
          walls (list of lists): A list of walls defined by end points - X0, Y0, X1, Y1, length (m, m, m, m, m).
        Returns:
          collision_found (boolean): True if there is a collision.
    """
    # added traj_distance as a return value 
    traj, traj_distance = construct_dubins_traj(state_1, state_2)
    return collision_found(traj, self.objects, self.walls), traj_distance



if __name__ == '__main__':
  # for i in range(0, 5):
  #   maxR = 10
  #   tp0 = [0, -8, -8, 0]
  #   tp1 = [20, 0, 0, 0]
  #   # tp1 = [300, random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0]
  #   planner = A_Star_Planner()
  #   walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
  #   # num_objects = 25
  #   # objects = []
  #   # for j in range(0, num_objects): 
  #   #   obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0.5]
  #   #   while (abs(obj[0]-tp0[1]) < 1 and abs(obj[1]-tp0[2]) < 1) or (abs(obj[0]-tp1[1]) < 1 and abs(obj[1]-tp1[2]) < 1):
  #   #     obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0.5]
  #   #   objects.append(obj)
  #   objects = [[-9, -4, 1], [-7, -4, 1], [-5, -4, 1], [-3, -4, 1], [-1, -4, 1], [-4, -2, 1], [1, -4, 1], [3, 3, 0.5]]
  #   traj = planner.construct_traj(tp0, tp1, objects, walls)
  #   if len(traj) > 0:
  #     plot_traj(traj, traj, objects, walls)

    maxR = 10
    tp0 = [0, -8, -8, 0]
    tp1 = [20, 0, 0, 0]
    # tp1 = [300, random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0]
    planner = A_Star_Planner()
    walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
    # num_objects = 25
    # objects = []
    # for j in range(0, num_objects): 
    #   obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0.5]
    #   while (abs(obj[0]-tp0[1]) < 1 and abs(obj[1]-tp0[2]) < 1) or (abs(obj[0]-tp1[1]) < 1 and abs(obj[1]-tp1[2]) < 1):
    #     obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0.5]
    #   objects.append(obj)
    objects = [[-8, -4, 1.5], [-3, -4, 1.5], [-1, -4, 1], [-5, -8, 1], [-2, -6, 1], [1, -4, 1]]
    traj = planner.construct_traj(tp0, tp1, objects, walls)
    print(total_traj_distance)
    if len(traj) > 0:
      plot_traj(traj, traj, objects, walls)  

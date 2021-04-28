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

  def getState(self):
    return self.state
    
  def manhattan_distance_to_node(self, node):
    return abs(self.state[1] - node.state[1]) + abs(self.state[2] - node.state[2])
  
  def manhattan_distance_to_state(self, state):
    return abs(self.state[1] - state[1]) + abs(self.state[2] - state[2])
    
  def euclidean_distance_to_state(self, state):
    return math.sqrt( (self.state[1] - state[1])**2 + (self.state[2] - state[2])**2 )

class Shark():

  SHARK_VELOCITY = 2.22222        # m/s
  MIN_RAND_DISTANCE = 1           # m
  MAX_RAND_DISTANCE = 3           # m
  MAX_DELTA_THETA = math.pi / 4   # rad
  DESIRED_STATE_THETA = 1.39626   # rad
  DESIRED_STATE_RADIUS = 2.5      # m  (will scale up if need be)

  MIN_DESIRED_RADIUS = DESIRED_STATE_RADIUS - 1    # m
  MAX_DESIRED_RADIUS = DESIRED_STATE_RADIUS + 1    # m

  def __init__(self, state, boundaries):
    self.state = state            # Contains [x, y, theta]
    self.previous_states = []
    self.boundaries = boundaries

  def getState(self):
    return self.state

  def euclidean_distance_to_state(self, state):
    return math.sqrt( (self.state[1] - state[1])**2 + (self.state[2] - state[2])**2 )

  def updateState(self):
    x, y, theta = self.state
    self.previous_states.append(self.state)

    not_valid = True

    while not_valid:
      random_distance = random.randint(self.MIN_RAND_DISTANCE, self.MAX_RAND_DISTANCE)
      random_angle = random.uniform(self.MAX_DELTA_THETA, self.MAX_DELTA_THETA)
      
      x = x + random_distance * math.cos(theta + random_angle)
      y = y + random_distance * math.sin(theta + random_angle)

      x_min = self.boundaries[0][0]
      x_max = self.boundaries[0][2]
      y_min = self.boundaries[1][3]
      y_max = self.boundaries[1][1]

      if x_min <= x <= x_max and y_min <= y <= y_max:
        not_valid = False  

    theta = node_to_expand.state[3] + random_angle
    self.state = (x, y, theta)

  def get_desired_state(self):

    x_min = self.boundaries[0][0]
    x_max = self.boundaries[0][2]
    y_min = self.boundaries[1][3]
    y_max = self.boundaries[1][1]

    x, y, theta = self.state

    x_des_1 = x + self.DESIRED_STATE_RADIUS * math.cos(theta + self.DESIRED_STATE_THETA)
    y_des_1 = y + self.DESIRED_STATE_RADIUS * math.sin(theta + self.DESIRED_STATE_THETA)
    theta_des_1 = theta + 2 * self.DESIRED_STATE_THETA
    state_1 = (x_des_1, y_des_1, theta_des_1)

    x_des_2 = x + self.DESIRED_STATE_RADIUS * math.cos(theta - self.DESIRED_STATE_THETA)
    y_des_2 = y + self.DESIRED_STATE_RADIUS * math.sin(theta - self.DESIRED_STATE_THETA)
    theta_des_2 = theta - 2 * self.DESIRED_STATE_THETA
    state_2 = (x_des_2, y_des_2, theta_des_2)
    
    if not (x_min <= x_des_1 <= x_max and y_min <= y_des_1 <= y_max):
      return state_2

    if not (x_min <= x_des_2 <= x_max and y_min <= y_des_2 <= y_max):
      return state_1
    
    # Consider adding check for if state is in object

    if self.euclidean_distance_to_state(state_1) <= self.euclidean_distance_to_state(state_2):
      return state_1
    
    return state_2


class A_Star_Planner():
  
  DIST_TO_GOAL_THRESHOLD = 0.5 #m
  CHILDREN_DELTAS = [-0.5, -0.25, 0.0, 0.25, 0.5]
  DISTANCE_DELTA = 1.5 #m
  EDGE_TIME = 10 #s
  LARGE_NUMBER = 9999999

  def __init__(self):
    self.fringe = []

  def construct_traj(self, initial_state, desired_state, objects, walls, shark):
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
    self.shark = shark
    

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
  for i in range(0, 5):
    maxR = 10
    tp0 = [0, -8, -8, 0]
    tp1 = []
    planner = A_Star_Planner()
    walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
    shark = Shark((0, 0, 0), walls)
    x, y, theta = shark.get_desired_state()
    tp1 = [300, x, y, theta]
    # Call below repeatedly
    objects = []
    num_objects = 25
    for j in range(0, num_objects): 
      obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0.5]
      while (abs(obj[0]-tp0[1]) < 1 and abs(obj[1]-tp0[2]) < 1) or (abs(obj[0]-tp1[1]) < 1 and abs(obj[1]-tp1[2]) < 1):
        obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0.5]
      objects.append(obj)
    
    traj = planner.construct_traj(tp0, tp1, objects, walls, shark)
    shark.updateState()
    x, y, theta = shark.get_desired_state()
    tp1 = [300, x, y, theta]

    if len(traj) > 0:
      plot_traj(traj, traj, objects, walls)

    # maxR = 10
    # tp0 = [0, -8, -8, 0]
    # tp1 = [20, 0, 0, 0]
    # # tp1 = [300, random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0]
    # planner = A_Star_Planner()
    # walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
    # # num_objects = 25
    # # objects = []
    # # for j in range(0, num_objects): 
    # #   obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0.5]
    # #   while (abs(obj[0]-tp0[1]) < 1 and abs(obj[1]-tp0[2]) < 1) or (abs(obj[0]-tp1[1]) < 1 and abs(obj[1]-tp1[2]) < 1):
    # #     obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0.5]
    # #   objects.append(obj)
    # objects = [[-8, -4, 1.5], [-3, -4, 1.5], [-1, -4, 1], [-5, -8, 1], [-2, -6, 1], [1, -4, 1]]
    # traj = planner.construct_traj(tp0, tp1, objects, walls)
    # print(total_traj_distance)
    # if len(traj) > 0:
    #   plot_traj(traj, traj, objects, walls)  
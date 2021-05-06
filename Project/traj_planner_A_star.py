# E206 Motion Planning Project

import math
import dubins
import random
import matplotlib.pyplot as plt
import time
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
  MAX_DESIRED_RADIUS = DESIRED_STATE_RADIUS + 1.2    # m

  TIME_STEP = 1     # sec

  def __init__(self, state, boundaries):
    self.state = state            # Contains [x, y, theta]
    self.previous_states = []
    self.boundaries = boundaries

  def getState(self):
    return self.state

  def euclidean_distance_to_state(self, current_state, state):
    return math.sqrt( (current_state[2] - state[1])**2 + (current_state[3] - state[2])**2 )

  def updateState(self):
    x, y, theta = self.state
    if self.previous_states == []:
      self.previous_states.append((0, x, y, theta))
    else:
      time = self.previous_states[-1][0] + TIME_STEP
      self.previous_states.append((time, x, y, theta))

    not_valid = True

    while not_valid:
      random_distance = random.randint(self.MIN_RAND_DISTANCE, self.MAX_RAND_DISTANCE)
      random_angle = random.uniform(-self.MAX_DELTA_THETA, self.MAX_DELTA_THETA)
      
      x = x + random_distance * math.cos(theta + random_angle)
      y = y + random_distance * math.sin(theta + random_angle)

      x_min = self.boundaries[0][0]
      x_max = self.boundaries[0][2]
      y_min = self.boundaries[1][3]
      y_max = self.boundaries[1][1]

      if x_min <= x <= x_max and y_min <= y <= y_max:
        not_valid = False  

    theta = theta + random_angle
    self.state = (x, y, theta)

  def get_desired_state(self, current_state):

    x_min = self.boundaries[0][0]
    x_max = self.boundaries[0][2]
    y_min = self.boundaries[1][3]
    y_max = self.boundaries[1][1]

    x, y, theta = self.state

    x_des_1 = x + self.DESIRED_STATE_RADIUS * math.cos(theta + self.DESIRED_STATE_THETA)
    y_des_1 = y + self.DESIRED_STATE_RADIUS * math.sin(theta + self.DESIRED_STATE_THETA)
    state_1 = (x_des_1, y_des_1, theta)
    

    x_des_2 = x + self.DESIRED_STATE_RADIUS * math.cos(theta - self.DESIRED_STATE_THETA)
    y_des_2 = y + self.DESIRED_STATE_RADIUS * math.sin(theta - self.DESIRED_STATE_THETA)
    state_2 = (x_des_2, y_des_2, theta)
    
    # print(f"State1: {state_1}")
    # print(f"State2: {state_2}")

    if not (x_min <= x_des_1 <= x_max and y_min <= y_des_1 <= y_max):
      return state_2 #2

    if not (x_min <= x_des_2 <= x_max and y_min <= y_des_2 <= y_max):
      return state_1
    
    distance_to_1 = self.euclidean_distance_to_state(current_state, state_1)
    distance_to_2 = self.euclidean_distance_to_state(current_state, state_2)

    # Consider adding check for if state is in object
    if distance_to_1 <= distance_to_2:
      return state_1
    
    return state_2 #2


class A_Star_Planner():
  
  DIST_TO_GOAL_THRESHOLD = 0.5 #m
  CHILDREN_DELTAS = [-0.5, -0.25, 0.0, 0.25, 0.5]
  DISTANCE_DELTA = 1.5 #m
  EDGE_TIME = 5 #s
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

    start_time = time.perf_counter()

    initial_node = self.create_initial_node(initial_state)
    self.fringe.append(initial_node)

    self.closest_node = initial_node
    self.shortest_distance = 10000

    while time.perf_counter() - start_time < 1:
      while self.generate_goal_node(self.fringe[0], desired_state) == None:
        newNode = self.get_best_node_on_fringe()
        children_list = self.get_children(newNode)
        for child in children_list:
          self.add_to_fringe(child)
      
      new_desired = (self.fringe[0].state[0] + self.EDGE_TIME, desired_state[1], desired_state[2], desired_state[3])
      goalNode = self.generate_goal_node(self.fringe[0], new_desired)

      return self.build_traj(goalNode)
    
    return self.build_traj(closest_node)

  def add_to_fringe(self, node):
    distance_to_shark = self.shark.euclidean_distance_to_state(node.state, shark.state)
    if distance_to_shark < self.shortest_distance:
      self.closest_node = node
      self.shortest_distance = distance_to_shark
      
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

    for i in range(len(self.CHILDREN_DELTAS)):
      CHILDREN_DELTA = self.CHILDREN_DELTAS[i]
      time = node_to_expand.state[0] + self.EDGE_TIME
      x = node_to_expand.state[1] + self.DISTANCE_DELTA * math.cos(node_to_expand.state[3] + CHILDREN_DELTA)
      y = node_to_expand.state[2] + self.DISTANCE_DELTA * math.sin(node_to_expand.state[3] + CHILDREN_DELTA)
      theta = node_to_expand.state[3] + 2 * CHILDREN_DELTA
      
      state = (time, x, y, theta)
      child = self.create_node(state, node_to_expand)
      children_list.append(child)
  
    return children_list

  def generate_goal_node(self, node, desired_state):
    
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

    return traj, total_traj_distance

  def build_shark_traj(self, all_states):
    
    global total_traj_distance
    total_traj_distance = 0
    
    traj = []
    for i in range(1,len(all_states)):
      traj_point_0 = all_states[i-1]
      traj_point_1 = all_states[i]
      traj_point_1 = list(traj_point_1)
      traj_point_1[3] = math.atan2(traj_point_1[2]-traj_point_0[2], traj_point_1[1]-traj_point_0[1])
      traj_point_1 = tuple(traj_point_1)
      print(f"Traj_point0 = {traj_point_0}")
      print(f"Traj_point1 = {traj_point_1}")
      edge_traj, edge_traj_distance = construct_dubins_traj(traj_point_0, traj_point_1)
      
      total_traj_distance += edge_traj_distance
      traj = traj + edge_traj

    return traj, total_traj_distance

  def build_object_traj(self, previous_objects):
    
    global total_traj_distance
    total_traj_distance = 0

    all_trajs = []
    print(f"Here is previous objects: {previous_objects} with len {len(previous_objects)}")
    #print(f"Here is previous_objects[0]: {previous_objects[0]} with len {len(previous_objects[0])}")
    for j in range(len(previous_objects[0])):
      states = []
      for i in range(len(previous_objects)):
        print(f"States before: {states}")
        x = previous_objects[i][j][0]
        y = previous_objects[i][j][1]
        states.append((i, x, y, 0))
        print(f"States after: {states}")
      traj, traj_distance = self.build_shark_traj(states)
      all_trajs.append(traj)
      total_traj_distance += traj_distance

      # print(f"Here are the previous objects: {previous_objects}")
      #print(f'Here are the states: {states}')
    return all_trajs, total_traj_distance

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

  def update_objects(self):
    max_change = 1  # m. Both negative and positive
    #print(f"Here are the objects: {self.objects}")
    for obstacle in self.objects:
      obstacle[0] += random.uniform(-max_change, max_change)
      obstacle[1] += random.uniform(-max_change, max_change)
    
    #print(f"Here are the objects after: {self.objects}")
      

if __name__ == '__main__':

  start_time = time.perf_counter()
  maxR = 15
  tp0 = [0, -2, -2, 0]
  tp1 = []
  planner = A_Star_Planner()
  walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
  shark = Shark((0, 0, 0), walls)
  x, y, theta = shark.get_desired_state(tp0)
  tp1 = [300, x, y, theta]
  TIME_STEP = 1
  num_objects = 7
  objects = []

  for j in range(0, num_objects): 
    obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0.5]
    while (abs(obj[0]-tp0[1]) < 1 and abs(obj[1]-tp0[2]) < 1) or (abs(obj[0]-tp1[1]) < 1 and abs(obj[1]-tp1[2]) < 1):
      obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0.5]
    objects.append(obj)

  #print("Objects:", objects)

  time_in_disk = 0
  total_traj = []
  total_traj_distance = 0
  traj, traj_distance = planner.construct_traj(tp0, tp1, objects, walls, shark)

  total_traj_distance += traj_distance

  for point in traj:
    distance_from_shark = shark.euclidean_distance_to_state(point, shark.state)
    if shark.MAX_DESIRED_RADIUS > distance_from_shark > shark.MIN_DESIRED_RADIUS:
      time_in_disk += 1
  
  total_traj+=traj
  list_of_goal_points = []

  previous_objects = []
  previous_objects.append(objects)

  # Call below repeatedly
  for i in range(5):
    print(f"Objects before: {planner.objects}")
    shark.updateState()
    planner.update_objects()
    object_list = planner.objects
    previous_objects.append(object_list)
    print(f"Objects after: {planner.objects}")
    current_x, current_y, current_theta = shark.state
    current_state = (i, current_x, current_y, current_theta)
    x, y, theta = shark.get_desired_state(current_state)
    tp0 = total_traj[-1]
    tp1 = [tp0[0] + TIME_STEP, x, y, theta]
    traj, traj_distance = planner.construct_traj(tp0, tp1, planner.objects, walls, shark)
    total_traj_distance += traj_distance

    for point in traj:
      distance_from_shark = shark.euclidean_distance_to_state(point, shark.state)
      if shark.MAX_DESIRED_RADIUS > distance_from_shark > shark.MIN_DESIRED_RADIUS:
        time_in_disk += 1

    total_traj+=traj
    list_of_goal_points.append(tp0)

  end_time = time.perf_counter()

  previous_states = shark.previous_states
  x, y, theta = shark.state
  time = shark.previous_states[-1][0] + TIME_STEP
  previous_states.append((time, x, y, theta))

  

  shark_traj, shark_cost = planner.build_shark_traj(previous_states)
 
    
  object_trajs, total_cost = planner.build_object_traj(previous_objects)

  if len(total_traj) > 0:
    print(f"Plan construction time: {end_time - start_time}")
    print(f"Trajectory distance: {total_traj_distance}")
    print(f"Shark trajectory distance: {shark_cost}")
    print(f"The number of time steps with the AUV in the shark disk: {time_in_disk}.")
    print(f"The number of time steps in total: {len(total_traj)}.")
    print(f"The percent of time steps within the disk: {time_in_disk / len(total_traj)}.")
    plot_traj(total_traj, total_traj, objects, walls, shark, list_of_goal_points, shark_traj)
    animationAll(total_traj, shark_traj, object_trajs, shark)
   







  # for i in range(0, 5):
  #   maxR = 10
  #   tp0 = [0, -8, -8, 0]
  #   tp1 = []
  #   planner = A_Star_Planner()
  #   walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
  #   shark = Shark((0, 0, 0), walls)
  #   x, y, theta = shark.get_desired_state()
  #   tp1 = [300, x, y, theta]
  #   # Call below repeatedly
  #   objects = []
  #   num_objects = 15
  #   for j in range(0, num_objects): 
  #     obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0.5]
  #     while (abs(obj[0]-tp0[1]) < 1 and abs(obj[1]-tp0[2]) < 1) or (abs(obj[0]-tp1[1]) < 1 and abs(obj[1]-tp1[2]) < 1):
  #       obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0.5]
  #     objects.append(obj)
    
  #   traj = planner.construct_traj(tp0, tp1, objects, walls, shark)
  #   shark.updateState()
  #   x, y, theta = shark.get_desired_state()
  #   tp1 = [300, x, y, theta]

  #   if len(traj) > 0:
  #     plot_traj(traj, traj, objects, walls, shark)
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
from traj_planner_ExpPlanner import *

class Planning_Problem():
  """ Class that holds a single motion planning problem for one robot.
  """

  def __init__(self, initial_state, desired_state, objects, walls):
    """ Constructor for planning problem.
        - Arguments:
          - initial_state (list of floats): The initial state of the robot (t, x, y, theta).
          - desired_state (list of floats): The desired state of the robot (t, x, y, theta)
          - objects (list of list of floats): The objects to avoid.
          - walls (list of list of floats): The walls to not drive through.
    """
    self.initial_state = initial_state
    self.desired_state = desired_state
    self.objects = objects
    self.walls = walls
    self.planner = Expansive_Planner()
    
  def add_trajs_as_obstacles(self, traj_set):
    """ Function to add trajs as obstacles to the pp.
        - Arguments
          - traj_set (list of list of floats): The trajectories to be treated as obstacles.
    """
    for traj in traj_set:
      self.objects.append(['traj', traj])

class Sequential_Planner():
  
  def __init__(self):
    pass

  def construct_traj_set(self, planning_problem_list):
    """ Function that creates a set of trajectories for several planning problems (1 per robot).
        - Parameters:
          - planning_problem_list (list of Planning_Problems): One pp to solve per robot.
        - Returns:
          - traj_set (list of list of floats): One traj per robot.
    """
    traj_set = []
    # Add code here to create a traj set #
    random_list = planning_problem_list

    while (len(random_list) != 0):
      pp = random.choice(random_list)
      random_list.remove(pp)
      pp.add_trajs_as_obstacles(traj_set)
      best_traj, _ = pp.planner.construct_traj(pp.initial_state, pp.desired_state, pp.objects, pp.walls)
      traj_set.append(best_traj)

    return traj_set

      # Sequential:
      # for pp in planning_problem_list:
      #   pp.add_trajs_as_obstacles(traj_set)
      #   best_traj, _ = pp.planner.construct_traj(pp.initial_state, pp.desired_state, pp.objects, pp.walls)
      #   traj_set.append(best_traj)
      # return traj_set

def random_pose(maxR):
  """ Function to generate random pose (x, y, theta) within bounds +/- maxR.
      Parameters:
        - maxR (float): the half width of the square environment in meters.
      Returns:
        - random_pose (list of floats): The x, y, theta in (m, m, rad).
  """
  return [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), random.uniform(-math.pi, math.pi)]
  
def get_new_random_pose(pose_list, maxR, radius):
  """ Function to generate random pose (x, y, theta) within bounds +/- maxR.
      Parameters:
        - pose_list (list of floats):
        - maxR (float): the half width of the square environment in meters.
        - radius (float): The objects' radius in meters.
      Returns:
        - random_pose (list of floats): The x, y, theta, radius in (m, m, rad, m).
  """
  new_pose_found = False
  while not new_pose_found:
    new_pose = random_pose(maxR)
    new_pose_found = True
    for pl in pose_list:
      effective_radius = radius + pl[3]
      if (abs(pl[0]-new_pose[1]) < effective_radius and abs(pl[1]-new_pose[2]) < effective_radius) or (abs(pl[0]-new_pose[1]) < effective_radius and abs(pl[1]-new_pose[2]) < effective_radius):
        new_state_found = False
  
  return new_pose + [radius]

if __name__ == '__main__':
  totalTime = 0
  TIME_BUDGET = 10
  num_robots = 5
  num_objects = 2
  maxR = 10
  obj_vel = 2.0   # object velocity
  numTimes = 0

  start_time = time.perf_counter()
  current_time = time.perf_counter()

  while (current_time - start_time) < TIME_BUDGET:
    print(f'Your time budget is: {TIME_BUDGET}')
    initial_time = time.perf_counter()
    robot_initial_pose_list = []
    robot_initial_state_list = []
    for _ in range(num_robots):
      ns = get_new_random_pose(robot_initial_pose_list, maxR, ROBOT_RADIUS)
      robot_initial_pose_list += [ns]
      robot_initial_state_list += [[0, ns[0], ns[1], ns[2]]]
    
    robot_desired_pose_list = []
    robot_desired_state_list = []
    for _ in range(num_robots):
      ns = get_new_random_pose(robot_desired_pose_list, maxR, ROBOT_RADIUS)
      robot_desired_pose_list += [ns]
      robot_desired_state_list += [[15, ns[0], ns[1], ns[2]]] # fixed going back in time error 

    object_list = []
    for _ in range(num_objects):
      object_radius = random.uniform(0.3, 1.0)
      obj_pose = get_new_random_pose(robot_initial_pose_list + robot_desired_pose_list, maxR, object_radius)
      obj_yaw = obj_pose[2]
      object_list += [ ['obstacle',[obj_pose[0], obj_pose[1], 0.5, obj_vel, obj_yaw]] ]
      
    walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
      
    planning_problem_list = []
    for i in range(num_robots):
      pp = Planning_Problem(robot_initial_state_list[i], robot_desired_state_list[i], object_list, walls)
      planning_problem_list.append(pp)
    
    planner = Sequential_Planner()
    traj_list = planner.construct_traj_set(planning_problem_list)

    if len(traj_list) > 0 and [] not in traj_list:
      current_time = time.perf_counter()
      totalTime += current_time - initial_time
      numTimes += 1

    current_time = time.perf_counter()

  print(totalTime/numTimes)
      #plot_traj_list(traj_list, object_list, walls)


#  start_time = time.perf_counter()
#     current_time = start_time
#     best_traj = []
#     best_traj_cost = self.LARGE_NUMBER
#     TIME_BUDGET = 1 #In seconds
#     print("The time budget is: " + str(TIME_BUDGET))
#     while (current_time - start_time) < TIME_BUDGET:
#       traj, traj_cost = self.construct_traj(initial_state, desired_state, objects, walls)
#       if traj_cost < best_traj_cost:
#         best_traj_cost = traj_cost
#         best_traj = traj
#       current_time = time.perf_counter()
#     return best_traj, best_traj_cost
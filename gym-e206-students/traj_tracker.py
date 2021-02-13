import time
import math
import random
from traj_planner_utils import *

TIME_STEP_SIZE = 0.01 #s
LOOK_AHEAD_TIME = 1.0 #s
MIN_DIST_TO_POINT = 0.1 #m
MIN_ANG_TO_POINT = 0.50 #rad

class TrajectoryTracker():
  """ A class to hold the functionality for tracking trajectories.
      Arguments:
        traj (list of lists): A list of traj points Time, X, Y, Theta (s, m, m, rad).
  """
  current_point_to_track = 0
  traj_tracked = False
  traj = []


  def __init__(self, traj):
    self.current_point_to_track = 0
    self.traj = traj
    self.traj_tracked = False
      
  def get_traj_point_to_track(self, current_state):
    """ Determine which point of the traj should be tracked at the current time.
        Arguments:
          current_state (list of floats): The current Time, X, Y, Theta (s, m, m, rad).
        Returns:
          desired_state (list of floats: The desired state to track - Time, X, Y, Theta (s, m, m, rad).
    """
    current_time = current_state[0]
    
    for point in self.traj:
      if current_time + TIME_STEP_SIZE == point[0]:
        self.current_point_to_track = point
        break
    
    return self.traj[self.current_point_to_track]
  
  def print_traj(self):
    """ Print the trajectory points.
    """
    print("Traj:")
    for i in range(len(self.traj)):
        print(i,self.traj[i])
          
  def is_traj_tracked(self):
    """ Return true if the traj is tracked.
        Returns:
          traj_tracked (boolean): True if traj has been tracked.
    """
    return self.traj_tracked
    
class PointTracker():
  """ A class to determine actions (motor control signals) for driving a robot to a position.
  """
  def __init__(self):
    pass

  def get_dummy_action(self, x_des, x):
    """ Return a dummy action for now
    """
    action = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
    return action

  def point_tracking_control(self, desired_state, current_state):
    """ Return the motor control signals as actions to drive a robot to a desired configuration
        Arguments:
          desired_state (list of floats): The desired Time, X, Y, Theta (s, m, m, rad).
          current_state (list of floats): The current Time, X, Y, Theta (s, m, m, rad).
    """
    # In order to be stable, our proportional constants should follow:
    # kp > 0
    # kb < 0
    # ka - kp > 0

    kp = 10
    kb = -3
    ka = 15
    k_rots= 2
    
    # Constants to get velocities and torques
    #ROBOT_BODY_LENGTH = 1  # 
    L = 0.35

    # Extract theta, x, y
    theta = current_state[3]
    x = current_state[1]
    y = current_state[2]

    # Extract the desired theta, x, y
    theta_des = desired_state[3]
    x_des = desired_state[1]
    y_des = desired_state[2]
    
    # Covert Cartesian coordinates to polar coordinates
    p = math.sqrt((x_des-x)**2 + (y_des-y)**2)
    a = angle_diff(-theta + math.atan2((y_des-y), (x_des-x)))
    b = angle_diff(angle_diff(-theta - a) + theta_des)

    # Check to see if we are within tolerance of point being tracked
    if (abs(a) <= MIN_ANG_TO_POINT) and (abs(p) <= MIN_DIST_TO_POINT):
      self.traj_tracked = True

  
    if p > 0.1:
      # Use the control law to get forward velocity and angular velocity of vehicle
      if a >= -math.pi/2 and a <= math.pi/2: # how are we declaring a in the else statement if we are using a as the iF???
        v = kp * p
        w = (ka * a) + (kb * b)
      else:
        a = angle_diff(-theta + math.atan2(-(y_des-y), -(x_des-x)))
        b = angle_diff(angle_diff(-theta - a) - theta_des)
        v = -kp * p
        w = (ka * a) + (kb * b)
    else:
      v = 0
      w = k_rots * (theta_des-theta_des)

    # w = -(1.57*10)/5
    # v = 7.0
    # print(current_state)

    right_wheel_torque = w*L + v
    left_wheel_torque = -w*L + v

    # zero all of action
    action = [right_wheel_torque, left_wheel_torque, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
    
    return action
  
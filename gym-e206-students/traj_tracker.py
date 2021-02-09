import time
import math
import random
from traj_planner_utils import *

TIME_STEP_SIZE = 0.01 #s
LOOK_AHEAD_TIME = 1.0 #s
MIN_DIST_TO_POINT = 0.1 #m
MIN_ANG_TO_POINT = 0.10 #rad

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
    self.current_point_to_track = 0

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

    kp = 1
    kb = -1
    ka = 2
    
    # Constants to get velocities and torques
    ROBOT_BODY_LENGTH = 1  # In m
    MAGIC_CONSTANT = 1  # Used to convert to torque, will get sucked up in gain

    # Extract theta, x, y
    theta = current_state[3]
    x = current_state[1]
    y = current_state[2]

    # Extract the desired theta, x, y
    theta_des = desired_state[3]
    x_des = desired_state[1]
    y_des = desired_state[2]
    
    # Covert Cartesian coordinates to polar coordinates
    p = math.sqrt(x**2 + y**2)
    a = -theta + math.atan2(y, x)
    b = -theta - a + theta_des
    p_des = math.sqrt(x_des**2 + y_des**2)

    # Check to see if we are within tolerance of point being tracked
    if abs(theta_des - theta) <= MIN_ANG_TO_POINT and abs(p_des - p) <= MIN_DIST_TO_POINT:
      self.traj_tracked = True

    # Use the control law to get forward velocity and angular velocity of vehicle
    if a >= -math.pi/2 or a <= math.pi/2:
      v = kp * p
      w = ka * a + kb * b
    else:
      a = -theta + math.atan2(-y, -x)
      b = -theta - a - theta_des
      v = -kp * p
      w = ka * a + kb * b

    # Get the angular velocity of the individual wheels
    angVelLeft = 0.5*(w - (v/ROBOT_BODY_LENGTH))
    angVelRight = w - angVelLeft
    
    # Get torques for each wheel
    leftTorque = angVelLeft * MAGIC_CONSTANT
    rightTorque = angVelRight * MAGIC_CONSTANT

    # zero all of action
    action = [leftTorque, rightTorque, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
    
    return action

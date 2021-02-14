import gym
import gym_fetch
import time
import math
import random
from traj_planner_utils import *
from traj_tracker import *

    
def main():

  # Run this for many points to point track individually
  if False:
    states = [([0, 0, 0, 0],[10, 2, 0, 0]), ([0, 0, 0, 0], [10, 2, 2, 0]), ([0, 0, 0, 0],[10, 0, 2, math.pi/2]), 
    ([0, 0, 0, 0], [10, -2, 2, 0]), ([0, 0, 0, 0], [10, -2, 0, 0]), ([0, 0, 0, 0], [10, -2, -2, 0]), ([0, 0, 0, 0], [10, 0, -2, -math.pi/2]),
    ([0, 0, 0, 0], [10, 2, -2, 0])]

    maxR = 8
    walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
    objects = [[4, 0, 1.0], [-2, -3, 1.5]]

    # Construct an environment
    env = gym.make("fetch-v0") # <-- this we need to create
    env.set_parameters(TIME_STEP_SIZE, objects)
    env.render('human')
    env.reset()

    desired_traj_list = []
    actual_traj_list = []

    controller = PointTracker()

    for state in states:
      print(state)
      current_state = state[0]
      desired_state = state[1]
      desired_traj = [desired_state]
      
      controller.reset()

      traj_tracker = TrajectoryTracker(desired_traj)

      time.sleep(1)
      current_time_stamp = 0
      observation = [0,0,0,0,0]
      desired_traj_list.append(desired_state)
      while not controller.is_Done():
          #write some logic for is tracked to help to go from point to point for our plot
          current_state = [current_time_stamp, observation[0], observation[1], observation[2]]
          desired_state = traj_tracker.get_traj_point_to_track(current_state)
          action = controller.point_tracking_control(desired_state, current_state)
          observation, reward, done, dummy = env.step(action)
          env.render('human')
          actual_traj_list.append(current_state)
          current_time_stamp += TIME_STEP_SIZE
      time.sleep(2)
      env.reset()

    print(desired_traj_list)
    plot_traj(desired_traj, actual_traj_list, objects, walls)
    env.close()


  # Run this for Trajectories 

  if True:
    # Create a motion planning problem and solve it
    current_state, desired_state, objects, walls = create_motion_planning_problem()
    desired_traj = construct_dubins_traj(current_state, desired_state) #as in Lab00
    #desired_traj = [desired_state]
    #plot_traj(desired_traj, [current_state], objects, walls) ##LAB00

    # Construct an environment
    env = gym.make("fetch-v0") # <-- this we need to create
    env.set_parameters(TIME_STEP_SIZE, objects)
    env.render('human')
    env.reset()

    # Create the trajectory and tracking controller
    controller = PointTracker()
    traj_tracker = TrajectoryTracker(desired_traj)
        
    # Create the feedback loop
    time.sleep(1)
    current_time_stamp = 0
    observation = [0,0,0,0,0]
    actual_traj = []
    while not controller.is_Done():
        #write some logic for is tracked to help to go from point to point for our plot
        current_state = [current_time_stamp, observation[0], observation[1], observation[2]]
        desired_state = traj_tracker.get_traj_point_to_track(current_state)
        action = controller.point_tracking_control(desired_state, current_state)
        observation, reward, done, dummy = env.step(action)
        env.render('human')
        actual_traj.append(current_state)
        current_time_stamp += TIME_STEP_SIZE
    time.sleep(2)
    print("desired:", desired_traj)
    print("actual:", actual_traj)
    env.close()
    plot_traj(desired_traj, actual_traj, objects, walls)
    

    # Calculate average mean errors
    x_error = 0
    y_error = 0
    theta_error = 0
    desired_index = 0

    for i in range(len(actual_traj)):
      
      if desired_index >= len(desired_traj):
        desired_state = desired_traj[-1]
      else:
        desired_point = desired_traj[desired_index]

      actual_point = actual_traj[i]

      if desired_point[0] < actual_point[0]:
        desired_index += 1

      x_error += abs(desired_point[1] - actual_point[1])
      y_error += abs(desired_point[2] - actual_point[2])
      theta_error += abs(desired_point[3] - actual_point[3])
    
    x_error = x_error / len(actual_traj)
    y_error = y_error / len(actual_traj)
    theta_error = theta_error / len(actual_traj)

    print("x_error:", x_error)
    print("y_error:", y_error)
    print("theta_error:", theta_error)



  
def create_motion_planning_problem():
  current_state =  [0, 0, 0, 0]
  desired_state =  [20, 5, 3, 0] #[20, 2.5, -2.5, 1.57]
  maxR = 8
  walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
  objects = [[4, 0, 1.0], [-2, -3, 1.5]]
  
  return current_state, desired_state, objects, walls

if __name__ == '__main__':
    main()
    
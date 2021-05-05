# E206 Motion Planning

# Simple planner
# C Clark

import numpy as np
import matplotlib.animation as animation

import math
import dubins
import matplotlib.pyplot as plt

DISTANCE_STEP_SIZE = 0.1 #m
COLLISION_INDEX_STEP_SIZE = 5
ROBOT_RADIUS = 0.4 #m

def construct_dubins_traj(traj_point_0, traj_point_1):
  """ Construct a trajectory in the X-Y space and in the time-X,Y,Theta space.
      Arguments:
        traj_point_0 (list of floats): The trajectory's first trajectory point with time, X, Y, Theta (s, m, m, rad).
        traj_point_1 (list of floats): The trajectory's last trajectory point with time, X, Y, Theta (s, m, m, rad).
      Returns:
        trajectory (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
        traj_distance (float): The length ofthe trajectory (m).
  """
  
  traj = []
  traj_distance = 0
  
  startTime = traj_point_0[0]
  endTime = traj_point_1[0]

  q0 = (traj_point_0[1], traj_point_0[2], traj_point_0[3])
  q1 = (traj_point_1[1], traj_point_1[2], traj_point_1[3])
  turning_radius = 0.75               
  step_size = DISTANCE_STEP_SIZE

  path = dubins.shortest_path(q0, q1, turning_radius)
  configurations, _ = path.sample_many(step_size) 

  units = (endTime-startTime)/(len(configurations) - 1) 
  listOfTimes = [(t * units) + startTime for t in range(len(configurations))]

  for i in range(len(configurations) - 1):
    tup = configurations[i]
    traj_point = (listOfTimes[i], tup[0], tup[1], tup[2])
    traj.append(traj_point)

    nextPoint = configurations[i+1]
    traj_distance += math.sqrt((nextPoint[0] - tup[0])**2 + (nextPoint[1] - tup[1])**2)

  traj.append((listOfTimes[-1], configurations[-1][0], configurations[-1][1], configurations[-1][2]))
  return traj, traj_distance


def plot_traj(traj_desired, traj_actual, objects, walls, shark, goal_points, shark_traj):
  """ Plot a trajectory in the X-Y space and in the time-X,Y,Theta space.
      Arguments:
        desired_traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
        actual_traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
        objects (list of lists): A list of stationay object states with X, Y, radius (m, m, m).
        walls (list of lists: A list of walls with corners X1, Y1 and X2, Y2 points, length (m, m, m, m, m).
  """
  fig, axis_array = plt.subplots(2,1)
  time_stamp_desired = []
  x_desired = []
  y_desired = []
  theta_desired = []
  for tp in traj_desired:
    time_stamp_desired.append(tp[0])
    x_desired.append(tp[1])
    y_desired.append(tp[2])
    theta_desired.append(angle_diff(tp[3]))
  axis_array[0].plot(x_desired, y_desired, 'b')
  axis_array[0].plot(x_desired[0], y_desired[0], 'ko')
  axis_array[0].plot(x_desired[-1], y_desired[-1], 'kx')
  time_stamp_actual = []
  x_actual = []
  y_actual = []
  theta_actual = []
  for tp in traj_actual:
    time_stamp_actual.append(tp[0])
    x_actual.append(tp[1])
    y_actual.append(tp[2])
    theta_actual.append(angle_diff(tp[3]))
  axis_array[0].plot(x_actual, y_actual, 'k')

  time_stamp_shark = []
  x_shark = []
  y_shark = []
  theta_shark = []
  for tp in shark_traj:
    time_stamp_shark.append(tp[0])
    x_shark.append(tp[1])
    y_shark.append(tp[2])
    theta_shark.append(angle_diff(tp[3]))
  axis_array[0].plot(x_shark, y_shark, 'g')
  axis_array[0].plot(x_shark[0], y_shark[0], 'ro')
  axis_array[0].plot(x_shark[-1], y_shark[-1], 'rx')

  ang_res = 0.2
  for o in objects:
    x_obj = []
    y_obj = []
    ang = 0
    while ang < 6.28:
      x_obj.append(o[0]+o[2]*math.cos(ang))
      y_obj.append(o[1]+o[2]*math.sin(ang))
      ang += ang_res
    x_obj.append(x_obj[0])
    y_obj.append(y_obj[0])
    axis_array[0].plot(x_obj, y_obj, 'b')

  # For the shark
  # shark_x = []
  # shark_y = []
  # shark_ang = 0
  # while shark_ang < 6.28:
  #     shark_x.append(shark.state[0] + shark.DESIRED_STATE_RADIUS * math.cos(ang))
  #     shark_y.append(shark.state[1] + shark.DESIRED_STATE_RADIUS * math.sin(ang))
  #     shark_ang += ang_res
  # shark_x.append(shark_x[0])
  # shark_y.append(shark_y[0])
  # axis_array[0].plot(shark_x, shark_y, 'r')
  

  # Create shark + disk

  circle3 = plt.Circle( (shark.state[0], shark.state[1]), radius = shark.MAX_DESIRED_RADIUS, fill=False, ec='r')
  axis_array[0].add_patch( circle3 )

  circle2 = plt.Circle( (shark.state[0], shark.state[1]), radius = shark.MIN_DESIRED_RADIUS, fill=False, ec='y')
  axis_array[0].add_patch( circle2 )

  circle1 = plt.Circle( (shark.state[0], shark.state[1]), radius = 0.5, ec='g')
  axis_array[0].add_patch( circle1 )

  # Create x's on each goal point
  for goal in goal_points:
    axis_array[0].plot(goal[1], goal[2], 'kx')


  # Create trajectory line
  x_y_start = (shark.state[0], shark.state[1])
  theta = shark.state[2]
  x_new = shark.state[0] + shark.MIN_DESIRED_RADIUS * math.cos(theta)
  y_new = shark.state[1] + shark.MIN_DESIRED_RADIUS * math.sin(theta)
  x_y_end = (x_new, y_new)

  x_list = [x_y_start[0], x_y_end[0]] 
  y_list = [x_y_start[1], x_y_end[1]] 

  line = plt.Line2D(x_list, y_list, lw=2)
  axis_array[0].add_line(line)

  for w in walls:
    axis_array[0].plot([w[0], w[2]], [w[1], w[3]], 'k')
  axis_array[0].set_xlabel('X (m)')
  axis_array[0].set_ylabel('Y (m)')
  axis_array[0].axis('equal')

  
  
  axis_array[1].plot(time_stamp_desired, x_desired,'b')
  axis_array[1].plot(time_stamp_desired, y_desired,'b--')
  axis_array[1].plot(time_stamp_desired, theta_desired,'b-.')
  axis_array[1].plot(time_stamp_actual, x_actual,'k')
  axis_array[1].plot(time_stamp_actual, y_actual,'k--')
  axis_array[1].plot(time_stamp_actual, theta_actual,'k-.')
  axis_array[1].set_xlabel('Time (s)')
  axis_array[1].legend(['X Desired (m)', 'Y Desired (m)', 'Theta Desired (rad)', 'X (m)', 'Y (m)', 'Theta (rad)'])


  plt.show()
  
def collision_found(traj, objects, walls):
  """ Return true if there is a collision with the traj and the workspace
      Arguments:
        traj (list of lists): A list of traj points - Time, X, Y, Theta (s, m, m, rad).
        objects (list of lists): A list of object states - X, Y, radius (m, m, m).
        walls (list of lists): A list of walls defined by end points - X0, Y0, X1, Y1, length (m, m, m, m, m).
      Returns:
        collision_found (boolean): True if there is a collision.
  """
  index = 0
  while index < len(traj):
    traj_point = traj[index]
    for obj in objects:
      obj_distance = generate_distance_to_object(traj_point, obj) - obj[2] - ROBOT_RADIUS
      if obj_distance < 0:
        return True
    for wall in walls:
      wall_distance = generate_distance_to_wall(traj_point, wall) - ROBOT_RADIUS
      if wall_distance < 0:
        return True
    
    index += COLLISION_INDEX_STEP_SIZE
  
  return False
  
def generate_distance_to_object(traj_point, obj):
  """ Calculate the deistance between a spherical object and a cylindrical robot.
      Argument:
        traj_point (list of floats): A state of Time, X, Y, Theta (s, m, m, rad).
        obj (list of floats): An object state X, Y, radius (m, m, m).
      Returns:
        distance (float): The distance between a traj point and an object (m).
  """
  return math.sqrt( pow(traj_point[1]-obj[0],2) + pow(traj_point[2]-obj[1],2) )
  
def generate_distance_to_wall(traj_point, wall):
  """ Calculate the deistance between a spherical object and a cylindrical robot.
      Argument:
        traj_point (list of floats): A state of Time, X, Y, Theta (s, m, m, rad).
        wall (list of floats): An wall state X0, Y0, X1, Y1, length (m, m, m, m, m).
      Returns:
        distance (float): The distance between a traj point and an object (m).
  """
  x0 = traj_point[1]
  y0 = traj_point[2]
  x1 = wall[0]
  y1 = wall[1]
  x2 = wall[2]
  y2 = wall[3]
  num = abs( (x2-x1)*(y1-y0) - (x1-x0)*(y2-y1) )
  den = wall[4]
  
  return num/den
  
def print_traj(traj):
  """ Print a trajectory as a list of traj points.
      Arguments:
        traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
  """
  print("TRAJECTORY")
  for tp in traj:
    print("traj point - time:",tp[0], "x:", tp[1], "y:", tp[2], "theta:", tp[3] )
    
def angle_diff(ang):
  """ Function to push ang within the range of -pi and pi
      Arguments:
        ang (float): An angle (rad).
      Returns:
        ang (float): The angle, but bounded within -pi and pi (rad).
  """
  while ang > math.pi:
    ang -= 2*math.pi
  while ang < -math.pi:
    ang += 2*math.pi

  return ang
  
#################################################################
# animation function 


def animationAll(total_traj, shark_traj, objects, shark):
  # plt.style.use('dark_background')
  traj = total_traj
  fig = plt.figure() 
  fig.set_dpi(100)
  fig.set_size_inches(7, 6.5)
  ax = plt.axes(xlim=(-15, 15), ylim=(-15, 15)) 
  line, = ax.plot([], [], lw=2) 
  line_shark, = ax.plot([], [], lw=2)

  circle2 = plt.Circle( (shark.state[0], shark.state[1]), radius = shark.MIN_DESIRED_RADIUS, fill=False, ec='y')
  circle3 = plt.Circle( (shark.state[0], shark.state[1]), radius = shark.MAX_DESIRED_RADIUS, fill=False, ec='r')
  
  print(objects)
  objcir1 = plt.Circle( (objects[0][0][1], objects[0][0][2]), radius = 0.5, fill=False, ec='g')
  objcir2 = plt.Circle( (objects[1][0][1], objects[1][0][2]), radius = 0.5, fill=False, ec='g')
  objcir3 = plt.Circle( (objects[2][0][1], objects[2][0][2]), radius = 0.5, fill=False, ec='g')
  objcir4 = plt.Circle( (objects[3][0][1], objects[3][0][2]), radius = 0.5, fill=False, ec='g')
  objcir5 = plt.Circle( (objects[4][0][1], objects[4][0][2]), radius = 0.5, fill=False, ec='g')
  objcir6 = plt.Circle( (objects[5][0][1], objects[5][0][2]), radius = 0.5, fill=False, ec='g')
  objcir7 = plt.Circle( (objects[6][0][1], objects[6][0][2]), radius = 0.5, fill=False, ec='g')
  


  # lists to store x and y axis points 
  xdata, ydata = [], [] 
  xsharkdata, ysharkdata = [], [] 
  # cirxdata, cirydata = [], [] 
  # cir3xdata, cir3ydata = [], [] 
  multiple = len(traj)//len(shark_traj)
  
  difference = len(total_traj) - len(shark_traj) * multiple
  
  def init(): 
    # creating an empty plot/frame 
    line.set_data([], []) 
    line_shark.set_data([], [])

    x, y, _ = shark.state

    circle2.center = (x, y)
    circle3.center = (x, y)

    ####OBJECTs######            
    # 1
    objcir1.center = (objects[0][0][1], objects[0][0][2])
    ax.add_patch(objcir1)
    #2
    objcir2.center = (objects[1][0][1], objects[1][0][2])
    ax.add_patch(objcir2)
    #3
    objcir3.center = (objects[2][0][1], objects[2][0][2])
    ax.add_patch(objcir3)
    #4
    objcir4.center = (objects[3][0][1], objects[3][0][2])
    ax.add_patch(objcir4)
    #5
    objcir5.center = (objects[4][0][1], objects[4][0][2])
    ax.add_patch(objcir5)
    #6
    objcir6.center = (objects[5][0][1], objects[5][0][2])
    ax.add_patch(objcir6)
    #7
    objcir7.center = (objects[6][0][1], objects[6][0][2])
    ax.add_patch(objcir7)

    ax.add_patch(circle2)
    ax.add_patch(circle3)
    return line, line_shark, circle2, circle3, objcir1

  def animate(i): 
    # t is a parameter 
    t = i
    cirx, ciry = circle2.center
    cir3x, cir3y = circle3.center

    ####OBJECTS######
    #1-7
    objcir1x, objcir1y = objcir1.center
    objcir2x, objcir1y = objcir2.center
    objcir3x, objcir1y = objcir3.center
    objcir4x, objcir1y = objcir4.center
    objcir5x, objcir1y = objcir5.center
    objcir6x, objcir1y = objcir6.center
    objcir7x, objcir1y = objcir7.center
    
    # x, y values to be plotted 
    step = t * multiple

    for j in range(multiple):
      x = traj[step][1]
      y = traj[step][2]
      xdata.append(x)
      ydata.append(y)
      step += 1

    # x, y values to be plotted 
    # if t <= difference:
    #   xshark = shark_traj[0][1]
    #   yshark = shark_traj[0][2]

    #   cirx = shark_traj[0][1]
    #   ciry = shark_traj[0][2]
    #   cir3x = shark_traj[0][1]
    #   cir3y = shark_traj[0][2]

    if (t < len(shark_traj)):  #and t > difference
      xshark = shark_traj[t][1]
      yshark = shark_traj[t][2]

      cirx = shark_traj[t][1]
      ciry = shark_traj[t][2]
      cir3x = shark_traj[t][1]
      cir3y = shark_traj[t][2]
      ####OBJECTS######             WRONG
      #1-7 
      objcir1x = objects[0][t][1]
      objcir1y = objects[0][t][2]
      objcir2x = objects[1][t][1]
      objcir2y = objects[1][t][2]
      objcir3x = objects[2][t][1]
      objcir3y = objects[2][t][2]
      objcir4x = objects[3][t][1]
      objcir4y = objects[3][t][2]
      objcir5x = objects[4][t][1]
      objcir5y = objects[4][t][2]
      objcir6x = objects[5][t][1]
      objcir6y = objects[5][t][2]
      objcir7x = objects[6][t][1]
      objcir7y = objects[6][t][2]
      
    else:
      xshark = shark_traj[-1][1]
      yshark = shark_traj[-1][2]
      
      cirx = shark_traj[-1][1]
      ciry = shark_traj[-1][2]
      cir3x = shark_traj[-1][1]
      cir3y = shark_traj[-1][2]
      ####OBJECT######                
      #1
      objcir1x = objects[0][-1][1]
      objcir1y = objects[0][-1][2]
      objcir2x = objects[1][-1][1]
      objcir2y = objects[1][-1][2]
      objcir3x = objects[2][-1][1]
      objcir3y = objects[2][-1][2]
      objcir4x = objects[3][-1][1]
      objcir4y = objects[3][-1][2]
      objcir5x = objects[4][-1][1]
      objcir5y = objects[4][-1][2]
      objcir6x = objects[5][-1][1]
      objcir6y = objects[5][-1][2]
      objcir7x = objects[6][-1][1]
      objcir7y = objects[6][-1][2]


    # appending new points to x, y axes points list  
    line.set_data(xdata, ydata)

    #appending new points to x y shark traj
    xsharkdata.append(xshark)
    ysharkdata.append(yshark)
    
    line_shark.set_data(xsharkdata, ysharkdata) 
    circle2.center = (cirx, ciry)
    circle3.center = (cir3x, cir3y)

    ####OBJECT######
    #1
    objcir1.center = (objcir1x, objcir1y)
    objcir2.center = (objcir2x, objcir2y)
    objcir3.center = (objcir3x, objcir3y)
    objcir4.center = (objcir4x, objcir4y)
    objcir5.center = (objcir5x, objcir5y)
    objcir6.center = (objcir6x, objcir6y)
    objcir7.center = (objcir7x, objcir7y)

    
    return line, line_shark, circle2, circle3, objcir1, objcir2, objcir3, objcir4, objcir5, objcir6, objcir7

  # initialization function 
  
  
  # setting a title for the plot 
  plt.title('Plotting AUV Traj!') 
  # hiding the axis details 
  # plt.axis('off') 

  # call the animator	 
  anim = animation.FuncAnimation(fig, animate, init_func=init, 
                frames=len(total_traj), interval=40, blit=True) 
  
  
  plt.show()

  # save the animation as mp4 video file 
  # anim.save('coil.gif',writer='imagemagick')
#################################################################
  
if __name__ == '__main__':
  tp0 = [0,0,0,0]
  tp1 = [10,4,-4, -1.57]
  traj = construct_dubins_traj(tp0, tp1)
  maxR = 8
  walls = [[-maxR, maxR, maxR, maxR], [maxR, maxR, maxR, -maxR], [maxR, -maxR, -maxR, -maxR], [-maxR, -maxR, -maxR, maxR] ]
  objects = [[4, 0, 1.0], [-2, -3, 1.5]]
  plot_traj(traj, objects, walls)


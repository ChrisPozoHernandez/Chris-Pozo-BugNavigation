import time
import numpy as np
from mbot_bridge.api import MBot

robot = MBot()
# TODO: Declare any other variables you might need here.
state = "GO_TO_GOAL"
speed = 0.08
setpoint = 0.4

#find the nearest wall
def find_min_dist(ranges, thetas):
  min_dist = None
  min_angle = None

  for i in range(len(ranges)):
    if ranges [i] > 0:
      if min_dist is None or ranges[i] < min_dist:
        min_dist = ranges[i]
        min_angle = thestas[i]

  return min_dist, min_angle

#find the difference between two angles
def angle_difference(a, b):
  return np.arctan2(np.sin(a - b), np.cos(a - b))

# Reset the robot odometry to zero at the beginning of the run.
robot.reset_odometry()

# TODO: (P2.4) Ask the user for a goal pose (x and y position and angle).
goal_x = float(input("Enter goal x: "))
goal_y = float(input("Enter goal y: "))
goal_theta = float(input("Enter goal theta in radians: "))

start_x, start_y, _ = robot.read_odometry()
hit_distance = None
left_hit_point = False

"""
TODO: (P2.3 & P2.4) Write code to drive to the given goal while avoiding
obstacles here.

HINT: You may reuse your code and util functions from wall follower and hit the
spot wherever you need to.
"""
try:
  while True:
    #this is to read robots position and lidar
    x, y, theta = robot.read_odometry()
    ranges, thetas = robot.read_lidar()

    dx = goal_x - x
    dy = goal_y - y
    distance = np.sqrt(dx**2 + dy**2)
    
    #check if the robot reached teh goal
    angle_error = angle_difference(goal_theta, theta)

    if distance < 0.05:
      if state != "GOAL_REACHED":
        state = "GOAL_REACHED"
        print("State:", state)

      if abs(angle_error) < 0.1:
        break

      wz = np.clip(0.5 * angle_error, -0.2, 0.2)
      robot.drive(0, 0, float(wz))
      time.sleep(0.1)
      continue

    #Check for obstacles toward the goal
    goal_angle = angle_difference(np.arctan2(dy, dx), theta)
    obstacle_ahead = False

    for i in range(len(ranges)):
      if ranges[i] > 0:
        angle = angle_difference(thetas[i], goal_angle)
        if abs(angle) < 0.35 and ranges[i] < 0.5:
          obstacle_ahead = True
          break

    #Change to wall following
    if state == "GO_TO_GOAL" and obstacle_ahead:
      state = "FOLLOW_WALL"
      hit_distance = distance
      left_hit_point = False
      print("State:", state)

    #check the distance from the start-to-goal line
    line_dx = goal_x - start_x
    line_dy = goal_y - start_y
    line_length = np.sqrt(line_dx**2 + line_dy**2)
    
    if line_length > 0:
      line_error = abs(
        line_dy * (x - start_x) -
        line_dx * (y - start_y)
      ) / line_length
    else:
      line_error = 0

    #return to the goal when the path is clear
    if state == "FOLLOW_WALL":
      if line_error > 0.15:
        left_hit_point = True

      if (left_hit_point and line_error < 0.08
              and distance < hit_distance - 0.15
              and not obstacle_ahead):
        state = "GO_TO_GOAL"
        print("State:", state) 

# Stop the robot before exiting.
robot.stop()

# TODO: Print out the robot's final odometry before exiting.

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

# Reset the robot odometry to zero at the beginning of the run.
robot.reset_odometry()

# TODO: (P2.4) Ask the user for a goal pose (x and y position and angle).

"""
TODO: (P2.3 & P2.4) Write code to drive to the given goal while avoiding
obstacles here.

HINT: You may reuse your code and util functions from wall follower and hit the
spot wherever you need to.
"""

# Stop the robot before exiting.
robot.stop()

# TODO: Print out the robot's final odometry before exiting.

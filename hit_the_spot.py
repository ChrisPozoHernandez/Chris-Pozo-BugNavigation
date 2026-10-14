import time
import numpy as np
from mbot_bridge.api import MBot


def drive_to_pose(goal_x, goal_y, goal_theta, robot):
    """TODO: (P2.1) Implement your code to drive to the goal pose,
    [goal_x, goal_y, goal_theta].

    HINT: Use robot.read_odometry() to get a list of length 3 containing the
    robot's odometry [x, y, theta]. The odometry frame is the pose the robot
    is in at the start of this script."""
    pass


robot = MBot()
# TODO: Declare any other variables you might need here.

# Reset the robot odometry to zero at the beginning of the run.
robot.reset_odometry()

"""
TODO: (P2.1) Call your function drive_to_pose() to drive to a sequence of
goal points.

HINT: If you want your robot to stop for a while after reaching a goal,
you might find the function time.sleep(secs) useful.

HINT: You should use the drive_to_pose() function you will write, passing
it the target x, y and theta, and the robot object. For example, to go
to position (2, 3) with angle 0, do:

  drive_to_pose(2, 3, 0, robot);

"""

# Stop the robot before exiting.
robot.stop()

# TODO: Print out the robot's final odometry before exiting.

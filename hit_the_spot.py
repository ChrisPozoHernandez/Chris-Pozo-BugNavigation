import time
import numpy as np
from mbot_bridge.api import MBot


def drive_to_pose(goal_x, goal_y, goal_theta, robot):
    """TODO: (P2.1) Implement your code to drive to the goal pose,
    [goal_x, goal_y, goal_theta].

    HINT: Use robot.read_odometry() to get a list of length 3 containing the
    robot's odometry [x, y, theta]. The odometry frame is the pose the robot
    is in at the start of this script."""
    #keep moving until the robot reaches the goal
    while True:
        x, y, theta = robot.read_odometry()

        #find the distance to the goal
        dx = goal_x - x
        dy = goal_y - y
        distance = np.sqrt(dx**2 + dy**2)

        #find the difference in angle
        angle_error = goal_theta - theta
        angle_error = np.arctan2(
            np.sin(angle_error),
            np.cos(angle_error)
        )

        #stop when position and angle are close enough
        if distance < 0.05 and abs(angle_error) < 0.1:
            robot.stop()
            break

        #convert movement to robot coordinates
        vx = 0.5 * (np.cos(theta) * dx + np.sin(theta) * dy)
        vy = 0.5 * (-np.sin(theta) * dx + np.cos(theta) * dy)
        wz = 0.5 * angle_error

        #limit the speed
        vx = np.clip(vx, -0.15, 0.15)
        vy = np.clip(vy, -0.15, 0.15)
        wz = np.clip(wz, -0.3, 0.3)

        #move the robot
        robot.drive(float(vx), float(vy), float(wz))

        time.sleep(0.1)


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
#move to three goal points (example coordinates)
try:
    drive_to_pose(0.5, 0, 0, robot)
    time.sleep(1)

    drive_to_pose(0.5, 0.5, 0, robot)
    time.sleep(1)
    
    drive_to_pose(0, 0, 0, robot)

finally:
    # Stop the robot before exiting.
    robot.stop()

# TODO: Print out the robot's final odometry before exiting.
print("Final odometry:", robot.read_odometry())

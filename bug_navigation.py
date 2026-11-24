
import time
import numpy as np
from mbot_bridge.api import MBot

robot = MBot()
# TODO: Declare any other variables you might need here.
setpoint = 0.35
speed = 0.3
state = "GO_TO_GOAL"


def normalize_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def find_min_nonzero_dist_in_slice(ranges, thetas, angle, slice_size):
    min_idx = -1

    for i in range(len(ranges)):
        in_slice = abs(normalize_angle(thetas[i] - angle)) <= slice_size / 2

        if in_slice and ranges[i] > 0:
            if min_idx == -1 or ranges[i] < ranges[min_idx]:
                min_idx = i

    return min_idx


def find_min_dist(ranges, thetas):
    idx = find_min_nonzero_dist_in_slice(ranges, thetas, 0, 2 * np.pi)

    if idx == -1:
        return None, None

    return ranges[idx], thetas[idx]


def cross_product(v1, v2):
    res = np.zeros(3)

    res[0] = v1[1] * v2[2] - v1[2] * v2[1]
    res[1] = v1[2] * v2[0] - v1[0] * v2[2]
    res[2] = v1[0] * v2[1] - v1[1] * v2[0]

    return res


def wall_follow(ranges, thetas, setpoint, speed):
    dist, angle = find_min_dist(ranges, thetas)

    if dist is None:
        return 0, 0, 0

    wall = [np.cos(angle), np.sin(angle), 0]
    tangent = cross_product(wall, [0, 0, 1])

    correction = 1.5 * (dist - setpoint)

    vx = speed * tangent[0] + correction * wall[0]
    vy = speed * tangent[1] + correction * wall[1]

    return vx, vy, 0


# Reset the robot odometry to zero at the beginning of the run.
robot.reset_odometry()

# TODO: (P2.4) Ask the user for a goal pose (x and y position and angle).
goal_x = float(input("Enter goal x: "))
goal_y = float(input("Enter goal y: "))
goal_theta = float(input("Enter goal theta in radians: "))

hit_distance = None

"""
TODO: (P2.3 & P2.4) Write code to drive to the given goal while avoiding
obstacles here.

HINT: You may reuse your code and util functions from wall follower and hit the
spot wherever you need to.
"""

try:
    while True:
        pose = robot.read_odometry()
        ranges, thetas = robot.read_lidar()

        if len(pose) != 3:
            robot.stop()
            raise RuntimeError("No odometry data")

        x, y, theta = pose

        if len(ranges) == 0 or len(ranges) != len(thetas):
            robot.stop()
            print("No valid Lidar data")
            time.sleep(0.1)
            continue

        if not any(np.isfinite(r) and r > 0 for r in ranges):
            robot.stop()
            print("No valid Lidar distances")
            time.sleep(0.1)
            continue

        # Find the distance and angle to the goal
        dx = goal_x - x
        dy = goal_y - y
        distance = np.sqrt(dx**2 + dy**2)

        angle_error = normalize_angle(goal_theta - theta)

        # Check if the robot reached the goal position
        if distance < 0.05:
            state = "GOAL_REACHED"
            print("State:", state)

            if abs(angle_error) < 0.1:
                break

            wz = np.clip(0.5 * angle_error, -0.3, 0.3)
            robot.drive(0, 0, float(wz))

            time.sleep(0.1)
            continue

        if state == "GOAL_REACHED":
            state = "GO_TO_GOAL"

        # Find obstacles in the direction of the goal
        goal_angle = normalize_angle(np.arctan2(dy, dx) - theta)

        idx = find_min_nonzero_dist_in_slice(
            ranges, thetas, goal_angle, np.pi / 3
        )

        obstacle_ahead = False

        if idx != -1 and ranges[idx] < 0.5:
            obstacle_ahead = True

        # Change to wall following if an obstacle is detected
        if state == "GO_TO_GOAL" and obstacle_ahead:
            state = "FOLLOW_WALL"
            hit_distance = distance

        # Return to the goal when the path is clear
        elif state == "FOLLOW_WALL":
            if not obstacle_ahead and distance < hit_distance - 0.1:
                state = "GO_TO_GOAL"

        # Print the current state
        print("State:", state)

        if state == "GO_TO_GOAL":
            # Same movement calculations as Hit the Spot
            vx = 0.5 * (np.cos(theta) * dx + np.sin(theta) * dy)
            vy = 0.5 * (-np.sin(theta) * dx + np.cos(theta) * dy)
            wz = 0

        elif state == "FOLLOW_WALL":
            # Reuse the working wall follower
            vx, vy, wz = wall_follow(
                ranges, thetas, setpoint, speed
            )

        # Limit the movement speed
        vx = np.clip(vx, -0.15, 0.15)
        vy = np.clip(vy, -0.15, 0.15)

        robot.drive(float(vx), float(vy), float(wz))

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Navigation stopped.")

finally:
    # Stop the robot before exiting.
    robot.stop()

# TODO: Print out the robot's final odometry before exiting.
print("Final odometry:", robot.read_odometry())

from grid_map import GridMap
import pybullet as p
import pybullet_data
import time
import math

# === SETUP PYBULLET ===
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -3.7)
p.loadURDF("plane.urdf")

# === GRID + A* PATH PLANNING ===
grid = GridMap(width=10, height=10)

# Function to sync obstacles with grid + PyBullet
obs_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2])
obs_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2], rgbaColor=[1, 0, 0, 1])

def add_obstacle_to_sim(x, y):
    grid.add_obstacle(x, y)
    p.createMultiBody(0, obs_shape, obs_visual, [x, y, 0.2])

# Add obstacles
add_obstacle_to_sim(3, 3)
add_obstacle_to_sim(3, 4)
add_obstacle_to_sim(3, 5)

grid.display()

# Plan path
start = (0, 0)
goal = (7, 7)
path = grid.a_star(start, goal)
print("Planned path:", path)

# Draw path lines
for i in range(len(path) - 1):
    from_pos = (path[i][0], path[i][1], 0.1)
    to_pos = (path[i+1][0], path[i+1][1], 0.1)
    p.addUserDebugLine(from_pos, to_pos, [0, 1, 1], 10)

# === ROVER ===
chassis_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.3, 0.2, 0.1])
chassis_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.3, 0.2, 0.1], rgbaColor=[0.7, 0.7, 0.7, 1])

# Rotate cylinders to lie flat (wheel orientation)
wheel_orientation = p.getQuaternionFromEuler([math.pi / 2, 0, 0])

wheel_shape = p.createCollisionShape(
    p.GEOM_CYLINDER, radius=0.1, height=0.02, collisionFrameOrientation=wheel_orientation
)

wheel_visual = p.createVisualShape(
    p.GEOM_CYLINDER, radius=0.1, length=0.02, rgbaColor=[0, 0, 0, 1], visualFrameOrientation=wheel_orientation
)

rover_id = p.createMultiBody(
    baseMass=10,
    baseCollisionShapeIndex=chassis_shape,
    baseVisualShapeIndex=chassis_visual,
    basePosition=[0, 0, 0.5],
    linkMasses=[1] * 4,
    linkCollisionShapeIndices=[wheel_shape] * 4,
    linkVisualShapeIndices=[wheel_visual] * 4,
    linkPositions=[[0.25, 0.15, 0], [0.25, -0.15, 0], [-0.25, 0.15, 0], [-0.25, -0.15, 0]],
    linkOrientations=[[0, 0, 0, 1]] * 4,
    linkInertialFramePositions=[[0, 0, 0]] * 4,
    linkInertialFrameOrientations=[[0, 0, 0, 1]] * 4,
    linkParentIndices=[0, 0, 0, 0],
    linkJointTypes=[p.JOINT_REVOLUTE] * 4,
    linkJointAxis=[[0, 1, 0]] * 4
)

for joint in range(4):
    p.setJointMotorControl2(rover_id, joint, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

def angle_to_target(rover_pos, target):
    dx = target[0] - rover_pos[0]
    dy = target[1] - rover_pos[1]
    return math.atan2(dy, dx)

# === FOLLOW THE PATH ===
speed = 5
turn_gain = 5

for waypoint in path:
    target_x, target_y = waypoint

    while True:
        pos, orn = p.getBasePositionAndOrientation(rover_id)
        rover_x, rover_y = pos[0], pos[1]
        dist = math.hypot(target_x - rover_x, target_y - rover_y)

        if dist < 0.2:
            break

        yaw = p.getEulerFromQuaternion(orn)[2]
        target_angle = angle_to_target((rover_x, rover_y), (target_x, target_y))
        angle_diff = target_angle - yaw
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        forward_speed = speed * (1 - min(abs(angle_diff), 1.0))
        left_speed = forward_speed - turn_gain * angle_diff
        right_speed = forward_speed + turn_gain * angle_diff

        p.setJointMotorControl2(rover_id, 0, p.VELOCITY_CONTROL, targetVelocity=left_speed, force=10)
        p.setJointMotorControl2(rover_id, 2, p.VELOCITY_CONTROL, targetVelocity=left_speed, force=10)
        p.setJointMotorControl2(rover_id, 1, p.VELOCITY_CONTROL, targetVelocity=right_speed, force=10)
        p.setJointMotorControl2(rover_id, 3, p.VELOCITY_CONTROL, targetVelocity=right_speed, force=10)

        p.stepSimulation()
        time.sleep(1 / 240.)

p.disconnect()

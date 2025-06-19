import pybullet as p
import pybullet_data
import os
import time
import math
import cv2
import numpy as np

circle_radius = 10.2 / 2
circle_height = 3.50
circle_speed = 8.0
segments = 100

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

house_path = os.path.join("assets", "house_model.urdf")
p.loadURDF(house_path, basePosition=[0, 0, 0], useFixedBase=True)

drone_path = os.path.join(os.getcwd(), "quad_monobody.urdf")
drone_start_pos = [circle_radius + 1, 0, 0.2]
drone_id = p.loadURDF(drone_path, basePosition=drone_start_pos, useFixedBase=False)

camera_offset = [0, 0, 1.0]
width, height = 320, 240
fov, aspect, near, far = 60, width / height, 0.1, 10

p.resetDebugVisualizerCamera(
    cameraDistance=15,
    cameraYaw=45,
    cameraPitch=-1,
    cameraTargetPosition=[0, 0, circle_height]
)

# === Lift to altitude ===
target_pos = [drone_start_pos[0], drone_start_pos[1], circle_height]
while True:
    pos, _ = p.getBasePositionAndOrientation(drone_id)
    dz = target_pos[2] - pos[2]
    if abs(dz) < 0.05:
        break
    p.resetBaseVelocity(drone_id, linearVelocity=[0, 0, 0.1])
    p.stepSimulation()
    time.sleep(1/240)

p.resetBaseVelocity(drone_id, linearVelocity=[0, 0, 0])

# === Fly around in a circle and show camera view ===
for i in range(segments + 1):
    theta = 2 * math.pi * i / segments
    x = circle_radius * math.cos(theta)
    y = circle_radius * math.sin(theta)
    z = circle_height

    while True:
        pos, _ = p.getBasePositionAndOrientation(drone_id)
        direction = [x - pos[0], y - pos[1], z - pos[2]]
        distance = math.sqrt(sum(d ** 2 for d in direction))
        if distance < 0.05:
            break
        unit = [d / distance for d in direction]
        velocity = [circle_speed * u for u in unit]
        p.resetBaseVelocity(drone_id, linearVelocity=velocity)

        cam_pos = [pos[0] + camera_offset[0], pos[1] + camera_offset[1], pos[2] + camera_offset[2]]
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=cam_pos,
            cameraTargetPosition=[pos[0], pos[1], pos[2] - 1],
            cameraUpVector=[0, 1, 0]
        )
        proj_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

        _, _, px, _, _ = p.getCameraImage(width, height, view_matrix, proj_matrix)
        frame = np.reshape(px, (height, width, 4))[:, :, :3]  # RGB only
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        cv2.imshow("Drone Top Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        p.stepSimulation()
        time.sleep(1/240)

# === Return to start ===
return_point = [drone_start_pos[0], drone_start_pos[1], circle_height]
while True:
    pos, _ = p.getBasePositionAndOrientation(drone_id)
    direction = [return_point[i] - pos[i] for i in range(3)]
    distance = math.sqrt(sum(d**2 for d in direction))
    if distance < 0.05:
        break
    unit = [d / distance for d in direction]
    velocity = [circle_speed * u for u in unit]
    p.resetBaseVelocity(drone_id, linearVelocity=velocity)
    p.stepSimulation()
    time.sleep(1/240)

# === Descend ===
landing_height = 0.2
while True:
    pos, _ = p.getBasePositionAndOrientation(drone_id)
    dz = pos[2] - landing_height
    if dz < 0.05:
        break
    p.resetBaseVelocity(drone_id, linearVelocity=[0, 0, -0.05])
    p.stepSimulation()
    time.sleep(1/240)

p.resetBaseVelocity(drone_id, linearVelocity=[0, 0, 0])
cv2.destroyAllWindows()
print("✅ Autonomous gutter clean POC with live camera view completed.")

while True:
    p.stepSimulation()
    time.sleep(1/240)

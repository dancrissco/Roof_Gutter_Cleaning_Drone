import pybullet as p
import pybullet_data
import time
import os
import math

# === Setup ===
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

# === Visual Camera ===
p.resetDebugVisualizerCamera(cameraDistance=8,
                             cameraYaw=45,
                             cameraPitch=-45,
                             cameraTargetPosition=[0, 0, 0.1])

# === Paths to URDFs ===
base_dir = os.path.dirname(__file__)
mower_urdf = os.path.join(base_dir, "ev_mower_with_wheels.urdf")
drone_urdf = os.path.join(base_dir, "tri_quad_prop.urdf")

# === Load Lawn Area ===
def create_colored_box(pos, size, color):
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[s/2 for s in size], rgbaColor=color)
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[s/2 for s in size])
    return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis, basePosition=pos)

# Lawn = 6x4m, Sidewalk +1m each side = 8x6m
lawn_size = [6, 4, 0.02]
pavement_size = [8, 6, 0.01]
hedge_thickness = 0.1
hedge_height = 0.75

create_colored_box([0, 0, 0.005], pavement_size, [0.2, 0.2, 0.2, 1])  # sidewalk
create_colored_box([0, 0, 0.015], lawn_size, [0.0, 0.8, 0.0, 1])      # grass

# Hedges on 3 sides (north, east, west); south side left open
create_colored_box([0, 3.0 - hedge_thickness/2, hedge_height/2], [8, hedge_thickness, hedge_height], [0.0, 0.5, 0.0, 1])
create_colored_box([-4.0 + hedge_thickness/2, 0, hedge_height/2], [hedge_thickness, 6, hedge_height], [0.0, 0.5, 0.0, 1])
create_colored_box([4.0 - hedge_thickness/2, 0, hedge_height/2], [hedge_thickness, 6, hedge_height], [0.0, 0.5, 0.0, 1])

# === Load Mower and Drone ===
mower_start = [0, -3.5, 0.05]
mower_id = p.loadURDF(mower_urdf, basePosition=mower_start, useFixedBase=False)

drone_offset = [0, 0, 0.25]  # height above mower
drone_start = [mower_start[i] + drone_offset[i] for i in range(3)]
drone_id = p.loadURDF(drone_urdf, basePosition=drone_start, useFixedBase=False)

# === Drone Trimming Mission ===
step_time = 1./240.
speed = 0.02
altitude = 1.2

lawn_half_x = lawn_size[0] / 2
lawn_half_y = lawn_size[1] / 2
corners = [
    [-lawn_half_x, -lawn_half_y],
    [lawn_half_x, -lawn_half_y],
    [lawn_half_x, lawn_half_y],
    [-lawn_half_x, lawn_half_y],
]

def update_drone_camera():
    pos, _ = p.getBasePositionAndOrientation(drone_id)
    cam_pos = [pos[0], pos[1], pos[2] + 1]
    view = p.computeViewMatrix(cam_pos, pos, [0, 1, 0])
    proj = p.computeProjectionMatrixFOV(60, 1.0, 0.1, 10.0)
    p.getCameraImage(320, 320, view, proj)

def takeoff(target_z):
    while True:
        pos, orn = p.getBasePositionAndOrientation(drone_id)
        if pos[2] >= target_z:
            break
        new_pos = [pos[0], pos[1], pos[2] + 0.01]
        p.resetBasePositionAndOrientation(drone_id, new_pos, orn)
        update_drone_camera()
        p.stepSimulation()
        time.sleep(step_time)

def move_to(target_xy, z):
    while True:
        pos, orn = p.getBasePositionAndOrientation(drone_id)
        dx = target_xy[0] - pos[0]
        dy = target_xy[1] - pos[1]
        dist = math.hypot(dx, dy)
        if dist < 0.05:
            break
        direction = [dx / dist, dy / dist]
        new_pos = [pos[0] + speed * direction[0],
                   pos[1] + speed * direction[1],
                   z]
        p.resetBasePositionAndOrientation(drone_id, new_pos, orn)
        update_drone_camera()
        p.stepSimulation()
        time.sleep(step_time)

# === Start Drone Mission ===
print("🚁 Drone takeoff...")
takeoff(altitude)

print("✂️ Trimming lawn perimeter...")
for i in range(4):
    move_to(corners[(i+1)%4], altitude)

print("🔁 Returning to mower...")
move_to([drone_start[0], drone_start[1]], altitude)
takeoff(drone_start[2])  # descend to top of mower

print("✅ Mission complete. Drone landed on mower.")

# === Mower perimeter movement (after drone lands) ===

def move_mower_to(target_xy, z=0.05):

    while True:

        pos, orn = p.getBasePositionAndOrientation(mower_id)

        dx = target_xy[0] - pos[0]

        dy = target_xy[1] - pos[1]

        dist = math.hypot(dx, dy)

        if dist < 0.05:

            break

        direction = [dx / dist, dy / dist]

        new_pos = [pos[0] + speed * direction[0],

                   pos[1] + speed * direction[1],

                   z]

        p.resetBasePositionAndOrientation(mower_id, new_pos, orn)

        update_drone_camera()  # keep drone view

        p.stepSimulation()

        time.sleep(step_time)



# Same corners reused from drone

print("🚜 Mower driving lawn perimeter...")

for i in range(4):

    move_mower_to(corners[(i+1)%4])



print("✅ Mower perimeter complete. Holding position.")

def drive_mower_zigzag(x_min, x_max, y_min, y_max, row_spacing=0.4, z=0.05):
    direction = 1  # 1 for right, -1 for left
    y = y_min
    while y <= y_max:
        x_start = x_min if direction == 1 else x_max
        x_end = x_max if direction == 1 else x_min

        # Drive a row
        while True:
            pos, orn = p.getBasePositionAndOrientation(mower_id)
            dx = x_end - pos[0]
            dy = y - pos[1]
            dist = math.hypot(dx, dy)
            if dist < 0.05:
                break
            direction_vec = [dx / dist, dy / dist]
            new_pos = [pos[0] + speed * direction_vec[0],
                       pos[1] + speed * direction_vec[1],
                       z]
            p.resetBasePositionAndOrientation(mower_id, new_pos, orn)
            update_drone_camera()
            p.stepSimulation()
            time.sleep(step_time)

        # Shift one row upward
        y += row_spacing
        direction *= -1

# === Idle loop ===

print("🌿 Starting full zig-zag mowing pass...")
drive_mower_zigzag(
    x_min=-lawn_half_x,
    x_max= lawn_half_x,
    y_min=-lawn_half_y,
    y_max= lawn_half_y,
    row_spacing=0.4
)
print("✅ Mowing complete. Holding position.")
while p.isConnected():
    update_drone_camera()
    p.stepSimulation()
    time.sleep(step_time)

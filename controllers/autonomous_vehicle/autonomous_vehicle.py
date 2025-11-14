from controller import Robot, Camera, Display, GPS, Gyro, InertialUnit, Lidar, Radar, TouchSensor, Supervisor
from vehicle import Driver
import math

# to be used as array indices
X, Y, Z = 0, 1, 2

TIME_STEP = 50

# enable various 'features'
enable_display = False
has_gps = False

# GPS coordinates and speed (used in display update)
gps_coords = [0.0, 0.0, 0.0]
gps_speed = 0.0

# misc variables
speed = 0.0
steering_angle = 0.0

# Starting position for reset
START_POS_X = -45.0
START_POS_Y = 45.88
START_POS_Z = 0.4
START_ROT_X = 0.0
START_ROT_Y = 0.0
START_ROT_Z = 1.0
START_ROT_ANGLE = 3.14159

# Collision reset delay (in seconds)
COLLISION_RESET_DELAY = 5.0

# Static variables for collision reset
collision_reset_pending = False
collision_start_time = -1.0

# Supervisor (will be initialized in main)
is_supervisor = False

# =====================================================================
# DRIVING HELPER FUNCTIONS
# =====================================================================
def set_speed(kmh):
    """
    Set the target speed of the vehicle.
    """
    global speed
    # max speed
    if kmh > 250.0:
        kmh = 250.0

    speed = kmh

    print(f"setting speed to {kmh} km/h")
    driver.setCruisingSpeed(kmh)


def set_steering_angle(wheel_angle):
    """
    Set the steering angle of the vehicle.
    - positive: turn right
    - negative: turn left
    """
    global steering_angle
    # limit the difference with previous steering_angle
    if wheel_angle - steering_angle > 0.1:
        wheel_angle = steering_angle + 0.1
    if wheel_angle - steering_angle < -0.1:
        wheel_angle = steering_angle - 0.1
    steering_angle = wheel_angle
    # limit range of the steering angle
    if wheel_angle > 0.5:
        wheel_angle = 0.5
    elif wheel_angle < -0.5:
        wheel_angle = -0.5
    driver.setSteeringAngle(wheel_angle)


def update_display():
    global speedometer_image
    if not display or not speedometer_image:
        return
    
    NEEDLE_LENGTH = 50.0

    # display background
    display.imagePaste(speedometer_image, 0, 0, False)

    # draw speedometer needle
    current_speed = driver.getCurrentSpeed()
    if math.isnan(current_speed):
        current_speed = 0.0
    alpha = current_speed / 260.0 * 3.72 - 0.27
    x = int(-NEEDLE_LENGTH * math.cos(alpha))
    y = int(-NEEDLE_LENGTH * math.sin(alpha))
    display.drawLine(100, 95, 100 + x, 95 + y)

    # draw text
    txt = f"GPS coords: {gps_coords[X]:.1f} {gps_coords[Z]:.1f}"
    display.drawText(txt, 10, 130)
    txt = f"GPS speed:  {gps_speed:.1f}"
    display.drawText(txt, 10, 140)


def compute_gps_speed():
    global gps_speed, gps_coords
    coords = gps.getValues()
    speed_ms = gps.getSpeed()
    # store into global variables
    gps_speed = speed_ms * 3.6  # convert from m/s to km/h
    gps_coords = list(coords)


def reset_vehicle_position():
    """
    Reset the vehicle to the starting position.
    """
    global supervisor, is_supervisor
    if not is_supervisor or not supervisor:
        print("Warning: Cannot reset position - controller is not a Supervisor")
        return

    self_node = supervisor.getSelf()
    if not self_node:
        print("Warning: Could not get self node for reset")
        return

    translation_field = self_node.getField("translation")
    rotation_field = self_node.getField("rotation")

    if translation_field:
        translation = [START_POS_X, START_POS_Y, START_POS_Z]
        translation_field.setSFVec3f(translation)

    if rotation_field:
        rotation = [START_ROT_X, START_ROT_Y, START_ROT_Z, START_ROT_ANGLE]
        rotation_field.setSFRotation(rotation)

    # Reset velocity to zero
    zero_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    self_node.setVelocity(zero_velocity)

    print("Vehicle reset to starting position")


def check_collisions(sim_time):
    """
    Check for collisions and handle reset.
    """
    global collision_reset_pending, collision_start_time
    
    # Check if any TouchSensor detected collision
    collision_detected = False
    touch_sensor_names = [
        "touch_front_center", "touch_front_left", "touch_front_right",
        "touch_rear_center", "touch_rear_left", "touch_rear_right"
    ]
    touch_sensors = [
        touch_front_center, touch_front_left, touch_front_right,
        touch_rear_center, touch_rear_left, touch_rear_right
    ]

    for i in range(6):
        if touch_sensors[i]:
            value = touch_sensors[i].getValue()
            if value > 0.5:  # safer than >=1.0
                print(f"COLLISION DETECTED: {touch_sensor_names[i]} = {value:.6f}")
                collision_detected = True
                if not collision_reset_pending:
                    # Start the collision timer
                    collision_reset_pending = True
                    collision_start_time = sim_time
                    print(f"Collision detected, will reset in {COLLISION_RESET_DELAY:.1f} seconds...")

    # Reset logic when collision is detected
    if collision_reset_pending and collision_start_time >= 0.0:
        elapsed = sim_time - collision_start_time
        if elapsed >= COLLISION_RESET_DELAY:
            reset_vehicle_position()
            collision_reset_pending = False
            collision_start_time = -1.0

    # Reset the flag when no collision is detected
    if not collision_detected and not collision_reset_pending:
        collision_start_time = -1.0


# =====================================================================
# INIT SCENE AND SENSORS
# =====================================================================
robot = Robot()
driver = Driver()
# Check if robot is a supervisor (Supervisor is a subclass of Robot)
try:
    supervisor = Supervisor()
    is_supervisor = True
except:
    supervisor = None
    is_supervisor = False

# check if there is a display
for j in range(robot.getNumberOfDevices()):
    device = robot.getDeviceByIndex(j)
    name = device.getName()
    if name == "display":
        enable_display = True
    elif name == "gps":
        has_gps = True

# Initialize camera
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)
camera_width = camera.getWidth()
camera_height = camera.getHeight()
camera_fov = camera.getFov()
# Enable camera recognition
camera.recognitionEnable(TIME_STEP)
print(f"Camera initialized: {camera_width}x{camera_height}, FOV={camera_fov:.2f} (recognition enabled)")

# initialize gps
if has_gps:
    gps = robot.getDevice("gps")
    if gps:
        gps.enable(TIME_STEP)
        print("GPS initialized")
    else:
        print("Warning: GPS device not found")
else:
    print("GPS not available")

# initialize display (speedometer)
if enable_display:
    display = robot.getDevice("display")
    if display:
        try:
            speedometer_image = display.imageLoad("speedometer.png")
            if speedometer_image:
                print("Display initialized with speedometer image")
            else:
                print("Warning: Could not load speedometer.png image")
                speedometer_image = None
        except Exception as e:
            print(f"Warning: Error loading speedometer image: {e}")
            speedometer_image = None
    else:
        print("Warning: Display device not found")
else:
    print("Display not available")

# Initialize Radar
front_radar = robot.getDevice("front_radar")
if front_radar:
    front_radar.enable(TIME_STEP)
    print("Radar initialized")
else:
    print("Radar not available")

# Initialize Touch Sensors
touch_front_center = robot.getDevice("touch_front_center")
touch_front_left = robot.getDevice("touch_front_left")
touch_front_right = robot.getDevice("touch_front_right")
touch_rear_center = robot.getDevice("touch_rear_center")
touch_rear_left = robot.getDevice("touch_rear_left")
touch_rear_right = robot.getDevice("touch_rear_right")

if touch_front_center:
    touch_front_center.enable(TIME_STEP)
if touch_front_left:
    touch_front_left.enable(TIME_STEP)
if touch_front_right:
    touch_front_right.enable(TIME_STEP)
if touch_rear_center:
    touch_rear_center.enable(TIME_STEP)
if touch_rear_left:
    touch_rear_left.enable(TIME_STEP)
if touch_rear_right:
    touch_rear_right.enable(TIME_STEP)

# Log initialized sensors
touch_sensor_names = [
    "touch_front_center", "touch_front_left", "touch_front_right",
    "touch_rear_center", "touch_rear_left", "touch_rear_right"
]
touch_sensors = [
    touch_front_center, touch_front_left, touch_front_right,
    touch_rear_center, touch_rear_left, touch_rear_right
]
touch_sensor_count = 0
for i in range(6):
    if touch_sensors[i]:
        print(f"Touch sensor initialized: {touch_sensor_names[i]}")
        touch_sensor_count += 1
if touch_sensor_count == 0:
    print("Warning: No touch sensors found!")

# Initialize Gyro
gyro = robot.getDevice("gyro")
if gyro:
    gyro.enable(TIME_STEP)
    print("Gyro initialized")
else:
    print("Gyro not available")

# Initialize Inertial Unit (IMU)
imu = robot.getDevice("imu")
if imu:
    imu.enable(TIME_STEP)
    print("IMU initialized")
else:
    print("IMU not available")

# Initialize Roof Lidar
roof_lidar = robot.getDevice("roof_lidar")
if roof_lidar:
    roof_lidar.enable(TIME_STEP)
    roof_lidar_width = roof_lidar.getHorizontalResolution()
    roof_lidar_layers = roof_lidar.getNumberOfLayers()
    roof_lidar_range = roof_lidar.getMaxRange()
    roof_lidar_fov = roof_lidar.getFov()
    print(f"Roof Lidar initialized: {roof_lidar_width}x{roof_lidar_layers} layers, range={roof_lidar_range:.2f}, fov={roof_lidar_fov:.2f}")
else:
    print("Roof Lidar not available")

# start engine
driver.setHazardFlashers(True)
driver.setDippedBeams(True)
driver.setAntifogLights(True)
driver.setWiperMode(Driver.SLOW)
set_speed(50.0)  # Set initial speed to 50 km/h

# =====================================================================
# MAIN CONTROLLER LOOP
# =====================================================================
i = 0
while driver.step() != -1:
    # updates sensors only every TIME_STEP milliseconds
    if i % int(TIME_STEP / robot.getBasicTimeStep()) == 0:
        # update stuff
        if has_gps:
            compute_gps_speed()
        if enable_display:
            update_display()

        # Check for collisions
        sim_time = robot.getTime()
        check_collisions(sim_time)

    i += 1

driver.cleanup()


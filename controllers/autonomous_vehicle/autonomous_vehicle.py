from controller import Robot, Camera, Display, GPS, Lidar, TouchSensor, Supervisor
from vehicle import Driver
import math
import random
import pickle
import os
import time
import sys
import random

# Add parent directories to Python path so we can import constants and models
# Webots runs controllers from their directory, so we need to go up two levels
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from constants import *
from models.QLearningAgent import QLearningAgent

enable_display = False
has_gps = False

# GPS coordinates and speed (used in display update)
gps_coords = [0.0, 0.0, 0.0]
gps_speed = 0.0

# misc variables
steering_angle = 0.0

# Static variables for GPS stuck detection
last_gps_position = None
last_gps_position_time = -1.0
gps_stuck_reset_pending = False

# Supervisor (will be initialized in main)
supervisor = None
is_supervisor = False


speedometer_image = robot = driver = display = camera = gps = roof_lidar = None
touch_front_center = touch_front_left = touch_front_right = None
touch_rear_center = touch_rear_left = touch_rear_right = None
q_agent = None

# =====================================================================
# DRIVING HELPER FUNCTIONS
# =====================================================================
def set_speed(kmh):
    """
    Set the target speed of the vehicle. Relies on a globally defined 'Driver' object
    """
    if kmh > 250.0:
        kmh = 250.0
    driver.setCruisingSpeed(kmh)

def start_engine(): 
    driver.setHazardFlashers(True)
    driver.setDippedBeams(True)
    driver.setAntifogLights(True)
    driver.setWiperMode(Driver.SLOW)
    set_speed(SPEED)  # Set initial speed to 50 km/h

def set_steering_angle(angle_adjustment):
    """
    Set the steering angle of the vehicle.
    - positive: turn right
    - negative: turn left
    """
    global steering_angle
    wheel_angle = steering_angle + angle_adjustment
    wheel_angle = max(min(wheel_angle, MAX_ANGLE), MIN_ANGLE)
    steering_angle = wheel_angle
    
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
    txt = f"GPS coords: {gps_coords[0]:.1f} {gps_coords[2]:.1f}"
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

def check_gps_stuck(sim_time):
    """
    Check if vehicle is stuck (hasn't moved significantly in GPS_STUCK_TIMEOUT seconds).
    Returns True if stuck and reset should be triggered, False otherwise.
    """
    global last_gps_position, last_gps_position_time, gps_stuck_reset_pending
    
    if not has_gps or not gps:
        return False
    
    current_coords = gps.getValues()
    current_position = (current_coords[0], current_coords[1])  
    
    # Initialize or check if position has changed significantly
    if last_gps_position is None:
        last_gps_position = current_position
        last_gps_position_time = sim_time
        return False
    
    # Calculate distance moved
    dx = current_position[0] - last_gps_position[0]
    dz = current_position[1] - last_gps_position[1]  
    distance_moved = math.sqrt(dx*dx + dz*dz)
    
    # If moved significantly, update position and reset timer
    if distance_moved >= GPS_POSITION_THRESHOLD:
        last_gps_position = current_position
        last_gps_position_time = sim_time
        gps_stuck_reset_pending = False
        return False
    
    # Check if stuck for too long
    time_since_last_move = sim_time - last_gps_position_time
    if time_since_last_move >= GPS_STUCK_TIMEOUT:
        if not gps_stuck_reset_pending:
            # print(f"Vehicle stuck detected! Position unchanged for {time_since_last_move:.2f} seconds. Resetting...")
            gps_stuck_reset_pending = True
            reset_vehicle_position()
            set_speed(SPEED)
            # Reset GPS tracking after reset
            last_gps_position = None
            last_gps_position_time = sim_time
            gps_stuck_reset_pending = False
            return True
    
    return False

def reset_vehicle_position():
    """
    Reset the vehicle to the starting position with zero velocity.
    Completely resets position, rotation, velocity, speed, and steering.
    """
    global supervisor, is_supervisor, steering_angle

    index = random.randint(0, 9)
    respawn_pos = RESPAWN_CANDIDATES[index]
    random_float = random.uniform(0, 6.28)
    
    if not is_supervisor or not supervisor:
        print("Warning: Cannot reset position - controller is not a Supervisor")
        return

    self_node = supervisor.getSelf()
    if not self_node:
        print("Warning: Could not get self node for reset")
        return

    # Stop the vehicle first - reset speed and steering commands
    driver.setCruisingSpeed(0.0)
    driver.setSteeringAngle(0.0)
    steering_angle = 0.0

    # Reset velocity to zero (linear and angular)
    self_node.setVelocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # Reset position and rotation
    translation_field = self_node.getField("translation")
    rotation_field = self_node.getField("rotation")

    if rotation_field:
        rotation_field.setSFRotation([START_ROT_X, START_ROT_Y, START_ROT_Z, random_float])

    if translation_field:
        translation_field.setSFVec3f([respawn_pos[0], respawn_pos[1], respawn_pos[2]])


def check_collisions(sim_time):
    """
    Check for collisions and handle reset immediately.
    Returns True if collision detected, False otherwise.
    """
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
            if value > 0.5:
                # print(f"COLLISION DETECTED: {touch_sensor_names[i]} = {value:.6f}")
                collision_detected = True
                # Reset immediately on collision detection
                reset_vehicle_position()
                set_speed(SPEED)
                break  # Reset once per collision detection cycle
    
    return collision_detected

# ====================================
#  FEATURE EXTRACTION START
# ====================================

def extract_lidar_features(lidar_sensor):
    """
    Extract distance measurements from LIDAR in key directions.
    Returns: (front_distance, left_distance, right_distance)
    """
    if not lidar_sensor:
        raise ValueError("No LIDAR sensor available")
    
    try:
        range_image = lidar_sensor.getRangeImage()
        width = lidar_sensor.getHorizontalResolution()
        layers = lidar_sensor.getNumberOfLayers()
        max_range = lidar_sensor.getMaxRange()
        
        if not range_image or len(range_image) == 0:
            return (100.0, 100.0, 100.0)
        
        total_elements = width * layers
        
        if len(range_image) < total_elements:
            if len(range_image) < width:
                return (100.0, 100.0, 100.0)
            layer_data = range_image[:width]
        else:
            layer_data = range_image[:width]
        
        # Define angular sectors (assuming 360-degree FOV, starting from front)
        # Front: center of array (0 degrees)
        # Left: 90 degrees clockwise (1/4 from start)
        # Right: -90 degrees (3/4 from start)
        
        front_start = int(width * 0.45)
        front_end = int(width * 0.55)
        left_start = int(width * 0.2)
        left_end = int(width * 0.3)
        right_start = int(width * 0.7)
        right_end = int(width * 0.8)
        
        # Get minimum distances in each sector (filter out invalid values)
        front_distances = [layer_data[i] for i in range(front_start, min(front_end, len(layer_data))) 
                          if not math.isnan(layer_data[i]) and layer_data[i] > 0]
        left_distances = [layer_data[i] for i in range(left_start, min(left_end, len(layer_data))) 
                         if not math.isnan(layer_data[i]) and layer_data[i] > 0]
        right_distances = [layer_data[i] for i in range(right_start, min(right_end, len(layer_data))) 
                          if not math.isnan(layer_data[i]) and layer_data[i] > 0]
        
        front_min = min(front_distances) if front_distances else max_range
        left_min = min(left_distances) if left_distances else max_range
        right_min = min(right_distances) if right_distances else max_range
        
        # Clamp to max_range
        front_min = min(front_min, max_range)
        left_min = min(left_min, max_range)
        right_min = min(right_min, max_range)
        
        return (front_min, left_min, right_min)
    except Exception as e:
        print(f"Error extracting LIDAR features: {e}")
        return (100.0, 100.0, 100.0)


def extract_camera_features(camera_sensor):
    """
    Extract recognition data from camera.
    Returns: (object_count, closest_distance, closest_position_x, closest_position_z)
    """
    if not camera_sensor:
        return (0, 100.0, 0.0, 0.0)
    try:
        objects = camera_sensor.getRecognitionObjects()
        if not objects or len(objects) == 0:
            return (0, 100.0, 0.0, 0.0)
        
        object_count = len(objects)
        closest_distance = 100.0
        closest_position_x = 0.0
        closest_position_z = 0.0
        
        # Find closest object
        for obj in objects:
            if hasattr(obj, 'position') and obj.position:
                # Calculate distance from camera (position is relative to camera)
                distance = math.sqrt(obj.position[0]**2 + obj.position[1]**2 + obj.position[2]**2)
                if distance < closest_distance:
                    closest_distance = distance
                    closest_position_x = obj.position[0] if len(obj.position) > 0 else 0.0
                    closest_position_z = obj.position[2] if len(obj.position) > 2 else 0.0
        
        return (object_count, closest_distance, closest_position_x, closest_position_z)
    except Exception as e:
        print(f"Error extracting camera features: {e}")
        return (0, 100.0, 0.0, 0.0)

# ====================================
#  FEATURE EXTRACTION END
# ====================================


# ====================================
#  DISCRETIZE START
# ====================================

def discretize_distance(distance, max_range=100.0):
    """
    Discretize distance into bins.
    Bins: [0-2], [2-5], [5-10], [10-20], [20-30], [30-50], [50-70], [70-90], [90-100], [>100]
    """
    if distance < 2.0:
        return 0
    elif distance < 5.0:
        return 1
    elif distance < 10.0:
        return 2
    elif distance < 20.0:
        return 3
    elif distance < 30.0:
        return 4
    elif distance < 50.0:
        return 5
    elif distance < 70.0:
        return 6
    elif distance < 90.0:
        return 7
    elif distance < max_range:
        return 8
    else:
        return 9


def discretize_steering(steering_angle):
    """
    Discretize steering angle into bins matching steering actions.
    """
    if steering_angle < -0.225:
        return 0  # -0.25
    elif steering_angle < -0.175:
        return 1  # -0.20
    elif steering_angle < -0.125:
        return 2  # -0.15
    elif steering_angle < -0.075:
        return 3  # -0.10
    elif steering_angle < -0.025:
        return 4  # -0.05
    elif steering_angle < 0.025:
        return 5  # 0.0
    elif steering_angle < 0.075:
        return 6  # 0.05
    elif steering_angle < 0.125:
        return 7  # 0.10
    elif steering_angle < 0.175:
        return 8  # 0.15
    elif steering_angle < 0.225:
        return 9  # 0.20
    else:
        return 10  # 0.25


def discretize_camera_object_count(count, max_count=10):
    """
    Discretize camera object count.
    """
    return min(count, max_count - 1)


def discretize_camera_position(position):
    """
    Discretize camera object position (x or z coordinate).
    Bins: <-5m, -5 to -2m, -2 to 2m, 2 to 5m, >5m
    """
    if position < -5.0:
        return 0
    elif position < -2.0:
        return 1
    elif position < 2.0:
        return 2
    elif position < 5.0:
        return 3
    else:
        return 4

# ====================================
#  DISCRETIZE END
# ====================================

def get_state(lidar_sensor, current_steering, camera_sensor=None):
    """
    Extract and discretize current state from sensors.
    Returns: state tuple (lidar_front_bin, lidar_left_bin, lidar_right_bin, steering_bin, 
             camera_object_count_bin, camera_distance_bin, camera_pos_x_bin)
    """
    lidar_front, lidar_left, lidar_right = extract_lidar_features(lidar_sensor)
    
    lidar_front_bin = discretize_distance(lidar_front)
    lidar_left_bin = discretize_distance(lidar_left)
    lidar_right_bin = discretize_distance(lidar_right)
    steering_bin = discretize_steering(current_steering)

    # Camera features
    camera_object_count_bin = 0
    camera_distance_bin = 9  # Default to max distance bin
    camera_pos_x_bin = 2  # Default to center
    if camera_sensor:
        object_count, closest_distance, closest_pos_x, closest_pos_z = extract_camera_features(camera_sensor)
        camera_object_count_bin = discretize_camera_object_count(object_count, CAMERA_OBJECT_COUNT_BINS)
        camera_distance_bin = discretize_distance(closest_distance)
        camera_pos_x_bin = discretize_camera_position(closest_pos_x)
    
    return (lidar_front_bin, lidar_left_bin, lidar_right_bin, steering_bin, camera_object_count_bin, camera_distance_bin, camera_pos_x_bin)

def get_reward(collision_detected, lidar_front=None, lidar_left=None, lidar_right=None):
    """
    Calculate reward based on collision status and distance from objects.
    - Collision: -1000
    - No collision: base reward + distance-based reward
    - Distance reward: rewards being further from objects
    """
    if collision_detected:
        return REWARD_COLLISION
    
    reward = REWARD_NO_COLLISION
    
    distances = []
    if lidar_front is not None:
        distances.append(lidar_front)
    if lidar_left is not None:
        distances.append(lidar_left)
    if lidar_right is not None:
        distances.append(lidar_right)
    
    if distances:
        min_distance = min(distances)
        
        # Reward based on distance:
        if min_distance < MIN_SAFE_DISTANCE:
            distance_reward = -REWARD_DISTANCE_SCALE * (MIN_SAFE_DISTANCE - min_distance)
        elif min_distance >= MAX_REWARD_DISTANCE:
            distance_reward = REWARD_DISTANCE_SCALE * MAX_REWARD_DISTANCE
        else:
            distance_reward = REWARD_DISTANCE_SCALE * min_distance
        
        reward += distance_reward
    
    return reward

# ====================================
#  EXECUTION START
# ====================================
def init_scene_and_sensors():
    """
    returns all of the sensors so that they are usable
    """
    global robot, driver, display, camera, gps, roof_lidar, supervisor, is_supervisor, enable_display, has_gps
    global touch_front_center, touch_front_left, touch_front_right, touch_rear_center, touch_rear_left, touch_rear_right
    global q_agent, speedometer_image
    
    # Check if robot is a supervisor first
    try:
        robot = Supervisor()
        supervisor = robot
        is_supervisor = True
    except:
        robot = Robot()
        supervisor = None
        is_supervisor = False
    
    driver = Driver()

    # check if there is a display
    for j in range(robot.getNumberOfDevices()):
        device = robot.getDeviceByIndex(j)
        name = device.getName()
        if name == "display":
            enable_display = True

    # Initialize camera
    camera = robot.getDevice("camera")
    if camera:
        camera.enable(TIME_STEP)
        # Enable camera recognition
        camera.recognitionEnable(TIME_STEP)
        print("Camera initialized (recognition enabled)")
    else:
        print("Camera not available")

    # Initialize GPS (always available in this world)
    gps = robot.getDevice("gps")
    gps.enable(TIME_STEP)
    has_gps = True
    print("GPS initialized")

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
    for i in range(6):
        if touch_sensors[i]:
            print(f"Touch sensor initialized: {touch_sensor_names[i]}")

    # Initialize Roof Lidar
    roof_lidar = robot.getDevice("roof_lidar")
    if roof_lidar:
        roof_lidar.enable(TIME_STEP)
        print("Roof Lidar initialized")
    else:
        print("Roof Lidar not available")

    # start engine
    start_engine()

    # Initialize agent
    q_agent = QLearningAgent()

    # Return touch sensors list for convenience (all devices are now global)
    touch_sensors = [
        touch_front_center, touch_front_left, touch_front_right,
        touch_rear_center, touch_rear_left, touch_rear_right
    ]
    return touch_sensors

def main():
    i = 0
    last_q_save_time = 0.0
    Q_SAVE_INTERVAL = 60.0  # Save Q-table every 60 seconds

    # main controller loop
    while driver.step() != -1:
        # updates sensors only every time step
        if i % int(TIME_STEP / robot.getBasicTimeStep()) == 0:
            sim_time = robot.getTime()
            
            # update stuff
            if has_gps:
                compute_gps_speed()
                # Check if vehicle is stuck (hasn't moved in 3 seconds)
                gps_stuck = check_gps_stuck(sim_time)
            if enable_display:
                update_display()

            # Check for collisions

            collision_detected = check_collisions(sim_time)
            
            # Q-Learning collision avoidance
            if roof_lidar:
                # Get current state (speed is constant, so not included in state)
                current_state = get_state(roof_lidar, steering_angle, camera) # S

                # Extract sensor distances for reward calculation
                lidar_front, lidar_left, lidar_right = extract_lidar_features(roof_lidar)
                
                # If we have a previous state and action, update Q-table
                if q_agent.last_state is not None and q_agent.last_action is not None:
                    reward = get_reward(collision_detected, lidar_front, lidar_left, lidar_right)
                    q_agent.update(q_agent.last_state, q_agent.last_action, reward, current_state)
                
                # Select new action (unless GPS stuck reset is pending)
                if not gps_stuck_reset_pending and not collision_detected:
                    steering_angle_adjustment = q_agent.select_action(current_state)  # Returns steering angle
                    
                    # Execute action - only steering, speed stays constant
                    set_steering_angle(steering_angle_adjustment)
                    # Speed remains constant at SPEED
                    
                    # Store state and action for next update
                    q_agent.last_state = current_state
                    q_agent.last_action = steering_angle_adjustment  # Store steering angle
                    q_agent.episode_steps += 1
                    q_agent.step_count += 1
                    
                    # Decay exploration rate
                    q_agent.decay_epsilon()
                else:
                    # Reset Q-learning episode on collision or GPS stuck reset
                    if q_agent.episode_steps > 0:
                        q_agent.episode_count += 1
                        avg_episode_reward = q_agent.episode_reward / q_agent.episode_steps if q_agent.episode_steps > 0 else 0.0
                        print(f"[Episode {q_agent.episode_count}] Steps: {q_agent.episode_steps}, "
                            f"Total Reward: {q_agent.episode_reward:.2f}, Avg Reward/Step: {avg_episode_reward:.3f}")
                    q_agent.last_state = None
                    q_agent.last_action = None
                    q_agent.episode_reward = 0.0
                    q_agent.episode_steps = 0
                
                # Save Q-table periodically
                if sim_time - last_q_save_time >= Q_SAVE_INTERVAL:
                    q_agent.save_q_table()
                    last_q_save_time = sim_time
                    explore_rate = q_agent.explore_count / (q_agent.explore_count + q_agent.exploit_count) * 100 if (q_agent.explore_count + q_agent.exploit_count) > 0 else 0.0
                    avg_reward = q_agent.total_reward / q_agent.q_updates_count if q_agent.q_updates_count > 0 else 0.0
                    print(f"[Periodic Save] Epsilon: {q_agent.epsilon:.4f}, Q-table size: {len(q_agent.q_table)}, "
                        f"Episodes: {q_agent.episode_count}, Updates: {q_agent.q_updates_count}, "
                        f"Explore rate: {explore_rate:.1f}%, Avg reward: {avg_reward:.3f}")

        i += 1

    # Save Q-table on exit
    if roof_lidar:
        q_agent.save_q_table()
        print("Final Q-table saved on exit")

    driver.cleanup()

# Execution
touch_sensors = init_scene_and_sensors()  # All devices are now global variables
main()
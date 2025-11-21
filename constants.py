#######################################################################
# =====================================================================
# MAIN CONSTANTS (START)
# =====================================================================
TIME_STEP = 25

# Starting position for reset
START_POS_X = -0.613659
START_POS_Y = -0.0464224
START_POS_Z = 0.0975
START_ROT_X = 0.0
START_ROT_Y = 0.0
START_ROT_Z = -1
START_ROT_ANGLE = 5.12323
SPEED = 25.0

# Valid range for steering angle
MIN_ANGLE = -0.25
MAX_ANGLE = 0.25

# GPS stuck detection
GPS_STUCK_TIMEOUT = 5.0  # Reset if position doesn't change in 3 seconds
GPS_POSITION_THRESHOLD = 0.5  # Minimum distance change (in meters) to consider movement

# DISCRETIZE STATE SPACE (LIDAR, RADAR, IMU, CAMERA)
LIDAR_BINS = 10  # Distance bins: 0-2m, 2-5m, 5-10m, 10-20m, 20-30m, 30-50m, 50-70m, 70-90m, 90-100m, >100m
IMU_YAW_BINS = 8  # Yaw angle bins (8 directions)
CAMERA_OBJECT_COUNT_BINS = 10  # 0-9 objects
STEERING_BIN_IDX = 4 # TODO: if we change the items in the state, this should change

# Reward parameters
REWARD_COLLISION = -1000
REWARD_NO_COLLISION = 1
REWARD_DISTANCE_SCALE = 0.1 
MIN_SAFE_DISTANCE = 3
MAX_REWARD_DISTANCE = 10

# =====================================================================
# MAIN CONSTANTS (END)
# =====================================================================
#######################################################################
# =====================================================================
# QLEARNINGAGENT CONSTANTS (START)
# =====================================================================
# Hyperparameters
ALPHA = 0.1  # Learning rate
GAMMA = 0.9  # Discount factor

# Exploration with decay
EPSILON_START = 0.3 
EPSILON_DECAY = 0.9995 
EPSILON_MIN = 0.05  

# DISCRETIZE ACTION SPACE (STEERING ONLY) - 11 possible actions
STEERING_ACTIONS = [-0.25, -0.20, -0.15, -0.10, -0.05, 0.0, 0.05, 0.10, 0.15, 0.20, 0.25]  # 11 steering options (increments of 0.05, max 0.25 in either direction)

# DISCRETIZED ACTION SPACE (ADJUSTMENT RELATIVE TO THE CURRENT STEERING ANGLE) - 11 possible actions
STEERING_ADJUSTMENTS = [-0.10, -0.08, -0.06, -0.04, -0.02, 0, 0.02, 0.04, 0.06, 0.08, 0.10]

# Save Q values
Q_TABLE_FILE = "q_table.pkl"
# =====================================================================
# QLEARNINGAGENT CONSTANTS (END)
# =====================================================================
########################################################################
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
STEERING_BIN_IDX = 3 # TODO: if we change the items in the state, this should change

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
EPSILON_START = 0.7
EPSILON_DECAY = 0.9995 
EPSILON_MIN = 0.05  

# DISCRETIZED ACTION SPACE (ADJUSTMENT RELATIVE TO THE CURRENT STEERING ANGLE) - 11 possible actions
STEERING_ADJUSTMENTS = [-0.05, -0.04, -0.03, -0.02, -0.01, 0, 0.01, 0.02, 0.03, 0.04, 0.05]

# Save Q values
Q_TABLE_FILE = "q_table.pkl"
# =====================================================================
# QLEARNINGAGENT CONSTANTS (END)
# =====================================================================
########################################################################

RESPAWN_CANDIDATES = [(-35, 35, 0.113), (-10, 40, 0.113), (15, 50, 0.113), (-35, 5, 0.113), (0, 0, 0.113), (20, 10, 0.113), (40, -10, 0.113), (-22, -30, 0.113), (-30, -48, 0.113), (30, -55, 0.113)]
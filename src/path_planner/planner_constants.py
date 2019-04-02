CON_ADDRESS = '127.0.0.1'
CON_PORT = 19999
TIMEOUT_IN_MS = 5000
COMM_THREAD_CYCLE_IN_MS = 5
ROBOTS_NAMES_TREE = 'startConfigurations'
TARGETS_NAMES_TREE = 'goalConfigurations'
OBSTACLES_NAMES_TREE = 'OuterWalls'
LOW_BOUNDS = -5.
HIGH_BOUNDS = 5.
PLANNER_TYPE = 'rrtconnect'
PLANNER_RANGE = 0.05
RUN_TIME = 1
INIT_SPEED = 2
ROTATION_SPEED = 0.1

kp = 0.01
ki = 0.0001
kd = 0.00009


#kp = 0.007
#ki = 0.0001
#kd = 0.0001

iMin = -0.2
iMax = 0.2


platform_target = {2: 246, 3:247, 4:248}

vrep_platforms_target = {}
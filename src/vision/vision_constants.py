import numpy as np
from collections import namedtuple


CAMERA_INDEX = 0

IMAGE_SIZE = 720
CV_WAITKEY = 10

LOW_BOUNDS = -5.    # bound fot ompl remapping
HIGH_BOUNDS = 5.    # bound fot ompl remapping
EPS = 20
ANGLE_EPS = 3

MAP_ROWS = 7
MAP_COLUMNS = 7

MARKER_SIZE = 0.02
ROBOT_SIZE = 0.21
WHEEL_SIZE = 0.065

# cam_parametrs = np.load('/home/ivan/Documents/platforms_ws/src/mobile_platforms_ROS/src/vision/960x720_web_cam.npz')
# MTX = cam_parametrs['mtx']
# DIST = cam_parametrs['dist']



mtx = np.array([[1.04981304e+03, 0.00000000e+00, 4.77445758e+02],
       [0.00000000e+00, 1.04981304e+03, 3.80426521e+02],
       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

dist = np.array([[-1.16985883e+00],
       [ 6.10042424e+02],
       [ 4.31317245e-03],
       [ 2.70140414e-03],
       [ 5.70879786e+02],
       [-1.35415738e+00],
       [ 6.08091587e+02],
       [ 5.60461210e+02],
       [ 0.00000000e+00],
       [ 0.00000000e+00],
       [ 0.00000000e+00],
       [ 0.00000000e+00],
       [ 0.00000000e+00],
       [0.00000000e+00]])

CAM_PARAMS = namedtuple('CAM_PARAMS', ['mtx' , 'dist'])

cam_parametrs = CAM_PARAMS(mtx, dist)

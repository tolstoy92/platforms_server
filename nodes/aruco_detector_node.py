#!/usr/bin/env python


import os
import cv2
import math
import rospy
import numpy as np
from cv2 import aruco
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from vision.vision_constants import MARKER_SIZE, cam_parametrs
from platforms_server.msg import ArucoData, MarkerData, Point2d


def corners_to_msg(corners):
    corners_msgs_list = []
    for corner in corners:
        corners_msg = list(Point2d(x, y) for x, y in corner[0])
        corners_msgs_list.append(corners_msg)
    return corners_msgs_list

# def rotation_matrix_to_attitude_angles(r):
#     R = cv2.Rodrigues(r, np.zeros((3, 3)))[0]
#     print(R)
#     yaw_cos = R[1][1]
#     yaw_sign = R[0][1]/abs(R[0][1])
#     roll_cos = R[0][2]
#     pitch_cos = R[1][2]
#     return yaw_cos, yaw_sign, roll_cos, pitch_cos
#     # cos_beta = math.sqrt(R[2,1] * R[2,1] + R[2,2] * R[2,2])
#     # validity = cos_beta < 1e-6
#     # if not validity:
#     #     alpha = math.atan2(R[1,0], R[0,0])    # yaw   [z]
#     #     beta  = math.atan2(-R[2,0], cos_beta) # pitch [y]
#     #     gamma = math.atan2(R[2,1], R[2,2])    # roll  [x]
#     # else:
#     #     alpha = math.atan2(R[1,0], R[0,0])    # yaw   [z]
#     #     beta  = math.atan2(-R[2,0], cos_beta) # pitch [y]
#     #     gamma = 0                             # roll  [x]
#     #
#     # return np.array([alpha, beta, gamma])

def callback(data):
    if not rospy.is_shutdown():
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            all_corners, ids_np, _ = aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)
            rvecs, tvecs = aruco.estimatePoseSingleMarkers(all_corners, MARKER_SIZE,
                                                           cam_parametrs.mtx, cam_parametrs.dist)
            # if not isinstance(rvecs, type(None)):
            #     yaw_cos, yaw_sign, roll_cos, pitch_cos = rotation_matrix_to_attitude_angles(rvecs[0])
            #     # print('\n\n\n')

            if not isinstance(ids_np, type(None)):
                aruco_data_msg = ArucoData()
                corners_msgs_list = corners_to_msg(all_corners)
                ids_msgs_list = list(id[0] for id in ids_np.tolist())
                markers = list(MarkerData(id, corners, Point(*real_world_position[0]))\
                               for id, corners, real_world_position in zip(ids_msgs_list, corners_msgs_list, tvecs))
                aruco_data_msg.markers = markers
                markers_data_publisher.publish(aruco_data_msg)

        except CvBridgeError as e:
            print(e)
    else:
        cv2.destroyAllWindows()

bridge = CvBridge()

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
parameters = aruco.DetectorParameters_create()

rospy.init_node("aruco_detector_node")
markers_data_publisher = rospy.Publisher("detected_markers", ArucoData, queue_size=1)
image_sub = rospy.Subscriber("square_image", Image, callback)

rospy.spin()

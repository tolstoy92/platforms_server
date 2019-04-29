#!/usr/bin/env python


import cv2
import rospy
import numpy as np
from cv2 import aruco
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from platforms_server.msg import ArucoData, MarkerData, Point2d

mtx, dist = np.array(rospy.get_param('mtx')), np.array(rospy.get_param('dist'))
MARKER_SIZE = rospy.get_param('MARKER_SIZE')


def corners_to_msg(corners):
    corners_msgs_list = []
    for corner in corners:
        corners_msg = list(Point2d(x, y, False) for x, y in corner[0])
        corners_msgs_list.append(corners_msg)
    return corners_msgs_list


def callback(data):
    if not rospy.is_shutdown():
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            all_corners, ids_np, _ = aruco.detectMarkers(cv_image, aruco_dict,
                                                         parameters=parameters)
            rvecs, tvecs = aruco.estimatePoseSingleMarkers(all_corners, MARKER_SIZE, mtx, dist)
            if not isinstance(ids_np, type(None)):
                aruco_data_msg = ArucoData()
                corners_msgs_list = corners_to_msg(all_corners)
                ids_msgs_list = list(id[0] for id in ids_np.tolist())
                markers = list(MarkerData(id, corners, Point(*real_world_position[0])) for
                               id, corners, real_world_position in
                               zip(ids_msgs_list, corners_msgs_list, tvecs))
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

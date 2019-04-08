#!/usr/bin/env python

import cv2

import rospy
from vision.Connection import ParallelConnection

from time import sleep
from random import randint
from sensor_msgs.msg import Image
from vision.geometry_utils import Point
from cv_bridge import CvBridge, CvBridgeError
from platforms_server.msg import AllPathes, Path, FieldObjects as FieldObjects_msg


# MARKER_IDS = [2, 3, 4]
robots_num = 2

def create_msg(id, path):
    path_msg = Path()
    path_msg.platform_id = id
    path_msg.path_points = path
    return path_msg

OK = False

was_robot_data = False

PATH_CREATED = False

def obj_callback(msg_data):
    global was_robot_data, robots_num, PATH_CREATED
    if not was_robot_data:
        if not PATH_CREATED:
            final_msg = AllPathes()
            if len(msg_data.robots) == robots_num:
                robot1, robot2 = msg_data.robots[0], msg_data.robots[1]
                connection_path_creator = ParallelConnection(robot1, robot2)
                if not connection_path_creator.is_robots_too_close():
                    pt1 = connection_path_creator.find_point1()
                    pt2 = connection_path_creator.find_point2(pt1)
                    connection_path_creator.check(pt1, pt2)
                    r1_path_msg = create_msg(robot1.id, [pt1])
                    r2_path_msg = create_msg(robot2.id, [pt2])
                    final_msg.paths_list.append(r1_path_msg)
                    final_msg.paths_list.append(r2_path_msg)
                    paths_data_publisher.publish(final_msg)
                    PATH_CREATED = True

rospy.init_node("test_connection_node")
# img_sub = rospy.Subscriber("square_image", Image, img_callback)
objects_sub = rospy.Subscriber("field_objects", FieldObjects_msg, obj_callback)
paths_data_publisher = rospy.Publisher("paths_data", AllPathes, queue_size=1)

rospy.spin()
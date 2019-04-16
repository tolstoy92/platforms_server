#!/usr/bin/env python

import rospy
from platforms_server.msg import IK_Data
from cv_bridge import CvBridge
from vision.Connection import IkSensor


ik_sensor = IkSensor()


def ik_callback(msg_data):
    ik_data = msg_data.ik_data
    ik_sensor.update_ik_data(ik_data)
    print(ik_sensor.is_connection_possible())


rospy.init_node("test_ik_sensor")
img_sub = rospy.Subscriber("ik_data", IK_Data, ik_callback)

rospy.spin()

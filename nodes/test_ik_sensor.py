#!/usr/bin/env python

import rospy
from platforms_server.msg import IK_Data, FieldObjects as FieldObjects_msg
from cv_bridge import CvBridge
from vision.Fileds_objects import IkSensor



ik_sensors = []

def ik_callback(msg_data):
    pass
    # if ik_sensor.fine_tune_connection:
    #     ik_data = msg_data.ik_data
    #     ik_sensor.update_ik_data(ik_data)
    #     print(ik_sensor.is_connection_possible())

def obj_callback(msg_data):
    robots = msg_data.robots
    for robot in robots:
        if robot.fine_tune_connection:
            ik_sensors.append(IkSensor(robot.id))


rospy.init_node("test_ik_sensor")
ik_sub = rospy.Subscriber("ik_data", IK_Data, ik_callback)
# ik_pub = rospy.Publisher("")
objects_sub = rospy.Subscriber("field_objects", FieldObjects_msg, obj_callback)

rospy.spin()

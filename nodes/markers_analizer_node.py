#!/usr/bin/env python

import rospy
from vision.MarkersAnalizer import MarkersAnalizer
from platforms_server.msg import ArucoData, AllPathes, IK_Data


analizer = MarkersAnalizer()
rospy.init_node("robots_controller")
markers_data_sub = rospy.Subscriber("detected_markers", ArucoData, analizer.objects_callback)
paths_data_sub = rospy.Subscriber("paths_data", AllPathes, analizer.paths_callback)
ik_subscriber = rospy.Subscriber("ik_data", IK_Data, analizer.ik_callback)

rospy.spin()

#!/usr/bin/env python

import rospy
from platforms_server.msg import FieldObjects as FieldObjects_msg, Path, AllPathes
from path_planner.Planner import Paths_planner


def callback(msg_data):
    planner.set_robots(msg_data.robots)
    planner.set_targets(msg_data.goals)
    planner.set_obstacles(msg_data.obstacles)
    print(msg_data.obstacles)
    paths = []

    if planner.source == "markers_analizer":
        paths = planner.aruco_target_assignment()
    elif planner.source == "vrep":
        paths = planner.vrep_target_assignment()
    if len(paths):
        msg = prepare_msg(paths)
        paths_data_publisher.publish(msg)


def prepare_msg(paths):
    all_pathes_msg = AllPathes()
    paths_list = []
    for path_id in paths.keys():
        path_msg = Path()
        actual_path = planner.path_to_point_list(paths[path_id])
        path_msg.platform_id = path_id
        path_msg.path_points = list(pt for pt in actual_path)
        paths_list.append(path_msg)
    all_pathes_msg.paths_list = paths_list
    return all_pathes_msg


rospy.init_node("path_planner_node")
planner = Paths_planner()
objects_sub = rospy.Subscriber("field_objects", FieldObjects_msg, callback)
paths_data_publisher = rospy.Publisher("paths_data", AllPathes, queue_size=1)

rospy.spin()

#!/usr/bin/env python


import rospy
from vision.Connection import TConnection, RobotsPair, FineTuneConnection
from platforms_server.msg import AllPathes, Path, FieldObjects as FieldObjects_msg


robots_num = 2
PATH_CREATED = False


def create_msg(id, path):
    path_msg = Path()
    path_msg.platform_id = id
    path_msg.path_points = path
    path_msg.start_connection = True
    path_msg.start_fine_tune_mode = True
    return path_msg


def obj_callback(msg_data):
    global robots_num, PATH_CREATED
    if not PATH_CREATED:
        final_msg = AllPathes()
        if len(msg_data.robots) == robots_num:
            robot1, robot2 = msg_data.robots[0], msg_data.robots[1]
            if robot1.fine_tune_connection and robot2.fine_tune_connection:
                fine_tuner = FineTuneConnection(robot1, robot2)
                robot1_conn_side_idx = fine_tuner.get_robot_connection_side(robot1, robot2)
                conn_line_eq = fine_tuner.get_conn_line_eq(robot1_conn_side_idx)
                riding_line_eq = fine_tuner.get_riding_line_eq(conn_line_eq, robot2.center)
                fine_tune_path = list(fine_tuner.get_riding_points(conn_line_eq, riding_line_eq))
                msg = create_msg(robot2.id, fine_tune_path)
                final_msg.paths_list.append(msg)
                paths_data_publisher.publish(final_msg)
                PATH_CREATED = True


rospy.init_node("fine_tune_node")
objects_sub = rospy.Subscriber("field_objects", FieldObjects_msg, obj_callback)
paths_data_publisher = rospy.Publisher("paths_data", AllPathes, queue_size=1)

rospy.spin()

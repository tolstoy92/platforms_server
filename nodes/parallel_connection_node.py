#!/usr/bin/env python


import rospy
from vision.Connection import ParallelConnection, RobotsPair
from platforms_server.msg import AllPathes, Path, FieldObjects as FieldObjects_msg

robots_num = 2
PATH_CREATED = False


def create_msg(id, path):
    path_msg = Path()
    path_msg.platform_id = id
    path_msg.path_points = path
    path_msg.start_connection = True
    path_msg.start_fine_tune_mode = False
    return path_msg


def obj_callback(msg_data):
    global robots_num, PATH_CREATED
    if not PATH_CREATED:
        final_msg = AllPathes()
        if len(msg_data.robots) == robots_num:
            robot1, robot2 = msg_data.robots[0], msg_data.robots[1]
            if robot1.connection_mode and robot2.connection_mode:
                robots_pair = RobotsPair(robot1, robot2)
                robot1_path_data, robot2_path_data = robots_pair.create_parallel_connection_path()

                robot1_id, robot1_path = robot1_path_data
                robot2_id, robot2_path = robot2_path_data

                r1_path_msg = create_msg(robot1_id, robot1_path)
                r2_path_msg = create_msg(robot2_id, robot2_path)

                final_msg.paths_list.append(r1_path_msg)
                final_msg.paths_list.append(r2_path_msg)
                paths_data_publisher.publish(final_msg)
                PATH_CREATED = True


rospy.init_node("parallel_connection_node")
objects_sub = rospy.Subscriber("field_objects", FieldObjects_msg, obj_callback)
paths_data_publisher = rospy.Publisher("paths_data", AllPathes, queue_size=1)

rospy.spin()

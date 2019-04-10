#!/usr/bin/env python

import rospy
from vision.Connection import TConnection

from platforms_server.msg import AllPathes, Path, FieldObjects as FieldObjects_msg


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
                connection_path_creator = TConnection(robot1, robot2)
                base_robot, riding_robot = connection_path_creator.chose_base_robot()
                base_robot_heading_point = connection_path_creator.find_base_robot_heading_points()
                riding_robot_conn_point = connection_path_creator.get_riding_robot_connection_point()
                riding_robot_path = connection_path_creator.create_riding_robot_connection_path(riding_robot_conn_point)
                # connection_path_creator.check(riding_robot_conn_point, base_robot.center)
                r1_path_msg = create_msg(base_robot.id, [base_robot_heading_point])
                r2_path_msg = create_msg(riding_robot.id, riding_robot_path)
                final_msg.paths_list.append(r1_path_msg)
                final_msg.paths_list.append(r2_path_msg)
                paths_data_publisher.publish(final_msg)
                PATH_CREATED = True

rospy.init_node("test_t_connection_node")
# img_sub = rospy.Subscriber("square_image", Image, img_callback)
objects_sub = rospy.Subscriber("field_objects", FieldObjects_msg, obj_callback)
paths_data_publisher = rospy.Publisher("paths_data", AllPathes, queue_size=1)

rospy.spin()
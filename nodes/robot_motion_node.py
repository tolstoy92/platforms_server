#!/usr/bin/env python

import rospy
from vrep_communicator.VrepCommunicator import Robot, Vrep
import vrep_communicator.vrep_constants as const
from platforms_server.msg import RobotMovement
from vision.Fileds_objects import Point
from math import fabs

def callback(msg_data):
    robot = Robot(msg_data.id)
    velocity = msg_data.velocity
    goal_pos = msg_data.goal
    robot.vrep_con.create_dummy(goal_pos)
    desired_dir = robot.vrep_con.get_object_orientation(robot.handle, goal_pos)
    if fabs(desired_dir - robot.get_robot_orientation()) > 1:
        robot.turn_to_a_point(goal_pos)
    old_error = 0
    error_sum = 0
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        robot_pos = robot.get_robot_position()
        if robot_pos.get_distance_to(goal_pos) < 0.05:
            robot.stop()
            break
        left_velo, right_velo, old_error, error_sum = robot.get_PID_impact(goal_pos, velocity, old_error, error_sum)
        robot.wheel_rotation(left_velo, right_velo)
        rate.sleep()

def stop_func():
    global robot_data
    robot_data.unregister()

rospy.init_node("robot_motion_node")
robot_data = rospy.Subscriber("point_data", RobotMovement, callback)

rospy.on_shutdown(stop_func)
rospy.spin()
#!/usr/bin/env python

import rospy
from vrep_communicator.VrepCommunicator import Vrep
from platforms_server.msg import RobotData, GoalData, ObstacleData, FieldObjects, RobotMovement


def prepare_robot_msg(robots):
    goal_msgs = []
    for id in robots:
        msg = RobotData()
        msg.id = id
        msg.center = robots[id][0]
        msg.direction = robots[id][1]
        msg.corners = robots[id][2]
        goal_msgs.append(msg)
    return goal_msgs


def prepare_goal_msg(goals):
    robot_msgs = []
    for id in goals:
        msg = GoalData()
        msg.id = id
        msg.center = goals[id][0]
        msg.corners = goals[id][1]
        robot_msgs.append(msg)
    return robot_msgs


def prepare_obstacle_msg(obstacles):
    obstacle_msgs = []
    for id in obstacles:
        msg = ObstacleData()
        msg.id = id
        msg.center = obstacles[id][0]
        msg.corners = obstacles[id][1]
        obstacle_msgs.append(msg)
    return obstacle_msgs


def prepare_point_msg(robot, goal, velocity):
    point_data_msg = RobotMovement()
    point_data_msg.id = robot.id
    point_data_msg.goal = goal
    point_data_msg.velocity = velocity
    return point_data_msg


rospy.init_node("vrep_communicator_node")
vrep_data_pub = rospy.Publisher("field_objects", FieldObjects)
point_data_pub = rospy.Publisher("point_data", RobotMovement)
rows = 13
cols = 13
velo = 2

vrep_con = Vrep()
objects_msg = FieldObjects()
point_data_msg = RobotMovement()

robots_data = vrep_con.get_robots_data()
goal_data = vrep_con.get_goal_data()
obstacles_data = vrep_con.get_obstacles_data()
cell_mas = vrep_con.create_mesh(rows, cols)

objects_msg.robots = prepare_robot_msg(robots_data)
objects_msg.goals = prepare_goal_msg(goal_data)
objects_msg.obstacles = prepare_obstacle_msg(obstacles_data)

robot = objects_msg.robots[0]
target = cell_mas[8][3]
point_msg = prepare_point_msg(robot, target, velo)

vrep_con.finish_connection()

point_data_pub.publish(point_msg)
vrep_data_pub.publish(objects_msg)

rospy.spin()

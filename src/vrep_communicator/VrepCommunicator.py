import vrep
import vrep_constants as const
import math
from matplotlib.path import Path
from numpy import array
from vision.Fileds_objects import Point
from math import acos, cos, fabs, pow, pi, sin, sqrt

class Robot():
    def __init__(self, handle):
        self.handle = handle
        self.vrep_con = Vrep()
        if self.vrep_con.client_id == -1:
            raise Exception('Failed to connect to remote API server.')
        self.left_motor_handle = self.vrep_con.get_object_child(self.handle, 1)
        self.right_motor_handle = self.vrep_con.get_object_child(self.handle, 0)
        self.direction_point = self.vrep_con.get_object_child(self.handle, 15)

    def wheel_rotation(self, left_motor_speed, right_motor_speed):
        retCode = vrep.simxSetJointTargetVelocity(self.vrep_con.client_id, self.left_motor_handle, \
                                                  left_motor_speed, vrep.simx_opmode_streaming)
        retCode = vrep.simxSetJointTargetVelocity(self.vrep_con.client_id, self.right_motor_handle, \
                                                  right_motor_speed, vrep.simx_opmode_streaming)

    def get_PID_impact(self, goal_pos, velocity, old_error, error_sum):
        desired_dir = self.vrep_con.get_object_orientation(self.handle, goal_pos)
        error = desired_dir - self.get_robot_orientation()
        error_sum = error_sum + error
        if error_sum < const.iMin:
            error_sum = const.iMin
        elif error_sum > const.iMax:
            error_sum = const.iMax
        print(error)
        up = const.kp * error
        ui = const.ki * error_sum
        ud = const.kd * (error - old_error)
        old_error = error
        u = up + ui + ud
        if u > 0:
            left_u = velocity - fabs(u)
            right_u = velocity
        else:
            left_u = velocity
            right_u = velocity - fabs(u)
        return left_u, right_u, old_error, error_sum

    def get_left_accomodation_circle_pos(self):
        robot_pos = self.get_robot_position()
        norm_vector = self.get_robot_direction_vector()
        left_circle = (robot_pos[0] - const.TURNING_RADIUS * norm_vector[1], robot_pos[1] + const.TURNING_RADIUS * norm_vector[0])
        return left_circle

    def get_right_accomodation_circle_pos(self):
        robot_pos = self.get_robot_position()
        norm_vector = self.get_robot_direction_vector()
        right_circle = (robot_pos[0] + const.TURNING_RADIUS * norm_vector[1], robot_pos[1] - const.TURNING_RADIUS * norm_vector[0])
        return right_circle

    def stop(self):
        self.wheel_rotation(0, 0)

    def get_robot_orientation(self):
        dir_point_pos = self.vrep_con.get_object_position(self.direction_point)
        angle = self.vrep_con.get_object_orientation(self.handle, dir_point_pos)
        return angle

    def get_robot_direction_vector(self):
        robot_pos = self.get_robot_position()
        dir_point_pos = self.vrep_con.get_object_position(self.direction_point)
        dir_vector = (dir_point_pos.x - robot_pos.x, dir_point_pos.y - robot_pos.y)
        vector_mod = sqrt(pow(dir_vector[0], 2) + pow(dir_vector[1], 2))
        norm_dir_vector = (dir_vector[0] / vector_mod, dir_vector[1] / vector_mod)
        return norm_dir_vector

    def get_robot_position(self):
        return self.vrep_con.get_object_position(self.handle)

    def turn_in_a_direction(self, direction_vector):
        robot_position = self.vrep_con.get_object_position(self.handle)
        target_position = (robot_position[0] + direction_vector[0], robot_position[1] + direction_vector[1])
        self.turn_to_a_point(target_position)

    def turn_to_a_point(self, point_pos):
        goal_angle = self.vrep_con.get_object_orientation(self.handle, point_pos)
        angle_difference = goal_angle - self.get_robot_orientation()
        if angle_difference > 180:
            angle_difference = -(360 - angle_difference)
        elif angle_difference < -180:
            angle_difference = 360 + angle_difference
        if angle_difference > 0:
            self.wheel_rotation(-const.ROTATION_SPEED, const.ROTATION_SPEED)
        else:
            self.wheel_rotation(const.ROTATION_SPEED, -const.ROTATION_SPEED)
        while fabs(angle_difference) > 0.5:
            current_angle = self.get_robot_orientation()
            angle_difference = goal_angle - current_angle
            if angle_difference > 180:
                angle_difference = -(360 - angle_difference)
            elif angle_difference < -180:
                angle_difference = 360 + angle_difference
            print("angle_difference = {0:.4}".format(angle_difference))
        self.stop()


class Vrep():
    def __init__(self):
        self.client_id = vrep.simxStart(const.CON_ADDRESS, const.CON_PORT, False, True, \
                                        const.TIMEOUT_IN_MS, const.COMM_THREAD_CYCLE_IN_MS)

    def get_object_handle(self, obj_name):
        ret, handle = vrep.simxGetObjectHandle(self.client_id, obj_name, vrep.simx_opmode_oneshot_wait)
        return handle

    def get_object_child(self, parent_handle, index):
        ret, child_handle = vrep.simxGetObjectChild(self.client_id, \
                            parent_handle, index, vrep.simx_opmode_oneshot_wait)
        return child_handle

    def get_object_position(self, object_handle):
        """
        Function that returns position of object on the scene in V-REP
        """
        res, object_position = vrep.simxGetObjectPosition(self.client_id, object_handle, -1, \
                                                          vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            return Point(object_position[0], object_position[1])
        else:
            print('Remote function call failed with result {0}.'.format(res))
            return ()

    def get_robots_data(self):
        if not (self.get_object_handle(const.ROBOTS_NAMES_TREE) == 0):
            robots_data = dict()
            robots_handles = self.get_object_childs(const.ROBOTS_NAMES_TREE)
            for robot in robots_handles:
                robot_boundary_points = self.get_boundary_points(robot)
                robot_position = self.get_object_position(robot)
                robot_direction = self.get_robot_direction_vector(robot)
                robots_data[robot] = [robot_position, robot_direction, robot_boundary_points]
            return robots_data
        else:
            return {}

    def get_goal_data(self):
        if not (self.get_object_handle(const.TARGETS_NAMES_TREE) == 0):
            goal_data = dict()
            goal_handles = self.get_object_childs(const.TARGETS_NAMES_TREE)
            for goal in goal_handles:
                goal_boundary_points = self.get_boundary_points(goal)
                goal_position = self.get_object_position(goal)
                goal_data[goal] = [goal_position, goal_boundary_points]
            return goal_data
        else:
            return {}

    def get_obstacles_data(self):
        if not (self.get_object_handle(const.OBSTACLES_NAMES_TREE) == 0):
            if const.WITH_DYNAMIC_OBSTACLES:
                pass
            else:
                obstacles_data = dict()
                obstacle_handles = self.get_object_childs(const.OBSTACLES_NAMES_TREE)
                for obstacle in obstacle_handles:
                    obstacle_boundary_points = self.get_boundary_points(obstacle)
                    obstacle_position = self.get_object_position(obstacle)
                    obstacles_data[obstacle] = [obstacle_position, obstacle_boundary_points]
                return obstacles_data
        else:
            return {}

    def get_boundary_points(self, object_handle):
        """
        Function that returns boundary points of object's (obstacle) boundary box
        """
        points = []
        obstacle_position = self.get_object_position(object_handle)
        ret, orient = vrep.simxGetObjectOrientation(self.client_id, object_handle, -1, \
                                                    vrep.simx_opmode_blocking)
        ret, x_1 = vrep.simxGetObjectFloatParameter(self.client_id, object_handle, 15, \
                                                    vrep.simx_opmode_blocking)
        ret, y_1 = vrep.simxGetObjectFloatParameter(self.client_id, object_handle, 16, \
                                                    vrep.simx_opmode_blocking)
        ret, x_2 = vrep.simxGetObjectFloatParameter(self.client_id, object_handle, 18, \
                                                    vrep.simx_opmode_blocking)
        ret, y_2 = vrep.simxGetObjectFloatParameter(self.client_id, object_handle, 19, \
                                                    vrep.simx_opmode_blocking)
        angle = orient[2]
        # Extension of boundaries, so that the robots moves without collisions
        x_1 = x_1 - 0.3
        x_2 = x_2 + 0.3
        y_1 = y_1 - 0.3
        y_2 = y_2 + 0.3


        p_1 = (x_1 * cos(angle) - y_1 * sin(angle) + obstacle_position.x, y_1 * \
               cos(angle) + x_1 * sin(angle) + obstacle_position.y)
        points.append(Point(*p_1))
        p_2 = (x_1 * cos(angle) - y_2 * sin(angle) + obstacle_position.x, y_2 * \
               cos(angle) + x_1 * sin(angle) + obstacle_position.y)
        points.append(Point(*p_2))
        p_3 = (x_2 * cos(angle) - y_2 * sin(angle) + obstacle_position.x, y_2 * \
               cos(angle) + x_2 * sin(angle) + obstacle_position.y)
        points.append(Point(*p_3))
        p_4 = (x_2 * cos(angle) - y_1 * sin(angle) + obstacle_position.x, y_1 * \
               cos(angle) + x_2 * sin(angle) + obstacle_position.y)
        points.append(Point(*p_4))
        return points

    def get_object_childs(self, obj_name):
        """
        Function that return handles of object's childs from the V-REP scene.
        This function is useful when the exact number of objects is unknown
        """
        index = 0
        children_list = []
        child = 0
        parent_handle = self.get_object_handle(obj_name)
        while child != -1:
            res, child = vrep.simxGetObjectChild(self.client_id, parent_handle, index, vrep.simx_opmode_blocking)
            if res == vrep.simx_return_ok:
                children_list.append(child)
                index = index + 1
            else:
                print('Remote fucntion get_object_childs call failed.')
                return []
        del children_list[len(children_list) - 1]
        return children_list

    def finish_connection(self):
        vrep.simxFinish(-1)

    def get_robot_direction_vector(self, robot_handle):
        direction_point = self.get_object_child(robot_handle, 15)
        robot_position = self.get_object_position(robot_handle)
        dir_point_position = self.get_object_position(direction_point)
        direction_vector = (dir_point_position.x - robot_position.x, \
                            dir_point_position.y - robot_position.y)
        direction_vector_mod = sqrt(direction_vector[0] ** 2 \
                                         + direction_vector[1] ** 2)
        norm_direction_vector = (direction_vector[0] / direction_vector_mod, \
                                 direction_vector[1] / direction_vector_mod)
        return Point(*norm_direction_vector)

    def get_object_orientation(self, object_handle, target_pos):
        object_pos = self.get_object_position(object_handle)
        direction_vector = (target_pos.x - object_pos.x, target_pos.y - object_pos.y)
        direction_vector_mod = sqrt(direction_vector[0] ** 2 + direction_vector[1] ** 2)
        norm_direction_vector = (direction_vector[0] / direction_vector_mod, \
                                 direction_vector[1] / direction_vector_mod)
        if norm_direction_vector[1] != 0:
            angle = acos(norm_direction_vector[0]) * 180 / pi * fabs(norm_direction_vector[1]) / \
                norm_direction_vector[1]
        else:
            angle = acos(norm_direction_vector[0]) * 180 / pi
        return angle

    def create_mesh(self, row_num, col_num):
        x_min = -5
        x_max = 5
        y_min = -5
        y_max = 5
        x_range = x_max - x_min
        y_range = y_max - y_min
        cell_x_size = float(x_range) / float(col_num)
        cell_y_size = float(y_range) / float(row_num)
        cells_list = []
        for row in range(row_num):
            cells_list.append([])
            for col in range(col_num):
                x = x_min + cell_x_size * (0.5 + col)
                y = y_min + cell_y_size * (0.5 + row)
                cell_pos = Point(x, y)
                cells_list[row].append(cell_pos)
        return cells_list

    def set_object_position(self, objectHandle, pos):
        retCode = vrep.simxSetObjectPosition(self.client_id, objectHandle, -1, (pos.x, pos.y, 0.3), vrep.simx_opmode_oneshot)

    def create_dummy(self, pos):
        retCode, dummyHandle = vrep.simxCreateDummy(self.client_id, 0.2, (128, 128, 128), vrep.simx_opmode_blocking)
        self.set_object_position(dummyHandle, pos)
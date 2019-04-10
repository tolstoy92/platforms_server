from vision.geometry_utils import *
from constants.robot_constants import MARKER_SIZE, CONNECTION_DISTANCE, ROBOT_H, ROBOT_SIZE
from math import sqrt
from sympy import symbols, solve

class ParallelConnection():
    def __init__(self, robot1, robot2):
        self.robot1, self.robot2 = robot1, robot2
        self.connection_distanse_in_meters = CONNECTION_DISTANCE + ROBOT_SIZE
        self.connection_distanse_in_pix = self.connection_distanse_in_meters / \
                                          get_meters_in_pix(MARKER_SIZE, self.robot1.corners)

    def is_robots_too_close(self):
        half_robots_size_in_pix = (ROBOT_SIZE/2) / get_meters_in_pix(MARKER_SIZE, self.robot1.corners)
        if get_distance_between_points(self.robot1.center, self.robot2.center) <\
                half_robots_size_in_pix * 2 + self.connection_distanse_in_pix:
            return True
        else:
            return False

    def find_point1(self):
        x1, y1 = self.robot1.center.x, self.robot1.center.y
        x2, y2 = self.robot2.center.x, self.robot2.center.y
        d = self.connection_distanse_in_pix
        l = get_distance_between_points(self.robot1.center, self.robot2.center)
        c = sqrt(l ** 2 - d ** 2)

        X, Y = symbols('x y')
        eq1 = (x1 - X)**2 + (y1 - Y)**2 - c**2
        eq2 = (x2 - X)**2 + (y2 - Y)**2 - d**2

        pt1, _ = solve([eq1, eq2], X, Y, set=False)
        pt1 = Point(int(pt1[0]), int(pt1[1]))

        connection_pt1 = self.get_connection_point_for_robots(self.robot1, pt1)
        return connection_pt1

    def find_point2(self, pt1):
        x1, y1 = self.robot1.center.x, self.robot1.center.y
        x2, y2 = self.robot2.center.x, self.robot2.center.y

        Ox = (x1 + x2)/2
        Oy = (y1 + y2)/2

        X = int(2*Ox - pt1.x)
        Y = int(2*Oy - pt1.y)

        connection_pt2 = Point(X, Y)

        return connection_pt2

    def get_connection_point_for_robots(self, robot, pt):
        x1, y1 = robot.center.x, robot.center.y
        x2, y2 = pt.x, pt.y
        return Point(int((x1 + x2)/2), int((y1+y2)/2))

    # def create_connection_path

    def check(self, pt1, pt2):
        dist = get_distance_between_points(pt1, pt2)
        dist = dist * get_meters_in_pix(MARKER_SIZE, self.robot1.corners)
        print('distance: {}'.format(dist))


class TConnection(ParallelConnection):
    def __init__(self, robot1, robot2):
        self.robot1, self.robot2  = robot1, robot2
        self.connection_distanse_in_meters = CONNECTION_DISTANCE + ROBOT_H/2 + ROBOT_SIZE/2
        self.connection_distanse_in_pix = self.connection_distanse_in_meters / \
                                          get_meters_in_pix(MARKER_SIZE, self.robot1.corners)

    def chose_base_robot(self):
        # base robot is < | > robot in T connection
        # riding robot is < - > robot in T connection
        connection_angle = 90
        robot1_angle = get_angle_by_3_points(self.robot1.direction, self.robot2.center, self.robot1.center)
        robot2_angle = get_angle_by_3_points(self.robot2.direction, self.robot1.center, self.robot2.center)
        if abs(robot1_angle - connection_angle) <= abs(robot2_angle - connection_angle):
            self.base_robot, self.riding_robot = self.robot1, self.robot2
        else:
            self.base_robot, self.riding_robot = self.robot2, self.robot1
        return self.base_robot, self.riding_robot

    def find_base_robot_heading_points(self):
        base_robot_heading_point1 = Point(is_heading=True)
        base_robot_heading_point2 = Point(is_heading=True)

        robots_eq = get_line_equation(self.robot1.center, self.robot2.center)
        base_robot_heading_eq = get_perpendicular_line_equation(robots_eq, self.base_robot.center)

        k, b = base_robot_heading_eq

        base_robot_heading_point1.x = self.base_robot.center.x + 100
        base_robot_heading_point1.y = base_robot_heading_point1.x * k + b

        base_robot_heading_point2.x = self.base_robot.center.x - 100
        base_robot_heading_point2.y = base_robot_heading_point2.x * k + b

        heading_point = self.chose_base_robot_heading_point(base_robot_heading_point1, base_robot_heading_point2)
        return heading_point

    def chose_base_robot_heading_point(self, pt1, pt2):
        angle_to_pt1 = get_angle_by_3_points(pt1, self.base_robot.direction, self.base_robot.center)
        angle_to_pt2 = get_angle_by_3_points(pt2, self.base_robot.direction, self.base_robot.center)
        if angle_to_pt1 <= angle_to_pt2:
            return pt1
        else:
            return pt2

    def get_connection_line_eq(self):
        connection_line_eq = get_line_equation(self.base_robot.center, self.riding_robot.center)
        return connection_line_eq

    def get_riding_robot_connection_point(self):
        connection_line_eq = self.get_connection_line_eq()
        k, b = connection_line_eq
        X = symbols('x')
        d = self.connection_distanse_in_pix
        x, y = self.base_robot.center.x, self.base_robot.center.y
        eq = (X - x)**2 + ((k*X + b) - y)**2 - d**2

        pts_lst = solve([eq], X)

        pt1_x, pt2_x = pts_lst

        pt1_x, pt2_x = pt1_x[0], pt2_x[0]

        pt1_y = k * pt1_x + b
        pt2_y = k * pt2_x + b

        pt1 = Point(pt1_x, pt1_y)
        pt2 = Point(pt2_x, pt2_y)

        if get_distance_between_points(self.riding_robot.center, pt1) <= get_distance_between_points(self.riding_robot.center, pt2):
            riding_robot_connection_point = pt1
        else:
            riding_robot_connection_point = pt2

        return riding_robot_connection_point

    def create_riding_robot_connection_path(self, conn_point):
        middle_point = Point(int((self.riding_robot.center.x + conn_point.x)/2), int((self.riding_robot.center.y + conn_point.y)/2))
        add_pt1 = Point(int((self.riding_robot.center.x + middle_point.x)/2), int((self.riding_robot.center.y + middle_point.y)/2))
        add_pt2 = middle_point
        add_pt3 = Point(int((middle_point.x + conn_point.x)/2), int((middle_point.y + conn_point.y)/2))
        return [add_pt1, add_pt2, add_pt3, conn_point]

    def check(self, pt1, pt2):
        dist = get_distance_between_points(pt1, pt2)
        dist = dist * get_meters_in_pix(MARKER_SIZE, self.robot1.corners)
        print('distance: {}'.format(dist))

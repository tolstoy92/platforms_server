from vision.geometry_utils import *
from vision.vision_constants import MARKER_SIZE, CONNECTION_DISTANCE
from math import degrees, sqrt, acos, tan, radians, cos, sin, atan, asin
from sympy import symbols, solve

class ParallelConnection():
    def __init__(self, robot1, robot2):
        self.robot1, self.robot2 = robot1, robot2
        self.connection_distanse_in_meters = CONNECTION_DISTANCE + ROBOT_SIZE
        self.connection_distanse_in_pix = self.connection_distanse_in_meters / \
                                          get_meters_in_pix(MARKER_SIZE, self.robot1.corners)
        self.connection_figure = None

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

    def check(self, pt1, pt2):
        dist = get_distance_between_points(pt1, pt2)
        dist = dist * get_meters_in_pix(MARKER_SIZE, self.robot1.corners)
        print('distance: {}'.format(dist))


class TConnection(ParallelConnection):
    def __init__(self, robot1, robot2):
        self.robot1, self.robot2  = robot1, robot2
        self.connection_distanse_in_meters = 0.02 + ROBOT_H/2
        self.connection_distanse_in_pix = self.connection_distanse_in_meters / \
                                          get_meters_in_pix(MARKER_SIZE, self.robot1.corners)

    def set_base_robot(self, robot):
        self.base_robot = robot
        if self.base_robot is self.robot1:
            self.riding_robot = self.robot2
        else:
            self.riding_robot = self.robot1

    def find_base_robot_heading_points(self):
        base_robot_heading_point1 = Point()
        base_robot_heading_point2 = Point()

        robots_eq = get_line_equation(self.robot1.center, self.robot2.center)
        base_robot_heading_eq = get_perpendicular_line_equation(robots_eq, self.base_robot.center)

        base_robot_heading_point1.x = self.base_robot.center.x + 10
        base_robot_heading_point1.y = base_robot_heading_point1.x * base_robot_heading_eq[0] + base_robot_heading_eq[1]

        base_robot_heading_point2.x = self.base_robot.center.x - 10
        base_robot_heading_point2.y = base_robot_heading_point2.x * base_robot_heading_eq[0] + base_robot_heading_eq[1]
        return base_robot_heading_point1, base_robot_heading_point2

    def chose_base_robot_heading_point(self, pt1, pt2):
        angle_to_pt1 = get_angle_by_3_points(pt1, self.base_robot.direction, self.base_robot.center)
        angle_to_pt2 = get_angle_by_3_points(pt2, self.base_robot.direction, self.base_robot.center)

        if angle_to_pt1 <= angle_to_pt2:
            base_robot_heading_point = pt1
        else:
            base_robot_heading_point = pt2

        return base_robot_heading_point

    def get_connection_line_eq(self):
        connection_line_eq = get_line_equation(self.base_robot.center, self.riding_robot.center)
        return connection_line_eq

    def find_riding_robot_connection_point(self, connection_line_eq):
        riding_robot_connection_point = Point()
        k, b = connection_line_eq
        sin_a = sqrt(k**2 / (1 + k**2))
        dy = sin_a * self.connection_distanse_in_pix
        if self.base_robot.center.y >= self.riding_robot.center.y:
            riding_robot_connection_point.y = self.base_robot.center.y - dy
        else:
            riding_robot_connection_point.y = self.base_robot.center.y + dy
        try:
            riding_robot_connection_point.x = (riding_robot_connection_point.y - b) / k
        except:
            riding_robot_connection_point.x = self.base_robot.center.x
        return riding_robot_connection_point


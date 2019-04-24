import rospy
import numpy as np
from math import sqrt, acos, degrees
from sympy import symbols, solve

# from constants.vision_constants import *

CAMERA_INDEX = rospy.get_param('CAMERA_INDEX')
IMAGE_SIZE = rospy.get_param('IMAGE_SIZE')
CV_WAITKEY = rospy.get_param('CV_WAITKEY')

LOW_BOUNDS = rospy.get_param('LOW_BOUNDS')    # bound fot ompl remapping
HIGH_BOUNDS = rospy.get_param('HIGH_BOUNDS')    # bound fot ompl remapping

mtx = np.array(rospy.get_param('mtx'))
dist = np.array(rospy.get_param('dist'))

def get_line_cntr(pt1, pt2):
    line_cntr = list(map(lambda x: x, ((pt1.x + pt2.x) / 2, (pt1.y + pt2.y) / 2)))
    return Point(line_cntr[0], line_cntr[1])

def get_distance_between_points(pt1, pt2):
    return sqrt((pt1.x - pt2.x) ** 2 + (pt1.y - pt2.y) ** 2)

def get_vector_coords(start_pt, end_pt):
    vec_coords = ((end_pt.x - start_pt.x), (end_pt.y - start_pt.y))
    return Point(*vec_coords)

def get_angle_cos(vec1, vec2):
    try:
        angle_cos = ((vec1.x * vec2.x) + (vec1.y * vec2.y)) / (
                    sqrt(vec1.x ** 2 + vec1.y ** 2) * sqrt(vec2.x ** 2 + vec2.y ** 2))
    except:
        angle_cos = 1
    if angle_cos > 1:
        angle_cos = 1
    return angle_cos

def get_angle_by_3_points(pt1, pt2, angle_point):
    vec1 = get_vector_coords(angle_point, pt1)
    vec2 = get_vector_coords(angle_point, pt2)
    angle_cos = get_angle_cos(vec1, vec2)
    angle_radians = acos(angle_cos)
    angle_degrees = degrees(angle_radians)
    return angle_degrees

def get_meters_in_pix(marker_size, marker_corners):
    pix_size = get_distance_between_points(marker_corners[0], marker_corners[1])
    return marker_size/pix_size

def get_angle_sign_pt_to_line(point, line_point1, line_point2):
    """ This func needed for computing angle sign
     If we need to turn our robot clockwise, it well be < + >.
     Else: < - >. """
    projection = get_projection_of_pt_on_line(point, line_point1, line_point2)
    if line_point1.x <= line_point2.x:
        if point.y >= projection.y:
            sign = 1
        else:
            sign = -1
    else:
        if point.y >= projection.y:
            sign = -1
        else:
            sign = 1
    return sign

def get_projection_of_pt_on_line(point, line_point1, line_point2):
    """ This func helps  to find angle sign."""
    projection = Point(-1, -1)
    projection.x = point.x
    if (line_point2.x - line_point1.x) != 0:
        projection.y = (projection.x - line_point1.x) * (line_point2.y - line_point1.y) / \
                       (line_point2.x - line_point1.x) + line_point1.y
    else:
        projection.y = (projection.x - line_point1.x) * (line_point2.y - line_point1.y) / 1 + line_point1.y
    return projection

def get_line_equation(pt1, pt2):
    # y = kx + b
    a = np.array([[pt1.x, 1], [pt2.x, 1]])
    b = np.array([pt1.y, pt2.y])
    k, b = np.linalg.solve(a, b)
    return k, b

def get_perpendicular_line_equation(eq1, pt):
    k, b = eq1
    perpendic_k = -1/k
    perpendic_b = pt.y - perpendic_k*pt.x
    return perpendic_k, perpendic_b

def get_crossing_lines_point(eq1, eq2):
    k1, b1 = eq1
    k2, b2 = eq2
    x = (b2 - b1) / (k1 - k2)
    y = k1*x + b1
    return Point(x, y)

def get_points_by_distance_to_point_on_line(line_eq, point_on_line, distance):

    k, b = line_eq
    X = symbols('x')
    d = distance
    x, y = point_on_line.x, point_on_line.y
    eq = (X - x)**2 + ((k*X + b) - y)**2 - d**2

    pts_lst = solve([eq], X)

    pt1_x, pt2_x = pts_lst
    pt1_x, pt2_x = pt1_x[0], pt2_x[0]

    pt1_y = k * pt1_x + b
    pt2_y = k * pt2_x + b

    pt1 = Point(abs(pt1_x), abs(pt1_y))
    pt2 = Point(abs(pt2_x), abs(pt2_y))

    print(pt1, pt2)

    # if get_distance_between_points(self.riding_robot.center, pt1) <= \
    #    get_distance_between_points(self.riding_robot.center, pt2):
    #     riding_robot_connection_point = pt1
    # else:
    #     riding_robot_connection_point = pt2

    #
    # k, b = line_eq
    # x, y = point_on_line.x, point_on_line.y
    # d = distance
    #
    # a_ = 1 + k**2
    # b_ = (2*k*b) + (2*x) - (2*y*k)
    # c_ = x**2 + y**2 + 2*y*b + b**2 - d**2
    #
    # D = b_**2 - (4*a_*c_)
    #
    # print('conn_D', D)
    #
    #
    # x1, x2 = (-b_ + sqrt(D))/(2*a_), (-b_ - sqrt(D))/(2*a_)
    # y1, y2 = k*x1 + b, k*x2 + b
    #
    # return Point(int(x1), int(y1)), Point(int(x2), int(y2))
    return pt1, pt2


class Point:
    def __init__(self, x=None, y=None, is_heading=False):
        self.x = x
        self.y = y
        self.is_heading = is_heading

    def __str__(self):
        return str(self.x) + " " + str(self.y)

    def __repr__(self):
        return str(self.x) + " " + str(self.y)

    def __call__(self, type_of=int):
        return type_of(self.x), type_of(self.y)

    def set_x(self, x):
        self.x = x

    def set_y(self, y):
        self.y = y

    def set_xy(self, x, y):
        self.x = x
        self.y = y

    def get_xy(self):
        return self.x, self.y

    def is_empty(self):
        if self.x and self.y: return False
        else: return True

    def is_heading_point(self):
        if self.heading_point:
            return True
        else:
            return False

    def get_distance_to(self, point):
        return sqrt((point.x - self.x) ** 2 + (point.y - self.y) ** 2)

    def remap_to_ompl_coord_system(self):
        ompl_x = self.x * (HIGH_BOUNDS - LOW_BOUNDS) / IMAGE_SIZE + LOW_BOUNDS
        ompl_y = self.y * (HIGH_BOUNDS - LOW_BOUNDS) / IMAGE_SIZE + LOW_BOUNDS
        return Point(ompl_x, ompl_y)

    def remap_to_img_coord_system(self):
        img_x = int((self.x + HIGH_BOUNDS) * IMAGE_SIZE / (HIGH_BOUNDS - LOW_BOUNDS))
        img_y = int((self.y + HIGH_BOUNDS) * IMAGE_SIZE / (HIGH_BOUNDS - LOW_BOUNDS))
        return Point(img_x, img_y)


class RealWorldPoint():
    def __init__(self, x=None, y=None, z=None):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return str(self.x) + " " + str(self.y) + " " + str(self.z)

    def __repr__(self):
        return str(self.x) + " " + str(self.y) + " " + str(self.z)

    def __call__(self):
        return self.x, self.y, self.z

    def update_real_world_position(self, x, y, z):
        self.x, self.y, self.z = x, y, z

    def get_distance_xy_to(self, point):
        return sqrt((point.x - self.x) ** 2 + (point.y - self.y) ** 2)

    def get_distance_xy_between_points(self, pt1, pt2):
        return sqrt((pt1.x - pt2.x) ** 2 + (pt1.y - pt2.y) ** 2)

    def is_empty(self):
        if self.x and self.y and self.z: return False
        else: return True

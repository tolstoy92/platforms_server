from math import sqrt, acos, degrees
# from vision.Fileds_objects import Point
from vision.vision_constants import IMAGE_SIZE, HIGH_BOUNDS, LOW_BOUNDS, EPS, ANGLE_EPS, MARKER_SIZE, ROBOT_SIZE, WHEEL_SIZE



def get_line_cntr(pt1, pt2):
    line_cntr = list(map(lambda x: x, ((pt1.x + pt2.x) / 2, (pt1.y + pt2.y) / 2)))
    return Point(line_cntr[0], line_cntr[1])

def get_distance_between_points(pt1, pt2):
    print(pt1.x, pt2.x)
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

class Point:
    def __init__(self, x=None, y=None):
        self.x = x
        self.y = y

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

    def is_empty(self):
        if self.x and self.y: return False
        else: return True

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
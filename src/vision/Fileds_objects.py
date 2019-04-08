from numpy import array
from copy import deepcopy
from matplotlib.path import Path
from math import degrees, sqrt, acos, tan, radians, cos, sin, atan, asin
from platforms_server.msg import RobotData, GoalData, ObstacleData, Point2d, Wheel as Wheel_msg
from vision.vision_constants import IMAGE_SIZE, HIGH_BOUNDS, LOW_BOUNDS, EPS, ANGLE_EPS, MARKER_SIZE, ROBOT_SIZE, WHEEL_SIZE
from vision.geometry_utils import *


class Marker:
    def __init__(self, id, corners, real_world_position):
        self.__marker_size = MARKER_SIZE
        self.id = id
        self.corners = list(Point(xy.x, xy.y) for xy in corners)
        self.center = self.get_center()
        self.real_world_position = real_world_position

    def get_corners(self):
        return self.corners

    def remap_marker_to_ompl_coord_system(self):
        ompl_corners = list(xy.remap_to_ompl_coord_system() for xy in self.corners)
        return ompl_corners

    def get_center(self):
        front_left_corner = self.corners[0]
        behind_right_corner = self.corners[2]
        center = get_line_cntr(front_left_corner, behind_right_corner)
        return center

    def points_to_list(self, points_list):
        return [(pt.x, pt.y) for pt in points_list]

    def get_ompl_path(self):
        ompl_corners = self.remap_marker_to_ompl_coord_system()
        points = self.points_to_list(ompl_corners)
        return Path(array(points))

    def update_real_world_position(self, position):
        x, y, z = position.x, position.y, position.z
        if self.real_world_position.is_empty:
            self.real_world_position = RealWorldPoint(x, y, z)
        else:
            self.real_world_position.update_real_world_position(x, y, z)

class Goal(Marker):
    def __init__(self, id, corners, real_world_position):
        Marker.__init__(self, id, corners, real_world_position)

    def __repr__(self):
        return "Goal:\n\tid: {}\n\tposition: {}".format(self.id,
                                                        self.center)

    def prepare_msg(self):
        msg = GoalData()
        msg.id = self.id
        msg.center = self.center
        msg.corners = self.corners
        return msg

    def update_data(self, corners, position):
        self.corners = corners
        self.real_world_position = position


class Robot(Marker):
    def __init__(self, id, corners, real_world_position):
        Marker.__init__(self, id, corners, real_world_position)
        self.__robot_size = ROBOT_SIZE
        self.__marker_size = MARKER_SIZE

        self.direction = self.get_direction()
        self.path = []
        self.connection_path = []

        self.actual_point = Point()
        self.next_point = Point()
        self.finish_point = Point()
        self.finish_heading_point = Point()

        self.angle_to_actual_point = None
        self.angle_to_next_point = None
        self.actual_angle = None
        self.final_angle_to_heading_point = 45
        self.angle_to_heading_point = None

        self.wheels_pair = Wheels_pair()

        self.trajectory = []
        self.path_created = False
        self.move_forward = False
        self.self_rotation = False

        self.on_finish_point = False
        self.on_finish_heading = False

    def __repr__(self):
        return "Robot:\n\tid: {}\n\tposition: {}\n\treal world position: {}\n\t" \
               "direction: {}\n\tmove forward: {}\n\trotation: {}\n\t" \
               "angle to actual point: {}\n\tangle to next point: {}\n\ton point: {}\n\t" \
                "on finish: {}\n\ton finish heading: {}".format(self.id,
                                                               self.center,
                                                               self.real_world_position,
                                                               self.direction,
                                                               self.move_forward,
                                                               self.self_rotation,
                                                               self.angle_to_actual_point,
                                                               self.angle_to_next_point,
                                                               self.on_point(self.actual_point),
                                                               self.on_finish_point,
                                                               self.on_finish_heading)

    def prepare_msg(self):
        msg = RobotData()
        msg.id = self.id
        msg.center = self.center
        msg.direction = self.get_direction()
        msg.corners = self.corners
        msg.path_created = self.path_created
        msg.real_world_position = self.real_world_position
        if self.path_created:
            msg.path = self.path
        if not self.actual_point.is_empty():
            msg.actual_point = self.actual_point
        if self.angle_to_actual_point:
            msg.angle_to_actual_point = self.angle_to_actual_point
        if not self.next_point.is_empty():
            msg.next_point = self.next_point
        if self.actual_angle:
            msg.actual_angle = self.actual_angle
        if not self.finish_heading_point.is_empty():
            msg.finish_heading_point = self.finish_heading_point
        if self.angle_to_heading_point:
            msg.angle_to_heading_point = self.angle_to_heading_point
        msg.rotation = self.self_rotation
        msg.move = self.move_forward
        msg.on_finish_point = self.on_finish_point
        msg.on_finish_heading = self.on_finish_heading
        if not self.wheels_pair.is_empty():
            msg.wheels_pair = self.wheels_pair
        return msg

    def update_data(self, corners, position):
        self.update_real_world_position(position)

        self.wheels_pair.get_wheels_centers(self.corners, self.center, self.direction)
        self.wheels_pair.update_wheel_edges(self.corners, self.center)

        if not self.on_finish_point and not self.on_finish_heading:
            self.update_corners(corners)
            self.update_position()
            self.update_direction()
            if self.path_created:
                if not self.finish_point:
                    self.finish_point = self.path[-1]
                if not self.finish_heading_point:
                    self.create_finish_heading_point(self.final_angle_to_heading_point)
                if self.actual_point.is_empty():
                    self.update_actual_point()
                self.update_angles()
                if self.on_point(self.actual_point):
                    self.update_actual_point()
                else:
                    if abs(self.angle_to_actual_point) < ANGLE_EPS:
                        self.move()
                    else:
                        self.rotation()
        elif self.on_finish_point and not self.on_finish_heading:
            self.update_corners(corners)
            self.update_position()
            self.update_direction()
            # self.angle_to_heading_point = get_angle_by_3_points(self.direction, self.finish_heading_point, self.center)
            # if abs(self.angle_to_heading_point) >= 3:
            #     self.actual_point = self.finish_heading_point
            #     self.update_angles()
            #     self.rotation()
            # else:
            self.on_finish_heading = True
        else:
            self.stop()

    def update_position(self):
        if self.center:
            if get_distance_between_points(self.center, self.get_center()) > 5:
                self.trajectory.append([self.center.x, self.center.y])
        self.center = self.get_center()

    def update_corners(self, corners):
        self.corners = list(Point(xy.x, xy.y) for xy in corners)

    def update_direction(self):
        self.direction = self.get_direction()

    def on_point(self, point):
        if point:
            distance_to_point = get_distance_between_points(self.center, point)
            return distance_to_point <= EPS
        else:
            return False

    def update_actual_point(self):
        if self.path_created:
            try:
                self.actual_point = self.path.pop(0)
                try:
                    self.next_point = self.path[0]
                except:
                    self.next_point = Point()
            except:
                self.on_finish_point = True

    def create_finish_heading_point(self, angle_to_point):
        delta = 50
        if self.finish_point:
            if angle_to_point >= 0 and angle_to_point < 90:
                y = self.finish_point.y - delta
                x = self.finish_point.x + tan(radians(angle_to_point))*delta
            elif angle_to_point == 90:
                y = self.finish_point.y
                x = self.finish_point.x + delta
            elif angle_to_point > 90 and angle_to_point <= 180:
                y = self.finish_point.y + delta
                x = self.finish_point.x + tan(radians(angle_to_point)) * delta
            elif angle_to_point == -90:
                y = self.finish_point.y
                x = self.finish_point.x - delta
            elif angle_to_point > -90 and angle_to_point < 0:
                y = self.finish_point.y + delta
                x = self.finish_point.x - tan(radians(angle_to_point)) * delta
            elif angle_to_point > -180 and angle_to_point < -90:
                y = self.finish_point.y - delta
                x = self.finish_point.x - tan(radians(angle_to_point)) * delta
            self.finish_heading_point = Point(int(x), int(y))

    def update_angles(self):
        if not self.actual_point.is_empty():
            sign_angle_to_actual_point = get_angle_sign_pt_to_line(self.direction, self.center, self.actual_point)
            self.angle_to_actual_point = get_angle_by_3_points(self.direction, self.actual_point, self.center) * sign_angle_to_actual_point
            self.actual_angle = self.angle_to_actual_point
        if not self.next_point.is_empty():
            sign_angle_to_next_point = get_angle_sign_pt_to_line(self.direction, self.center, self.actual_point)
            self.angle_to_next_point = get_angle_by_3_points(self.direction, self.next_point, self.center) * sign_angle_to_next_point
            self.angle_to_next_point = self.angle_to_next_point
        else:
            self.angle_to_next_point = None

    def rotation(self):
        self.move_forward = False
        self.self_rotation = True

    def move(self):
        self.move_forward = True
        self.self_rotation = False

    def stop(self):
        self.move_forward = False
        self.self_rotation = False

    def set_path(self, path_msg):
        if not self.path_created:
            tmp_path = []
            for pt in path_msg:
                tmp_path.append(Point(pt.x, pt.y))
            self.path = deepcopy(tmp_path)
            self.path_created = True

    def get_direction(self):
        front_left_corner = self.corners[0]
        front_right_corner = self.corners[1]
        direction_point = get_line_cntr(front_left_corner, front_right_corner)
        return direction_point

    def get_position(self):
        return self.center


class Wheel():
    def __init__(self, front_side_point=None, back_side_point=None, center=None):
        self.front_side_point = front_side_point
        self.back_side_point = back_side_point
        self.center = center

    def is_empty(self):
        if self.front_side_point and self.back_side_point and self.center: return False
        else: return True

    def __call__(self, *args, **kwargs):
        return self.center, self.front_side_point, self.back_side_point

class Wheels_pair():
    def __init__(self, rigth_wheel=Wheel(), left_wheel=Wheel()):
        self.right_wheel = rigth_wheel
        self.left_wheel = left_wheel

    def is_empty(self):
        if self.right_wheel.is_empty() or self.left_wheel.is_empty():
            return True
        else:
            return False

    def get_wheels_centers(self, robots_marker_corners, robot_center, robot_direction):
        axis_pt_right, axis_pt_left = self.get_axis_points(robots_marker_corners)

        angle_right, angle_left = self.get_wheels_angles(axis_pt_right, axis_pt_left, robot_center)

        wheel_h_in_pix = (ROBOT_SIZE / 2) / get_meters_in_pix(MARKER_SIZE, robots_marker_corners)

        d_x_right, d_y_right, d_x_left, d_y_left = self.get_dx_dy_of_wheels(angle_right, angle_left, wheel_h_in_pix)

        x_right, y_right, x_left, y_left = self.compute_wheels_centers(robot_center, robot_direction,
                                                                       d_x_right, d_y_right, d_x_left, d_y_left)

        self.right_wheel.center = Point(x_right, y_right)
        self.left_wheel.center = Point(x_left, y_left)

        if not robot_direction.y <= robot_center.y:
            if robot_direction.x > robot_center.x:
                self.right_wheel, self.left_wheel = self.left_wheel, self.right_wheel
            else:
                if self.right_wheel.center.x > self.left_wheel.center.x:
                    self.right_wheel, self.left_wheel = self.left_wheel, self.right_wheel

    def get_axis_points(self, robots_marker_corners):
        axis_pt_right = get_line_cntr(robots_marker_corners[1], robots_marker_corners[2])
        axis_pt_left = get_line_cntr(robots_marker_corners[0], robots_marker_corners[3])
        return axis_pt_right, axis_pt_left

    def get_wheels_angles(self, axis_pt_right, axis_pt_left, robot_center):
        if axis_pt_right.x >= axis_pt_left.x:
            angle_right = get_angle_by_3_points(axis_pt_right,
                                                Point(robot_center.x + 10, robot_center.y),
                                                robot_center)
            angle_left = get_angle_by_3_points(axis_pt_left,
                                               Point(robot_center.x - 10, robot_center.y),
                                               robot_center)
        else:
            angle_right = get_angle_by_3_points(axis_pt_right,
                                                Point(robot_center.x - 10, robot_center.y),
                                                robot_center)
            angle_left = get_angle_by_3_points(axis_pt_left,
                                               Point(robot_center.x + 10, robot_center.y),
                                               robot_center)
        return angle_right, angle_left

    def get_dx_dy_of_wheels(self, angle_right, angle_left, wheel_h_in_pix):
        # dx, dy of wheels centers regarding robot's center
        d_x_right = cos(radians(angle_right)) * wheel_h_in_pix
        d_y_right = sin(radians(angle_right)) * wheel_h_in_pix

        d_x_left = cos(radians(angle_left)) * wheel_h_in_pix
        d_y_left = sin(radians(angle_left)) * wheel_h_in_pix

        return d_x_right, d_y_right, d_x_left, d_y_left

    def compute_wheels_centers(self, robot_center, robot_direction, d_x_right, d_y_right, d_x_left, d_y_left):
        if robot_direction.y <= robot_center.y and robot_direction.x >= robot_center.x:
            x_right = robot_center.x + d_x_right
            y_right = robot_center.y + d_y_right
            x_left = robot_center.x - d_x_left
            y_left = robot_center.y - d_y_left
        elif robot_direction.y > robot_center.y and robot_direction.x >= robot_center.x:
            x_right = robot_center.x + d_x_right
            y_right = robot_center.y - d_y_right
            x_left = robot_center.x - d_x_left
            y_left = robot_center.y + d_y_left
        elif robot_direction.y > robot_center.y and robot_direction.x < robot_center.x:
            x_right = robot_center.x - d_x_right
            y_right = robot_center.y - d_y_right
            x_left = robot_center.x + d_x_left
            y_left = robot_center.y + d_y_left
        elif robot_direction.y <= robot_center.y and robot_direction.x < robot_center.x:
            x_right = robot_center.x + d_x_right
            y_right = robot_center.y - d_y_right
            x_left = robot_center.x - d_x_left
            y_left = robot_center.y + d_y_left
        else:
            x_right, x_left, y_right, y_left = None, None, None, None

        return x_right, y_right, x_left, y_left

    def update_wheel_edges(self, robots_marker_corners, robot_center):
        half_wheel_size_in_pix = (WHEEL_SIZE / 2) / get_meters_in_pix(MARKER_SIZE, robots_marker_corners)
        half_robot_size_in_pix = (ROBOT_SIZE / 2) / get_meters_in_pix(MARKER_SIZE, robots_marker_corners)

        wheel = self.right_wheel

        k = self.get_wheel_angle_sign(wheel, robot_center)

        angle_a = degrees(atan(half_wheel_size_in_pix / half_robot_size_in_pix))
        angle_b = get_angle_by_3_points(wheel.center, Point(robot_center.x + k*50, robot_center.y), robot_center)
        if angle_b > 90:
            angle_b = 180 - angle_b
        angle = abs(angle_a) + abs(angle_b)

        dx = cos(radians(angle)) * sqrt(half_wheel_size_in_pix ** 2 + half_robot_size_in_pix ** 2)
        dy = abs(sin(radians(angle)) * sqrt(half_wheel_size_in_pix ** 2 + half_robot_size_in_pix ** 2))

        self.compute_wheels_sides(robot_center, dx, dy)

    def get_wheel_angle_sign(self, wheel, robot_center):
        if self.right_wheel.center.x >= self.left_wheel.center.x:
            if wheel.center.x <= robot_center.x:
                k = -1
            else:
                k = 1
        else:
            if wheel.center.x <= robot_center.x:
                k = 1
            else:
                k = -1
        return k

    def compute_wheels_sides(self, robot_center, dx, dy):
        # wheel's sides are wheel's frontside and backside
        if self.right_wheel.center.x >= self.left_wheel.center.x:
            if self.right_wheel.center.y <= self.left_wheel.center.y:
                self.right_wheel.front_side_point = Point(robot_center.x + dx, robot_center.y - dy)
                ddx = self.right_wheel.center.x - self.right_wheel.front_side_point.x
                ddy = self.right_wheel.center.y - self.right_wheel.front_side_point.y
                self.right_wheel.back_side_point = Point(self.right_wheel.center.x + ddx,
                                                         self.right_wheel.center.y + ddy)

                self.left_wheel.back_side_point = Point(robot_center.x - dx, robot_center.y + dy)
                self.left_wheel.front_side_point = Point(self.left_wheel.center.x - ddx,
                                                         self.left_wheel.center.y - ddy)
            else:
                self.right_wheel.back_side_point = Point(robot_center.x + dx, robot_center.y + dy)
                ddx = self.right_wheel.center.x - self.right_wheel.back_side_point.x
                ddy = self.right_wheel.center.y - self.right_wheel.back_side_point.y
                self.right_wheel.front_side_point = Point(self.right_wheel.center.x + ddx,
                                               self.right_wheel.center.y + ddy)

                self.left_wheel.front_side_point = Point(robot_center.x - dx, robot_center.y - dy)
                self.left_wheel.back_side_point = Point(self.left_wheel.center.x - ddx,
                                                        self.left_wheel.center.y - ddy)
        else:
            if self.left_wheel.center.y <= self.right_wheel.center.y:
                self.left_wheel.back_side_point = Point(robot_center.x + dx, robot_center.y - dy)
                ddx = self.left_wheel.center.x - self.left_wheel.back_side_point.x
                ddy = self.left_wheel.center.y - self.left_wheel.back_side_point.y
                self.left_wheel.front_side_point = Point(self.left_wheel.center.x + ddx,
                                                         self.left_wheel.center.y + ddy)

                self.right_wheel.front_side_point = Point(robot_center.x - dx, robot_center.y + dy)
                self.right_wheel.back_side_point = Point(self.right_wheel.center.x - ddx,
                                                         self.right_wheel.center.y - ddy)
            else:
                self.left_wheel.front_side_point = Point(robot_center.x + dx, robot_center.y + dy)
                ddx = self.left_wheel.center.x - self.left_wheel.front_side_point.x
                ddy = self.left_wheel.center.y - self.left_wheel.front_side_point.y
                self.left_wheel.back_side_point = Point(self.left_wheel.center.x + ddx,
                                                        self.left_wheel.center.y + ddy)

                self.right_wheel.back_side_point = Point(robot_center.x - dx, robot_center.y - dy)
                self.right_wheel.front_side_point = Point(self.right_wheel.center.x - ddx,
                                                          self.right_wheel.center.y - ddy)

class PathPoint(Point):
    def __init__(self, x=None, y=None):
        Point.__init__(self, x, y)

class HeadingPoint(Point):
    def __init__(self, x=None, y=None):
        Point.__init__(self, x, y)


class Obstacle:
    def __init__(self, id, marker_list):
        self.id = id
        unsorted_points = self.get_unsorted_obstacles_points(marker_list)
        self.geometric_center = self.compute_geometric_center(marker_list)
        self.obstacle_points = self.sort_obstacles_points(unsorted_points)

    def __repr__(self):
        return "Obstacle:\n\tid: {}\n\tcenter: {}\n\tcorners: {}".format(self.id,
                                                                         self.geometric_center,
                                                                         self.obstacle_points)

    def prepare_msg(self):
        msg = ObstacleData()
        msg.id = self.id
        msg.center = self.geometric_center
        msg.corners = self.obstacle_points
        return msg

    def get_obstacle_points(self):
        return self.obstacle_points

    def get_unsorted_obstacles_points(self, markers_list, marker_border_points_num=2):
        obstacle_border_points = []
        geometric_center = self.compute_geometric_center(markers_list)
        if len(markers_list) > 1:
            for marker in markers_list:
                distances_to_geometric_center = {}
                for pt in marker.get_corners():
                    distance = get_distance_between_points(geometric_center, pt)
                    while distance in distances_to_geometric_center:
                        distance += 0.001
                    distances_to_geometric_center[distance] = pt
                for num in range(marker_border_points_num):
                    obstacle_border_points.append(distances_to_geometric_center.pop(
                                                  max(list(distances_to_geometric_center.keys()))))
        else:
            if len(markers_list):
                obstacle_border_points = list(pt for pt in markers_list[0].get_corners())

        return obstacle_border_points

    def compute_geometric_center(self, markers_list):
        all_points = []
        for marker in markers_list:
            all_points += marker.get_corners()

        max_x = max([pt.x for pt in all_points])
        min_x = min([pt.x for pt in all_points])
        max_y = max([pt.y for pt in all_points])
        min_y = min([pt.y for pt in all_points])

        geometric_center = Point((max_x + min_x)/2, (max_y + min_y)/2)
        return geometric_center

    def get_geometric_center(self):
        return self.geometric_center

    def sort_obstacles_points(self, corners):
        init_corner = corners.pop(0)
        sorted_corners = [init_corner]
        while len(corners) != 1:
            distances = list(get_distance_between_points(init_corner, corner) for corner in corners)
            init_corner = corners.pop(distances.index(min(distances)))
            sorted_corners.append(init_corner)
        sorted_corners.append(corners[0])
        return sorted_corners

    def points_to_list(self, points_list):
        return [(pt.x, pt.y) for pt in points_list]

    def remap_points_to_ompl_coord_system(self):
        ompl_corners = list(xy.remap_to_ompl_coord_system() for xy in self.obstacle_points)
        return ompl_corners

    def get_ompl_path(self):
        ompl_points = self.remap_points_to_ompl_coord_system()
        points = self.points_to_list(ompl_points)
        return Path(array(points))


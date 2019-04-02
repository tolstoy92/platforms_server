from numpy import array
from copy import deepcopy
from matplotlib.path import Path
from math import degrees, sqrt, acos, tan, radians, cos, sin, atan
from platforms_server.msg import RobotData, GoalData, ObstacleData, Point2d
from vision.vision_constants import IMAGE_SIZE, HIGH_BOUNDS, LOW_BOUNDS, EPS, ANGLE_EPS, MARKER_SIZE, ROBOT_SIZE, WHEEL_SIZE


class Point:
    def __init__(self, x, y):
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
    def __init__(self, x, y, z):
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
        center = Point(*self.get_line_cntr(front_left_corner, behind_right_corner))
        return center

    def get_line_cntr(self, pt1, pt2):
        line_cntr = tuple(map(lambda x: x, ((pt1.x + pt2.x) / 2, (pt1.y + pt2.y) / 2)))
        return line_cntr[0], line_cntr[1]

    def points_to_list(self, points_list):
        return [(pt.x, pt.y) for pt in points_list]

    def get_ompl_path(self):
        ompl_corners = self.remap_marker_to_ompl_coord_system()
        points = self.points_to_list(ompl_corners)
        return Path(array(points))

    def update_real_world_position(self, position):
        x, y, z = position.x, position.y, position.z
        if not self.real_world_position:
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
        self.actual_point = None
        self.angle_to_actual_point = None
        self.next_point = None
        self.angle_to_next_point = None
        self.actual_angle = None
        self.finish_point = None
        self.finish_heading_point = None
        self.final_angle_to_heading_point = 45
        self.angle_to_heading_point = None

        self.right_wheel = None
        self.right_wheel_edge1, self.right_wheel_edge2 = None, None

        self.left_wheel = None
        self.left_wheel_edge1, self.left_wheel_edge2 = None, None

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
        if self.actual_point:
            msg.actual_point = self.actual_point
        if self.angle_to_actual_point:
            msg.angle_to_actual_point = self.angle_to_actual_point
        if self.next_point:
            msg.next_point = self.next_point
        if self.actual_angle:
            msg.actual_angle = self.actual_angle
        if self.finish_heading_point:
            msg.finish_heading_point = self.finish_heading_point
        if self.angle_to_heading_point:
            msg.angle_to_heading_point = self.angle_to_heading_point
        if self.right_wheel and self.left_wheel:
            msg.right_wheel, msg.left_wheel = self.right_wheel, self.left_wheel
        msg.rotation = self.self_rotation
        msg.move = self.move_forward
        msg.on_finish_point = self.on_finish_point
        msg.on_finish_heading = self.on_finish_heading
        try:
            if self.right_wheel_edge1 and self.right_wheel_edge2 and self.left_wheel_edge1 and self.left_wheel_edge2:
                msg.right_wheel_edge1 = self.right_wheel_edge1
                msg.right_wheel_edge2 = self.right_wheel_edge2
                msg.left_wheel_edge1 = self.left_wheel_edge1
                msg.left_wheel_edge2 = self.left_wheel_edge2
        except:
            pass
        return msg

    def update_data(self, corners, position):
        self.update_real_world_position(position)
        self.get_wheels_position()
        self.update_wheel_edges()
        # print(self.right_wheel_edge1, self.right_wheel_edge2)
        # print(self.left_wheel_edge1, self.left_wheel_edge2)
        if not self.on_finish_point and not self.on_finish_heading:
            self.update_corners(corners)
            self.update_position()
            self.update_direction()
            if self.path_created:
                if not self.finish_point:
                    self.finish_point = self.path[-1]
                if not self.finish_heading_point:
                    self.create_finish_heading_point(self.final_angle_to_heading_point)
                if not self.actual_point:
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
            self.angle_to_heading_point = self.get_angle_to_point(self.finish_heading_point)
            if abs(self.angle_to_heading_point) >= 3:
                self.actual_point = self.finish_heading_point
                self.update_angles()
                self.rotation()
            else:
                self.on_finish_heading = True
        else:
            self.stop()

    def update_position(self):
        if self.center:
            if self.get_distance_between_pts(self.center, self.get_center()) > 5:
                self.trajectory.append([self.center.x, self.center.y])
        self.center = self.get_center()

    def update_corners(self, corners):
        self.corners = list(Point(xy.x, xy.y) for xy in corners)

    def update_direction(self):
        self.direction = self.get_direction()

    def on_point(self, point):
        if point:
            distance_to_point = Robot.get_distance_between_pts(self.center, point)
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
                    self.next_point = None
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
        if self.actual_point:
            self.angle_to_actual_point = self.get_angle_to_point(self.actual_point)
            self.actual_angle = self.angle_to_actual_point
        if self.next_point:
            self.angle_to_next_point = self.get_angle_to_point(self.next_point)
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
        direction_point = Point(*self.get_line_cntr(front_left_corner, front_right_corner))
        return direction_point

    @staticmethod
    def get_distance_between_pts(pt1, pt2):
        return sqrt((pt2.x - pt1.x) ** 2 + (pt2.y - pt1.y) ** 2)

    def get_position(self):
        return self.center

    def get_projection_of_direction_pt_on_trajectory(self, destination_point):
        """ This func helps  to find angle sign."""
        projection = Point(-1, -1)
        projection.x = self.direction.x
        if (destination_point.x - self.center.x) != 0:
            projection.y = (projection.x - self.center.x) * (destination_point.y - self.center.y) / \
                           (destination_point.x - self.center.x) + self.center.y
        else:
            projection.y = (projection.x - self.center.x) * (destination_point.y - self.center.y) / 1 + self.center.y
        return projection

    def get_angle_to_point(self, point):
        dir_vec = Point((self.direction.x - self.center.x), (self.direction.y - self.center.y))
        trajectory_vec = Point((point.x - self.center.x), (point.y - self.center.y))
        scalar_multiply = dir_vec.x * trajectory_vec.x + dir_vec.y * trajectory_vec.y
        dir_vec_module = sqrt(dir_vec.x ** 2 + dir_vec.y ** 2)
        trajectory_vec_module = sqrt(trajectory_vec.x ** 2 + trajectory_vec.y ** 2)
        if (trajectory_vec_module * dir_vec_module) != 0:
            cos_a = scalar_multiply / (trajectory_vec_module * dir_vec_module)
            angle = round(degrees(acos(min(1, max(cos_a, -1)))))
        else:
            angle = 0
        angle = self.get_angle_sign(point, angle)
        return angle

    def get_angle_cos(self, vec1, vec2):
        try:
            angle_cos = ((vec1.x * vec2.x) + (vec1.y * vec2.y)) / (
                        sqrt(vec1.x ** 2 + vec1.y ** 2) * sqrt(vec2.x ** 2 + vec2.y ** 2))
        except:
            angle_cos = 1
        if angle_cos > 1:
            angle_cos = 1
        return angle_cos

    def get_vector_coords(self, start_pt, end_pt):
        vec_coords = ((end_pt.x - start_pt.x), (end_pt.y - start_pt.y))
        return Point(*vec_coords)

    def get_angle_by_3_points(self, pt1, pt2, angle_point):
        vec1 = self.get_vector_coords(angle_point, pt1)
        vec2 = self.get_vector_coords(angle_point, pt2)
        angle_cos = self.get_angle_cos(vec1, vec2)
        angle_radians = acos(angle_cos)
        angle_degrees = degrees(angle_radians)
        return angle_degrees

    def get_angle_sign(self, destination_point, angle):
        """ This func needed for computing angle sign
         If we need to turn our robot clockwise, it well be < + >.
         Else: < - >. """
        projection = self.get_projection_of_direction_pt_on_trajectory(destination_point)
        if self.center.x <= destination_point.x:
            if self.direction.y >= projection.y:
                result_angle = angle
            else:
                result_angle = -angle
        else:
            if self.direction.y >= projection.y:
                result_angle = -angle
            else:
                result_angle = angle
        return result_angle

    def get_meters_in_pix(self):
        pix_size = self.get_distance_between_pts(self.corners[0], self.corners[1])
        meters = self.__marker_size
        return meters/pix_size

    def get_wheels_position(self):
        axis_pt_right = Point(*self.get_line_cntr(self.corners[1], self.corners[2]))
        axis_pt_left = Point(*self.get_line_cntr(self.corners[0], self.corners[3]))

        if axis_pt_right.x >= axis_pt_left.x:
            angle_right = self.get_angle_by_3_points(axis_pt_right, Point(self.center.x + 10, self.center.y), self.center)
            angle_left = self.get_angle_by_3_points(axis_pt_left, Point(self.center.x - 10, self.center.y), self.center)
        else:
            angle_right = self.get_angle_by_3_points(axis_pt_right, Point(self.center.x - 10, self.center.y), self.center)
            angle_left = self.get_angle_by_3_points(axis_pt_left, Point(self.center.x + 10, self.center.y), self.center)

        wheel_h_in_pix = (self.__robot_size / 2) / self.get_meters_in_pix()

        d_x_right = cos(radians(angle_right)) * wheel_h_in_pix
        d_y_right = sin(radians(angle_right)) * wheel_h_in_pix

        d_x_left = cos(radians(angle_left)) * wheel_h_in_pix
        d_y_left = sin(radians(angle_left)) * wheel_h_in_pix

        if self.direction.y <= self.center.y and self.direction.x >= self.center.x:
            x_right = self.center.x + d_x_right
            y_right = self.center.y + d_y_right
            x_left = self.center.x - d_x_left
            y_left = self.center.y - d_y_left
        elif self.direction.y > self.center.y and self.direction.x >= self.center.x:
            x_right = self.center.x + d_x_right
            y_right = self.center.y - d_y_right
            x_left = self.center.x - d_x_left
            y_left = self.center.y + d_y_left
        elif self.direction.y > self.center.y and self.direction.x < self.center.x:
            x_right = self.center.x - d_x_right
            y_right = self.center.y - d_y_right
            x_left = self.center.x + d_x_left
            y_left = self.center.y + d_y_left
        elif self.direction.y <= self.center.y and self.direction.x < self.center.x:
            x_right = self.center.x + d_x_right
            y_right = self.center.y - d_y_right
            x_left = self.center.x - d_x_left
            y_left = self.center.y + d_y_left
        else:
            x_right, x_left, y_right, y_left = None, None, None, None


        right_wheel_pos = Point(x_right, y_right)
        left_wheel_pos = Point(x_left, y_left)

        if self.direction.y <= self.center.y:
            self.right_wheel, self.left_wheel = right_wheel_pos, left_wheel_pos
        else:
            if self.direction.x < self.center.x:
                self.right_wheel, self.left_wheel = right_wheel_pos, left_wheel_pos
            elif self.direction.x > self.center.x:
                self.right_wheel, self.left_wheel = left_wheel_pos, right_wheel_pos
            else:
                if right_wheel_pos.x < left_wheel_pos:
                    self.right_wheel, self.left_wheel = right_wheel_pos, left_wheel_pos
                else:
                    self.right_wheel, self.left_wheel = left_wheel_pos, right_wheel_pos

    def update_wheel_edges(self):
        half_wheel_size_in_pix = (WHEEL_SIZE / 2) / self.get_meters_in_pix()
        half_robot_size_in_pix = (ROBOT_SIZE / 2) / self.get_meters_in_pix()

        wheel = self.right_wheel
        if self.right_wheel.x >= self.left_wheel.x:
            if wheel.x <= self.center.x:
                d = -1
            else:
                d = 1
        else:
            if wheel.x <= self.center.x:
                d = 1
            else:
                d = -1

        angle_a = degrees(atan(half_wheel_size_in_pix / half_robot_size_in_pix))
        angle_b = self.get_angle_by_3_points(wheel, Point(self.center.x + d*50, self.center.y), self.center)
        if angle_b > 90:
            angle_b = 180 - angle_b
        angle = abs(angle_a) + abs(angle_b)

        dx = cos(radians(angle)) * sqrt(half_wheel_size_in_pix ** 2 + half_robot_size_in_pix ** 2)
        dy = abs(sin(radians(angle)) * sqrt(half_wheel_size_in_pix ** 2 + half_robot_size_in_pix ** 2))

        if self.right_wheel.x >= self.left_wheel.x:
            if self.right_wheel.y <= self.left_wheel.y:
                self.right_wheel_edge1 = Point(self.center.x + dx, self.center.y - dy)
                ddx = self.right_wheel.x - self.right_wheel_edge1.x
                ddy = self.right_wheel.y - self.right_wheel_edge1.y
                self.right_wheel_edge2 = Point(self.right_wheel.x + ddx, self.right_wheel.y + ddy)

                self.left_wheel_edge2 = Point(self.center.x - dx, self.center.y + dy)
                self.left_wheel_edge1 = Point(self.left_wheel.x - ddx, self.left_wheel.y - ddy)
            else:
                self.right_wheel_edge2 = Point(self.center.x + dx, self.center.y + dy)
                ddx = self.right_wheel.x - self.right_wheel_edge2.x
                ddy = self.right_wheel.y - self.right_wheel_edge2.y
                self.right_wheel_edge1 = Point(self.right_wheel.x + ddx, self.right_wheel.y + ddy)

                self.left_wheel_edge1 = Point(self.center.x - dx, self.center.y - dy)
                self.left_wheel_edge2 = Point(self.left_wheel.x - ddx, self.left_wheel.y - ddy)
        else:
            if self.left_wheel.y <= self.right_wheel.y:
                self.left_wheel_edge2 = Point(self.center.x + dx, self.center.y - dy)
                ddx = self.left_wheel.x - self.left_wheel_edge2.x
                ddy = self.left_wheel.y - self.left_wheel_edge2.y
                self.left_wheel_edge1 = Point(self.left_wheel.x + ddx, self.left_wheel.y + ddy)

                self.right_wheel_edge1 = Point(self.center.x - dx, self.center.y + dy)
                self.right_wheel_edge2 = Point(self.right_wheel.x - ddx, self.right_wheel.y - ddy)
            else:
                self.left_wheel_edge1 = Point(self.center.x + dx, self.center.y + dy)
                ddx = self.left_wheel.x - self.left_wheel_edge1.x
                ddy = self.left_wheel.y - self.left_wheel_edge1.y
                self.left_wheel_edge2 = Point(self.left_wheel.x + ddx, self.left_wheel.y + ddy)

                self.right_wheel_edge2 = Point(self.center.x - dx, self.center.y - dy)
                self.right_wheel_edge1 = Point(self.right_wheel.x - ddx, self.right_wheel.y - ddy)


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
                    distance = self.get_distance_between_pts(geometric_center, pt)
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

    def get_distance_between_pts(self, pt1, pt2):
        return sqrt((pt2.x - pt1.x) ** 2 + (pt2.y - pt1.y) ** 2)

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
            distances = list(self.get_distance_between_pts(init_corner, corner) for corner in corners)
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


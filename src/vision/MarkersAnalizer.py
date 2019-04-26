import rospy
from math import sqrt
# from constants.robot_constants import EPS
from vision.Fileds_objects import Robot, Goal, Marker, RealWorldPoint
from platforms_server.msg import FieldObjects as FieldObjects_msg, IK_Data

EPS = rospy.get_param('EPS')

class MarkersAnalizer:
    def __init__(self):
        self.robots = []
        self.goals = []
        self.objects = []
        self.publisher = rospy.Publisher("field_objects", FieldObjects_msg, queue_size=1)

    def prepare_msgs(self, msg_data):
        msg = FieldObjects_msg()
        self.update_fields_object_by_id(msg_data)
        msg.robots = list(robot.prepare_msg() for robot in self.robots)
        msg.goals = list(goal.prepare_msg() for goal in self.goals)
        return msg

    def objects_callback(self, msg_data):
        msg = self.prepare_msgs(msg_data)
        self.publisher.publish(msg)

    def paths_callback(self, msg_data):
        for path in msg_data.paths_list:
            robot_id = path.platform_id
            start_connection = path.start_connection
            start_fine_tune_mode = path.start_fine_tune_mode
            path_points = path.path_points
            if robot_id in list(robot.id for robot in self.robots):
                robot = list(filter(lambda robot: robot.id == robot_id, self.robots))[0]
                robot.set_path(path_points)
                robot.start_connection = start_connection
                robot.start_fine_tune_mode = start_fine_tune_mode

    def update_fields_object_by_id(self, msg_data):
        ids = [object.id for object in msg_data.markers]
        corners = [object.corners for object in msg_data.markers]
        positions = [object.real_world_position for object in msg_data.markers]
        markers_list = list(Marker(marker_id, marker_corners, RealWorldPoint(position.x, position.y, position.z)) for \
                            marker_id, marker_corners, position in list(zip(ids, corners, positions)))
        self.parse_fields_objects_by_id(markers_list)

    def ik_callback(self, msg_data):
        print()

    def get_robots(self):
        return self.robots

    def get_goals(self):
        return self.goals

    def set_pathes(self, pathes):
        self.pathes = pathes

    def update_robots_data(self, marker):
        self.update_data(marker, self.robots, Robot)

    def update_goals_data(self, marker):
        self.update_data(marker, self.goals, Goal)

    def update_data(self, marker, objects_list, object):
        m_id = marker.id
        corners = marker.corners
        position = marker.real_world_position
        checking_list = [marker.id for marker in objects_list]
        if m_id in checking_list:
            obj = list(filter(lambda obj: obj.id == m_id, objects_list))[0]
            obj.update_data(corners, position)
            obj.update_real_world_position(position)
        else:
            objects_list.append(object(m_id, corners, position))

    def parse_fields_objects_by_id(self, markers_list):
        for marker in markers_list:
            if len(str(marker.id)) == 1:
                self.update_robots_data(marker)
            else:
                self.update_goals_data(marker)

    def set_goals_id_from_platform_id(self, goals_dict):
        for (platform_id, goal_id) in list(zip(self.robots.keys(), goals_dict.keys())):
            self.goals[platform_id] = goals_dict[goal_id]

    def on_position(self, robot_position, target_position):
        on_target_point = False
        distance = self.get_distance_between_pts(robot_position, target_position)
        if distance <= EPS: on_target_point = True
        return on_target_point

    def get_distance_between_pts(self, pt1, pt2):
        return sqrt((pt2.x - pt1.x) ** 2 + (pt2.y - pt1.y) ** 2)

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from platforms_server.msg import FieldObjects as FieldObjects_msg, AllPathes
from vision.vision_constants import IMAGE_SIZE, MAP_ROWS, MAP_COLUMNS
from vision.Fileds_objects import Point


class Visualizer():
    def __init__(self):
        self.bridge = CvBridge()
        self.IMG = None
        self.fields_objects = None
        self.pathes = None
        self.RUN = True
        self.path_lst = []
        rospy.init_node("visualizer_node")
        img_sub = rospy.Subscriber("square_image", Image, self.img_callback)
        field_objects_sub = rospy.Subscriber("field_objects", FieldObjects_msg, self.objects_callback)
        cv2.namedWindow("image", cv2.WINDOW_AUTOSIZE)

    def shutdown(self):
        print("The end!")

    def img_callback(self, data):
        if not rospy.is_shutdown() and self.RUN:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                self.IMG = cv_image
                self.draw_objects()
                cv2.imshow("image", self.IMG)
                if cv2.waitKey(3) & 0xFF == 27:
                    self.RUN = not self.RUN
            except CvBridgeError as e:
                print(e)
        else:
            cv2.destroyAllWindows()
            rospy.on_shutdown(self.shutdown)

    def draw_point(self, point, color=(255, 0, 100), size=3):
        cv2.circle(self.IMG, (int(point.x), int(point.y)), 1, color, thickness=size)

    def objects_callback(self, data):
        self.fields_objects = data

    def clear_paths(self):
        self.pathes = None

    def clear_field_objects(self):
        self.fields_objects = None

    def draw_objects(self):
        if self.fields_objects:
            for robot in self.fields_objects.robots:
                if not robot.on_finish_point or not robot.on_finish_heading:
                    self.draw_point(robot.center)
                    for pt in robot.path:
                        self.draw_point(pt, size=8)
                    self.draw_point(robot.direction, color=(50, 50, 250), size=5)
                    cv2.line(self.IMG, (int(robot.center.x), int(robot.center.y)),
                             (int(robot.direction.x), int(robot.direction.y)), color=(100, 100, 255), thickness=3)
                    cv2.putText(self.IMG, str(robot.id), (int(robot.center.x) + 5, int(robot.center.y)),
                                cv2.FONT_HERSHEY_PLAIN, 2,
                                (255, 100, 60), 3)
                    if robot.actual_point:
                        self.draw_crest(robot.actual_point, color=(100, 255, 50))
                        cv2.putText(self.IMG, str(robot.id), (int(robot.actual_point.x) + 5, int(robot.actual_point.y)), cv2.FONT_HERSHEY_PLAIN, 2,
                                    (255, 100, 60), 3)
                    if robot.next_point:
                        self.draw_crest(robot.next_point, color=(255, 50, 50))
                        #cv2.line(self.IMG, (int(robot.center.x), int(robot.center.y)),
                        #         (int(robot.actual_point.x), int(robot.actual_point.y)), color=(255, 100, 255), thickness=1)
                    if robot.finish_heading_point:
                        self.draw_point(robot.finish_heading_point)
                    if robot.right_wheel and robot.left_wheel:
                        self.draw_point(robot.right_wheel, color=(255, 255, 255), size=8)
                        self.draw_point(robot.left_wheel, color=(255, 255, 255))

                        # print(robot.left_wheel_edge1)

                        # self.draw_point(robot.right_wheel_edge1, color=(50, 250, 50), size=20)
                        # self.draw_point(robot.right_wheel_edge2, color=(50, 250, 50), size=10)
                        #
                        # self.draw_point(robot.left_wheel_edge1, color=(50, 50, 250), size=20)
                        # self.draw_point(robot.left_wheel_edge2, color=(50, 50, 250), size=10)

                        # cv2.line(self.IMG, (int(robot.right_wheel_edge1.x), int(robot.right_wheel_edge1.y)),
                        #          (int(robot.right_wheel_edge2.x), int(robot.right_wheel_edge2.y)), (0, 255, 0), 2)
                        # cv2.line(self.IMG, (int(robot.left_wheel_edge1.x), int(robot.left_wheel_edge1.y)),
                        #      (int(robot.left_wheel_edge2.x), int(robot.left_wheel_edge2.y)), (0, 255, 0), 2)

            for obstacle in self.fields_objects.obstacles:
                self.draw_point(obstacle.center)
                for c in obstacle.corners:
                    self.draw_point(c, size=5)
            for goal in self.fields_objects.goals:
                self.draw_point(goal.center)
                for c in goal.corners:
                    self.draw_point(c, size=5)

    def draw_crest(self, point, color=(0, 0, 255)):
        line_size = 10
        top = (int(point.x), int(point.y - line_size))
        bot = (int(point.x), int(point.y + line_size))
        left = (int(point.x) - line_size, int(point.y))
        right = (int(point.x) + line_size, int(point.y))
        cv2.line(self.IMG, top, bot, color, thickness=3)
        cv2.line(self.IMG, left, right, color, thickness=3)

    def draw_paths(self):
        if self.pathes:
            print(self.pathes)
            for path in self.pathes.paths_list:
                for pt in path.path_points:
                    self.draw_point(pt, size=5)

    def draw_rectangle(self, center, width):
        points = []
        points.append((int(center.x - width // 2), int(center.y - width // 2)))
        points.append((int(center.x + width // 2), int(center.y - width // 2)))
        points.append((int(center.x + width // 2), int(center.y + width // 2)))
        points.append((int(center.x - width // 2), int(center.y + width // 2)))
        for i in range(len(points)):
            cv2.line(self.IMG, points[i], points[i - 1], (255, 200, 40), 2)


    def start_spin(self):
        rospy.spin()


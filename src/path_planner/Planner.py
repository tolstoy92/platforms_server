from time import time
from ompl import base as ob
from ompl import geometric as og
from ompl.util import OMPL_ERROR
from path_planner import planner_constants as const
from functools import partial
from vision.Fileds_objects import Point
import numpy as np
from matplotlib.path import Path
from random import choice


class Paths_planner():
    def __init__(self):
        self.robots = {}
        self.targets = {}
        self.obstacles = {}
        self.source = None

    def multiple_paths_planning(self):
        # print('Program started')
        # print('Connected to remote API server')
        # print("Robots number: {0}".format(len(self.robots)))
        robots_paths, estimated_time = self.target_assignment()
        return robots_paths, estimated_time

    def vrep_target_assignment(self):

        """
        Target assignment and path planning function for group of robots
        """
        paths = {}

        for robot_id in self.robots.keys():
            tmp_targets = self.targets.copy()
            tmp_robots = self.robots.copy()
            target_id = choice(tmp_targets.keys())
            target_position = tmp_targets[target_id].center
            robot_position = tmp_robots[robot_id].center
            full_obstacles = self.get_full_obstacles(robot_id, target_id)
            paths[robot_id] = self.plan(robot_position, target_position, const.PLANNER_RANGE, full_obstacles)
            tmp_targets.pop(target_id)
        return paths

    def aruco_target_assignment(self):
        """
        Target assignment and path planning function for group of robots
        """
        paths = {}
        for robot_id in self.robots.keys():
            if not self.robots[robot_id].path_created:
                target_id = const.platform_target[robot_id]
                tmp_targets = self.targets.copy()
                tmp_robots = self.robots.copy()
                if target_id in list(tmp_targets.keys()):
                    img_target_position = tmp_targets[target_id].center
                    target_position = Point(img_target_position.x, img_target_position.y).remap_to_ompl_coord_system().get_xy()
                    img_robot_position = tmp_robots[robot_id].center
                    robot_position = Point(img_robot_position.x,img_robot_position.y).remap_to_ompl_coord_system().get_xy()
                    full_obstacles = self.get_full_obstacles(robot_id, target_id)
                    paths[robot_id] = self.plan(robot_position, target_position, const.PLANNER_RANGE, full_obstacles)
        return paths

    def get_full_obstacles(self, actual_robot_id, actual_target_id):
        robots_obstacles = self.robots_as_obstacles(actual_robot_id)
        targets_obstacles = self.targets_as_obstacles(actual_target_id)
        obstalces = self.get_obstacles_from_any_objects(self.obstacles)
        full_obstacles = obstalces + robots_obstacles + targets_obstacles
        return full_obstacles

    def get_obstacles_from_any_objects(self, objects_dict):
        full_obstacles = []
        for obstacle in objects_dict.values():
            if self.source == "markers_analizer":
                corners_list = list(Point(xy.x, xy.y).remap_to_ompl_coord_system().get_xy() for xy in obstacle.corners)
            else:
                corners_list = list(Point(xy.x, xy.y).get_xy() for xy in obstacle.corners)
            path = Path(np.array(corners_list))
            full_obstacles.append(path)
        return full_obstacles

    def robots_as_obstacles(self, actual_robot_id):
        tmp_robots = self.robots.copy()
        tmp_robots.pop(actual_robot_id)
        robots_as_obstacles = self.get_obstacles_from_any_objects(tmp_robots)
        return robots_as_obstacles

    def targets_as_obstacles(self, actual_target_id):
        tmp_targets = self.targets.copy()
        tmp_targets.pop(actual_target_id)
        targets_as_obstacles = self.get_obstacles_from_any_objects(tmp_targets)
        return targets_as_obstacles

    def get_line_cntr(self, pt1, pt2):
        line_cntr = tuple(map(lambda x: x, ((pt1[0] + pt2[0]) / 2, (pt1[1] + pt2[1]) / 2)))
        return line_cntr

    def get_marker_cntr(self, sqr):
        front_left_corner = sqr[0]
        behind_right_corner = sqr[2]
        cntr = self.get_line_cntr(front_left_corner, behind_right_corner)
        return cntr

    def set_robots(self, robots_list):
        robots_dict = dict()
        for robot in robots_list:
            if not robot.path_created:
                robots_dict[robot.id] = robot
        self.robots = robots_dict

    def set_obstacles(self, obstacles_list):
        obstacles_dict = dict()
        for obstacle in obstacles_list:
            obstacles_dict[obstacle.id] = obstacle
        self.obstacles = obstacles_dict

    def set_targets(self, goals_list):
        targets_dict = dict()
        for goal in goals_list:
            targets_dict[goal.id] = goal
        self.targets = targets_dict

    def set_targets_corners(self, corners):
        self.targets_corners = []
        self.targets_corners = corners

    def plan(self, start_pos, goal_pos, planner_range, obstacles):
        """
        Function that returns path for one robot
        """
        space = self.space_creation()
        space_info = self.space_info_creation(space, obstacles)
        pdef = self.problem_definition(space, space_info, start_pos, goal_pos)
        optimizing_planner = self.allocate_planner(space_info, pdef, planner_range)
        solved = optimizing_planner.solve(const.RUN_TIME)
        if solved:
            path = self.path_optimization(pdef.getSolutionPath(), space_info)
            return path
        else:
            print("No solution found")
            return None

    def allocate_planner(self, space_info, pdef, planner_range):
        """
        Planner setting function that includes choice of planner type,
        determination of maximum distance between 2 points of path and setting problem definition
        """
        optimizing_planner = self.choose_planner(space_info, const.PLANNER_TYPE)
        if planner_range != None:
            optimizing_planner.setRange(planner_range)
        optimizing_planner.setProblemDefinition(pdef)
        optimizing_planner.setup()
        return optimizing_planner

    def choose_planner(self, si, plannerType):
        if plannerType.lower() == "bfmtstar":
            return og.BFMT(si)
        elif plannerType.lower() == "bitstar":
            return og.BITstar(si)
        elif plannerType.lower() == "fmtstar":
            return og.FMT(si)
        elif plannerType.lower() == "informedrrtstar":
            return og.InformedRRTstar(si)
        elif plannerType.lower() == "prmstar":
            return og.PRMstar(si)
        elif plannerType.lower() == "rrtconnect":
            return og.RRTConnect(si)
        elif plannerType.lower() == "rrtsharp":
            return og.RRTsharp(si)
        elif plannerType.lower() == "rrtstar":
            return og.RRTstar(si)
        elif plannerType.lower() == "sorrtstar":
            return og.SORRTstar(si)
        else:
            OMPL_ERROR("Planner-type is not implemented in allocation function.")

    def path_optimization(self, path, space_info):
        """
        Function of optimizing the path and splitting it into equal segments
        """
        ps = og.PathSimplifier(space_info)
        shorteningPath = ps.shortcutPath(path)
        reduceVertices = ps.reduceVertices(path)
        path.interpolate(int(path.length() * 1.1))
        return path

    def space_creation(self):
        """
        Function of creating a configuration space
        """
        space = ob.SE2StateSpace()
        bounds = ob.RealVectorBounds(2)
        bounds.setLow(const.LOW_BOUNDS)
        bounds.setHigh(const.HIGH_BOUNDS)
        space.setBounds(bounds)
        return space

    def space_info_creation(self, space, obstacle_list):
        """
        Function of creating space information that includes valid and invalid states
        """
        space_info = ob.SpaceInformation(space)
        isValidFn = ob.StateValidityCheckerFn(partial(self.isStateValid, obstacle_list))
        space_info.setStateValidityChecker(isValidFn)
        # space_info.setStateValidityChecker(ob.StateValidityCheckerFn(self.isStateValid))
        space_info.setup()
        return space_info

    def isStateValid(self, obstacle_list, state):
        """
        Function that checks the validity of a state of the configuration space
        """
        return self.beyond_obstacles(state.getX(), state.getY(), obstacle_list)

    def beyond_obstacles(self, x_coord, y_coord, obstacle_list):
        """
        Function that check whether a point is inside an obstacle
        """
        for obstacle in obstacle_list:
            if obstacle.contains_point((x_coord, y_coord)):
                return False
        return True

    def problem_definition(self, space, space_info, start_pos, goal_pos):
        """
        Function which define path problem that includes start and goal states
        between which the path must be built
        """
        start = ob.State(space)
        goal = ob.State(space)

        pdef = ob.ProblemDefinition(space_info)
        if not isinstance(start_pos, tuple):
            start[0] = start_pos.x
            start[1] = start_pos.y
            goal[0] = goal_pos.x
            goal[1] = goal_pos.y
        else:
            start[0] = start_pos[0]
            start[1] = start_pos[1]
            goal[0] = goal_pos[0]
            goal[1] = goal_pos[1]
        pdef.setStartAndGoalStates(start, goal)
        return pdef

    def path_to_point_list(self, path):
        states = path.getStates()
        path_lst = []
        for state in states:
            x = state.getX()
            y = state.getY()
            cv_point = Point(x, y).remap_to_img_coord_system()
            path_lst.append(cv_point)
        return path_lst

    def set_source(self, source):
        self.source = source
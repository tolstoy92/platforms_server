cmake_minimum_required(VERSION 2.8.3)
project(platforms_server)

catkin_python_setup()

find_package (
  catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  geometry_msgs
)


add_message_files (
  FILES
  Point2d.msg
  MarkerData.msg
  ArucoData.msg
  RobotData.msg
  GoalData.msg
  ObstacleData.msg
  FieldObjects.msg
  Path.msg
  AllPathes.msg
  RobotMovement.msg
  XML_PATH.msg
  Step.msg
  Wheel.msg
  Wheels_pair.msg
  All_Ik_MSGS.msg
  IK_Data.msg
  RobotSide.msg
)


generate_messages (
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package (
  CATKIN_DEPENDS message_runtime
)


include_directories (
  ${catkin_INCLUDE_DIRS}
)

cmake_minimum_required(VERSION 3.14)
project(mrr_pcd_create_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotAnnouncement.msg"
  "action/CapturePcd.action"
  DEPENDENCIES action_msgs
)

ament_package()

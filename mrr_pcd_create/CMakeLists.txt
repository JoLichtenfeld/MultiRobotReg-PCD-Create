cmake_minimum_required(VERSION 3.14)
project(mrr_pcd_create)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(mrr_pcd_create_msgs REQUIRED)
find_package(hector_multi_robot_msgs REQUIRED)

include(FetchContent)
FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG        v1.91.8
)
FetchContent_MakeAvailable(imgui)

add_library(imgui_lib STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/imgui_demo.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui_lib PUBLIC
  ${imgui_SOURCE_DIR}
  ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui_lib PUBLIC glfw OpenGL::GL)

# Slave node (runs on robots)
add_executable(slave_node src/slave_node.cpp)
target_link_libraries(slave_node ${PCL_LIBRARIES})
target_include_directories(slave_node PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(slave_node PRIVATE ${PCL_DEFINITIONS})
ament_target_dependencies(slave_node rclcpp rclcpp_action sensor_msgs pcl_conversions mrr_pcd_create_msgs hector_multi_robot_msgs)

# Master UI node
add_executable(ui_node src/ui_node.cpp)
target_link_libraries(ui_node imgui_lib ${PCL_LIBRARIES})
target_include_directories(ui_node PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(ui_node PRIVATE ${PCL_DEFINITIONS})
ament_target_dependencies(ui_node rclcpp rclcpp_action sensor_msgs pcl_conversions mrr_pcd_create_msgs hector_multi_robot_msgs)

install(TARGETS ui_node slave_node DESTINATION lib/${PROJECT_NAME})

ament_package()

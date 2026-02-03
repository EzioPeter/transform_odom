cmake_minimum_required(VERSION 3.0.2)
project(transform_odom)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  tf2_eigen
  quadrotor_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf tf2_eigen quadrotor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# add_executable(transform_odom_node src/transform_odom.cpp)
# target_link_libraries(transform_odom_node
#   ${catkin_LIBRARIES}
# )

add_executable(transform_goal_node src/transform_goal.cpp)
target_link_libraries(transform_goal_node ${catkin_LIBRARIES})

add_executable(imagine_cmd_node src/imagine_cmd.cpp)
target_link_libraries(imagine_cmd_node ${catkin_LIBRARIES})
cmake_minimum_required(VERSION 2.8.3)
project(AuctionAssign)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    message_generation
)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_message_files(
  FILES
  DetectInfo.msg
  OdomInfo.msg
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  std_msgs
  nav_msgs
  message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(auction_node src/auction_node.cpp src/info_manager.cpp src/auction.cpp)
# target_link_libraries(auction_node ${catkin_LIBRARIES})

add_executable(sim_pub src/sim_pub.cpp)
target_link_libraries(sim_pub ${catkin_LIBRARIES})

add_executable(task_assign src/centralize_auction.cpp src/auction.cpp)
target_link_libraries(task_assign ${catkin_LIBRARIES})
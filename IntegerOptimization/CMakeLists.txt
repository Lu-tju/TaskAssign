cmake_minimum_required(VERSION 3.10)
project(IntegerOptimization)

# 设置默认构建类型
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall)

# 查找catkin包并声明依赖组件
find_package(catkin REQUIRED COMPONENTS roscpp message_generation)

# 查找Eigen3库
find_package(Eigen3 REQUIRED)

# 添加 lp_solve 的包含目录
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    include/ipsolve
    ${catkin_LIB_DIRS}
)
# 添加 lp_solve 的库目录
link_directories(
include/ipsolve
  ${catkin_LIB_DIRS}
  )


# 添加消息文件
add_message_files(
  FILES
  # formation.msg
)

# 生成消息和服务，包含所有依赖项
generate_messages(
  DEPENDENCIES
  std_msgs
)

# 声明catkin包
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp message_runtime
    DEPENDS EIGEN3
)

# 包含目录
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    
)

# 添加库
add_library(${PROJECT_NAME} src/firepower_allocation.cpp src/dubins_primitive.cpp)

# 链接库
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    lpsolve55 # 使用 lpsolve55 库
)

 set(linked_libs
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        lpsolve55
    )
# 添加可执行文件
add_executable(fire_assign src/main.cpp)
target_link_libraries(fire_assign
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${linked_libs}
  lpsolve55 # 使用 lpsolve55 库
)


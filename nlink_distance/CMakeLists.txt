cmake_minimum_required(VERSION 2.8.3)
project(nlink_distance)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
add_definitions(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    nlink_parser
    )


    # 查找Eigen库
find_package(Eigen3 REQUIRED)

# 包含Eigen头文件
include_directories(${EIGEN3_INCLUDE_DIRS})
## Generate messages in the 'msg' folder
#add_message_files(
#    FILES
#    )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

add_message_files(
  FILES
  DistanceArray_oneself.msg
  DistanceArray_all.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    )

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES nlink
    CATKIN_DEPENDS message_runtime
    #  DEPENDS system_lib
    )


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
#    include
    ${catkin_INCLUDE_DIRS}
    src
    )

set(NODE_NAME "linktrack_example")
add_executable(${NODE_NAME}
    src/main.cpp
    )
    
target_link_libraries(${NODE_NAME}
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    )
add_dependencies(${NODE_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

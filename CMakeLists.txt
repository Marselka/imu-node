cmake_minimum_required(VERSION 2.8.3)
project(imu) #TODO same name as in package.xml
set (CMAKE_CXX_STANDARD 11)

# ******************************************************************** 
#                 Add catkin additional components here
# ******************************************************************** 
find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    sensor_msgs
    std_msgs
    serial
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(imu src/imu.cpp)

target_link_libraries(imu ${catkin_LIBRARIES})

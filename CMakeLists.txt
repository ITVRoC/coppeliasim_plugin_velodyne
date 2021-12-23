cmake_minimum_required(VERSION 2.8.3)
project(coppeliasim_plugin_velodyne)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
            roscpp
            std_msgs
            sensor_msgs
            cmake_modules
            message_generation
            )

catkin_package(
               CATKIN_DEPENDS roscpp rosconsole std_msgs 
               DEPENDS roscpp rosconsole std_msgs )

include_directories(include ${catkin_INCLUDE_DIRS})

link_directories("/opt/ros/groovy/lib")
link_directories("/opt/ros/hydro/lib")
link_directories("/opt/ros/indigo/lib")
link_directories("/opt/ros/jade/lib")
link_directories("/opt/ros/kinetic/lib")
link_directories("/opt/ros/melodic/lib")
link_directories("/opt/ros/noetic/lib")

add_library(
            v_repExtRosVelodyne
            src/v_repLib.cpp
            src/ros_server_velodyne.cpp
            src/velodyneROSModel.cpp
            src/velodyneROSModelCont.cpp
            src/v_repExtVelodyneROS.cpp
            src/scriptFunctionDataItem.cpp
            src/scriptFunctionData.cpp
            include/v_repLib.h
            include/scriptFunctionData.h
            include/scriptFunctionDataItem.h
            include/coppeliasim_plugin_velodyne/ros_server_velodyne.h
            include/coppeliasim_plugin_velodyne/velodyneROSModel.h
            include/coppeliasim_plugin_velodyne/velodyneROSModelCont.h
            include/coppeliasim_plugin_velodyne/v_repExtVelodyneROS.h)

add_dependencies(v_repExtRosVelodyne ${catkin_EXPORTED_TARGETS})
target_link_libraries(v_repExtRosVelodyne
    pthread
    dl
    #roslib
    rosconsole
    rostime
    rospack
    roscpp_serialization
    roscpp
    boost_system
   ${catkin_LIBRARIES}
)

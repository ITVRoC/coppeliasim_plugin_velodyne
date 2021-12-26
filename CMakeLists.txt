cmake_minimum_required(VERSION 3.0.0)
project(coppeliasim_plugin_velodyne VERSION 1.0.0)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
            roscpp
            std_msgs
            sensor_msgs
            cmake_modules
            message_generation
            )

catkin_package(CATKIN_DEPENDS roscpp rosconsole std_msgs 
               DEPENDS roscpp rosconsole std_msgs )

include_directories(include ${catkin_INCLUDE_DIRS})

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
    boost_system
   ${catkin_LIBRARIES}
)

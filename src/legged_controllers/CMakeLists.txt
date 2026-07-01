cmake_minimum_required(VERSION 3.10)
project(legged_controllers)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        controller_manager_msgs
        geometry_msgs
        ocs2_msgs
        rospy
        roscpp
        std_msgs
        visualization_msgs
        legged_common
        legged_interface
        legged_wbc
        legged_estimation
        controller_interface
        ocs2_legged_robot_ros
        ocs2_self_collision_visualization
        angles
        )

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

set(pinocchio_ABSOLUTE_LIBRARIES)
foreach(PINOCCHIO_LIBRARY ${pinocchio_LIBRARIES})
    find_library(PINOCCHIO_ABSOLUTE_LIBRARY_${PINOCCHIO_LIBRARY}
            NAMES ${PINOCCHIO_LIBRARY}
            PATHS ${pinocchio_LIBRARY_DIRS}
            NO_DEFAULT_PATH
            )
    if(PINOCCHIO_ABSOLUTE_LIBRARY_${PINOCCHIO_LIBRARY})
        list(APPEND pinocchio_ABSOLUTE_LIBRARIES ${PINOCCHIO_ABSOLUTE_LIBRARY_${PINOCCHIO_LIBRARY}})
    else()
        list(APPEND pinocchio_ABSOLUTE_LIBRARIES ${PINOCCHIO_LIBRARY})
    endif()
endforeach()
set(pinocchio_LIBRARIES ${pinocchio_ABSOLUTE_LIBRARIES})

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        controller_manager_msgs
        geometry_msgs
        ocs2_msgs
        rospy
        roscpp
        std_msgs
        visualization_msgs
        legged_common
        legged_interface
        legged_wbc
        legged_estimation
        controller_interface
        ocs2_legged_robot_ros
        ocs2_self_collision_visualization
        angles
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
)

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
        src/LeggedController.cpp
        src/AmpDataLogger.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


# Target trajectories publish node
add_executable(legged_target_trajectories_publisher
        src/TargetTrajectoriesPublisher.cpp
        )
add_dependencies(legged_target_trajectories_publisher
        ${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(legged_target_trajectories_publisher
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
target_compile_options(legged_target_trajectories_publisher PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME} legged_target_trajectories_publisher
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Install plugins
install(FILES legged_controllers_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

catkin_install_python(PROGRAMS
        scripts/keyboard_control.py
        scripts/auto_amp_data_collector.py
        scripts/foot_trajectory_plotter.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

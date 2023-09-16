cmake_minimum_required(VERSION 3.0.2)
project(ocs2_pros_controllers)
SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    controller_interface
    ocs2_legged_robot_ros
    ocs2_self_collision_visualization
    angles
    ocs2_pros_interface
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        controller_interface
        ocs2_legged_robot_ros
        ocs2_self_collision_visualization
        angles
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
)

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)
##########################################
###########################################
add_executable(pros_mpc_node
  src/ProsMpcNode.cpp
)
add_dependencies(pros_mpc_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(pros_mpc_node
  ${catkin_LIBRARIES}
)
target_compile_options(pros_mpc_node PUBLIC ${FLAGS})
#############################################
add_executable(pros_dummy_mrt_node
  src/ProsDummyMRT.cpp
  src/ProsDummyVisualization.cpp
)
add_dependencies(pros_dummy_mrt_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(pros_dummy_mrt_node
  ${catkin_LIBRARIES}
)
target_compile_options(pros_dummy_mrt_node PUBLIC ${FLAGS})
#############################################
add_executable(pros_target
  src/ProsTarget.cpp
)
add_dependencies(pros_target
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(pros_target
  ${catkin_LIBRARIES}
)
target_compile_options(pros_target PUBLIC ${FLAGS})
#############################################
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


# Mark executables and/or libraries for installation
install(TARGETS 
        pros_mpc_node
        # pros_dummy_mrt_node 
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
cmake_minimum_required(VERSION 2.8.3)
project(yocs_diff_drive_pose_controller)

find_package(catkin REQUIRED COMPONENTS ecl_threads
                                        geometry_msgs
                                        nodelet
                                        pluginlib
                                        roscpp
                                        sensor_msgs
                                        std_msgs
                                        tf
                                        yocs_controllers)

catkin_package(INCLUDE_DIRS include
               LIBRARIES yocs_diff_drive_pose_controller_nodelet
               CATKIN_DEPENDS ecl_threads
                              geometry_msgs
                              nodelet
                              pluginlib
                              roscpp
                              sensor_msgs
                              std_msgs
                              tf
                              yocs_controllers)

include_directories(include
                    ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
    src/nodelet.cpp
    src/diff_drive_pose_controller_ros.cpp)
    
add_dependencies(${PROJECT_NAME}
    geometry_msgs_gencpp
    sensor_msgs_gencpp
    std_msgs_gencpp)
    
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY plugins DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


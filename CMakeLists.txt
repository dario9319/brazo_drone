cmake_minimum_required(VERSION 3.0.2)
project(brazo_drone)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs

  message_generation
  dynamixel_sdk
)


################################################################################
# Declare ROS messages, services and actions
################################################################################
## Generate messages in the 'msg' folder
add_message_files(
  FILES
    SetPosition.msg
    SyncSetPosition.msg
    BulkSetItem.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
    GetPosition.srv
    SyncGetPosition.srv
    BulkGetItem.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
)


find_package(OpenCV)
find_package(Qt5Widgets)

include_directories(${OpenCV_INCLUDE_DIRS})



catkin_package(

  CATKIN_DEPENDS 
    cv_bridge 
    image_transport 
    roscpp 
    rospy 
    sensor_msgs 
    std_msgs
    dynamixel_sdk

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}

)

add_executable(read_write_node src/read_write_node.cpp)
add_dependencies(read_write_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(read_write_node ${catkin_LIBRARIES})

add_executable(sync_read_write_node src/sync_read_write_node.cpp)
add_dependencies(sync_read_write_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sync_read_write_node ${catkin_LIBRARIES})

add_executable(bulk_read_write_node src/bulk_read_write_node.cpp)
add_dependencies(bulk_read_write_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bulk_read_write_node ${catkin_LIBRARIES})

add_executable(indirect_address_node src/indirect_address_node.cpp)
add_dependencies(indirect_address_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(indirect_address_node ${catkin_LIBRARIES})


catkin_install_python(PROGRAMS
#   scripts/my_python_script
  src/app.py
  src/appFinal.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


 install(TARGETS read_write_node sync_read_write_node bulk_read_write_node indirect_address_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)











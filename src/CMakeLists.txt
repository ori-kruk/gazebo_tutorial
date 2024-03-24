find_package(gazebo REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)
link_directories(${GAZEBO_LIBRARY_DIRS})
# list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(GimbalSmall2dPlugin SHARED GimbalSmall2dPlugin.cc)
target_link_libraries(GimbalSmall2dPlugin ${GAZEBO_LIBRARIES})

add_library(ArduCopterPlugin SHARED ArduCopterPlugin.cc)
target_link_libraries(ArduCopterPlugin ${GAZEBO_LIBRARIES})

add_library(ArduPilotPlugin SHARED ArduPilotPlugin.cc)
target_link_libraries(ArduPilotPlugin ${GAZEBO_LIBRARIES})

add_library(NodeDemoPlugin SHARED NodeDemoPlugin.cc)
target_link_libraries(NodeDemoPlugin ${GAZEBO_LIBRARIES})

add_library(JointControlDemoPlugin SHARED JointControlDemoPlugin.cc)
target_link_libraries(JointControlDemoPlugin ${GAZEBO_LIBRARIES})

add_library(RosNodeDemoPlugin
    SHARED 
    RosNodeDemoPlugin.cc
)
target_include_directories(RosNodeDemoPlugin
  PRIVATE
  ${std_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
)
target_link_libraries(RosNodeDemoPlugin 
  ${rclcpp_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${gazebo_ros_LIBRARIES}
  )


install(TARGETS
GimbalSmall2dPlugin
ArduCopterPlugin
ArduPilotPlugin
NodeDemoPlugin
JointControlDemoPlugin
RosNodeDemoPlugin
  DESTINATION ${PROJECT_SOURCE_DIR}/bin
)
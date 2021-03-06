cmake_minimum_required(VERSION 2.8.3)
project(ras_group8_brain)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  std_msgs
  message_generation 
  ras_group8_arm_controller
  visualization_msgs
  tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES
  Vision.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)


catkin_package(
  INCLUDE_DIRS
    include

  CATKIN_DEPENDS
    roscpp
    nav_msgs
    geometry_msgs
    std_msgs
    ras_group8_arm_controller
    visualization_msgs
    message_runtime
    tf

   #DEPENDS
)


add_executable(${PROJECT_NAME}
  src/brain_node.cpp
  src/Brain.cpp
  src/Arm.cpp
  src/FindPathStates.cpp
  src/InitState.cpp
  src/ObstacleState.cpp
  src/PathExectuionState.cpp
  src/Speaker.cpp
  src/HomeState.cpp
  src/Vizualisation.cpp
  src/textfile.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_tests
    test/test_brain.cpp
    src/Brain.cpp
  )
  
  target_link_libraries(${PROJECT_NAME}_tests
    ${catkin_LIBRARIES}
  )
endif()

# Make main executable
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

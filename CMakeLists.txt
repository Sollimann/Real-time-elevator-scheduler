cmake_minimum_required(VERSION 2.8.3)
project(simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

find_package(Armadillo REQUIRED)

add_message_files(
  FILES
  calls.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(CATKIN_DEPENDS rospy roscpp std_msgs message_runtime)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)



## Declare C++ executables:
add_executable(elevator_call_generator src/elevator_call_generator.cpp)
target_link_libraries(elevator_call_generator ${catkin_LIBRARIES})
add_dependencies(elevator_call_generator simulation_generate_message_cpp)

add_executable(master_elevator_nc src/master_elevator_nc.cpp)
target_link_libraries(master_elevator_nc ${catkin_LIBRARIES})
add_dependencies(master_elevator_nc simulation_generate_message_cpp)

add_executable(master_elevator_fso src/master_elevator_fso.cpp)
target_link_libraries(master_elevator_fso ${catkin_LIBRARIES})
add_dependencies(master_elevator_fso simulation_generate_message_cpp)


install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

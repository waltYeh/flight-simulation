cmake_minimum_required(VERSION 2.8.3)
project(r2sim)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  genmsg
)

add_message_files(
  FILES
  states.msg
  commands.msg
  output.msg
  control_sp.msg
  sending.msg
  commander_plot.msg
)

generate_messages(
   DEPENDENCIES
  std_msgs
#  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(simcommons SHARED src/commons.cpp)
add_library(simnurbs SHARED src/nurbs.cpp)
add_library(simsmooth SHARED src/tangential_smooth.cpp)

add_executable(simlinker src/linker.cpp)
target_link_libraries(simlinker
  simcommons 
 ${catkin_LIBRARIES} 
  )
add_dependencies(simlinker r2d2_cpp)

add_executable(simlistener src/listener.cpp)
target_link_libraries(simlistener 
  simcommons
  ${catkin_LIBRARIES})
add_dependencies(simlistener r2d2_cpp)

add_executable(simcommander src/commander.cpp)
target_link_libraries(simcommander 
  simcommons
  simnurbs
  simsmooth
  ${catkin_LIBRARIES})
add_dependencies(simcommander r2d2_cpp)

add_executable(simcontroller src/controller.cpp)
target_link_libraries(simcontroller 
  simcommons
  ${catkin_LIBRARIES})
add_dependencies(simcontroller r2d2_cpp)

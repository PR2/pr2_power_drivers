# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 3.0.2)
project(pr2_power_board)

find_package(catkin COMPONENTS roscpp diagnostic_updater diagnostic_msgs pr2_msgs message_generation)
find_package(Boost REQUIRED COMPONENTS thread program_options)
find_library(LOG4CXX_LIBRARY log4cxx)

add_service_files(
  FILES
  PowerBoardCommand.srv
  PowerBoardCommand2.srv
)

catkin_python_setup()

## Generate added messages and services with any dependencies listed here
generate_messages()

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(power_node src/power_node/power_node.cpp)
add_dependencies(power_node
   ${catkin_EXPORTED_TARGETS}
   ${pr2_power_board_EXPORTED_TARGETS})

add_executable(power_node2 src/power_node/power_node2.cpp)
add_dependencies(power_node2 
   ${catkin_EXPORTED_TARGETS}
   ${pr2_power_board_EXPORTED_TARGETS})

target_link_libraries(power_node 
   ${LOG4CXX_LIBRARY}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES})
target_link_libraries(power_node2 
   ${LOG4CXX_LIBRARY}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES})

catkin_package(CATKIN_DEPENDS message_runtime roscpp diagnostic_updater diagnostic_msgs pr2_msgs)

install(TARGETS power_node power_node2
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS 
   scripts/fake_powerboard.py
   scripts/send_command
   scripts/test_power.py
   scripts/view_power
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY ui
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES
   gui_power.launch
   gui_power2.launch
   power.xml
   pr_wiring.yaml
   test_wires.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

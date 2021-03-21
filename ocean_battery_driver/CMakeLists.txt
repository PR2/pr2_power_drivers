cmake_minimum_required(VERSION 3.0.2)
project(ocean_battery_driver)

find_package(catkin REQUIRED COMPONENTS roscpp pr2_msgs diagnostic_updater diagnostic_msgs)

catkin_package(
	DEPENDS roscpp pr2_msgs diagnostic_updater diagnostic_msgs
)

add_subdirectory(src)

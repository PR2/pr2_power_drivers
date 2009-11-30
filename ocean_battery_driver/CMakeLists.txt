cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${ocean_battery_driver_SOURCE_DIR}/bin)
add_subdirectory(src)
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${ocean_battery_driver_SOURCE_DIR}/bin/ocean")

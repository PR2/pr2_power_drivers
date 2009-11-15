cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

rosbuild_gensrv()

set(EXECUTABLE_OUTPUT_PATH ${pr2_power_board_SOURCE_DIR}/bin)
add_subdirectory(src)
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${pr2_power_board_SOURCE_DIR}/bin/power;${pr2_power_board_SOURCE_DIR}/power;${pr2_power_board_SOURCE_DIR}/power_node")

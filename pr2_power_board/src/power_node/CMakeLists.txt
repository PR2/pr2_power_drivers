rosbuild_add_executable(power_node power_node.cpp)

rosbuild_add_boost_directories()
rosbuild_link_boost(power_node thread program_options)


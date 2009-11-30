rosbuild_add_executable(ocean_server server.cpp ocean.cpp)

rosbuild_add_boost_directories()
rosbuild_link_boost(ocean_server thread program_options)


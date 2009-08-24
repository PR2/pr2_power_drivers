rospack_add_executable(ocean_server server.cpp ocean.cpp)
rospack_add_executable(ocean_test ocean.cpp ocean_test.cpp)

rospack_add_boost_directories()
rospack_link_boost(ocean_server thread program_options)


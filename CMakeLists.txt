cmake_minimum_required(VERSION 3.5)
project(r2r_minimal_node)
include(r2r_cargo.cmake)

# put ros package dependencies here.
r2r_cargo(std_msgs  # just to test that it works
          rcl       # we need the c ros2 api
         )

# install your binaries
install(PROGRAMS
  ${CMAKE_SOURCE_DIR}/target/release/${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# we need this for ros
find_package(ament_cmake REQUIRED)
ament_package()

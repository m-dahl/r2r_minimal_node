cmake_minimum_required(VERSION 3.5)
project(r2r_minimal_node)

find_package(ament_cmake REQUIRED)

include(r2r_cargo.cmake)

# put ros package dependencies here.
r2r_cargo(std_msgs               # just to test that it works
          rcl                    # we need the c ros2 api
          rcl_action             # as of r2r 0.1.0, we also need the action api
          r2r_minimal_node_msgs  # our custom message types
         )

# install binaries
install(PROGRAMS
  ${CMAKE_SOURCE_DIR}/target/release/${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# we need this for ros/colcon
ament_package()

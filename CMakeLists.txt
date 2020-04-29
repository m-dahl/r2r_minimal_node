cmake_minimum_required(VERSION 3.5)
project(r2r_minimal_node)

# create our own AMENT_PREFIX_PATH just for this package. this allows
# us to build our stuff without colcon by pulling in all ros deps
function(ros_deps)
  foreach(f ${ARGN})
    find_package(${f} REQUIRED)
    list(APPEND AMENT_HACK_PATH "${${f}_DIR}/../../../")
  endforeach()
  string (REPLACE ";" ":" AMENT_HACK_PATH_STR "${AMENT_HACK_PATH}")
  set(ENV{AMENT_HACK_PATH} ${AMENT_HACK_PATH_STR})
endfunction()

# put ros package dependencies here.
ros_deps(geometry_msgs
         sp_operator_msgs
         rclcpp
        )

# this works fine except that it will be called both on make all and again on make install...
add_custom_target(dummy_target ALL
    COMMAND "env" "AMENT_PREFIX_PATH=$ENV{AMENT_HACK_PATH}" "cargo" "build" "--release"
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )

install(PROGRAMS
  ${CMAKE_SOURCE_DIR}/target/release/${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# we need this
find_package(ament_cmake REQUIRED)
ament_package()

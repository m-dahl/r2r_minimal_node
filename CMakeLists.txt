cmake_minimum_required(VERSION 3.5)
project(r2r_minimal_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# custom_output will always be rebuilt because it depends on always_rebuild
add_custom_command(
    OUTPUT custom_output
    COMMAND "cargo" "build" "--release"
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    DEPENDS
        always_rebuild
    )

# no-op
add_custom_command(
    OUTPUT always_rebuild
    COMMAND cmake -E echo
    )

add_custom_target(dummy_target ALL DEPENDS custom_output)

install(PROGRAMS
  ${CMAKE_SOURCE_DIR}/target/release/${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

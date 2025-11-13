cmake_minimum_required(VERSION 2.8.3)
project(eagleye_navigation)

find_package(PkgConfig REQUIRED)

find_package(Eigen3 QUIET)
set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

#find_package(eagleye_coordinate CONFIG REQUIRED)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(eagleye_navigation
  src/velocity_scale_factor.cpp
  src/distance.cpp
  src/yaw_rate_offset_stop.cpp
  src/yaw_rate_offset.cpp
  src/heading.cpp
  src/position.cpp
  src/slip_angle.cpp
  # src/slip_coefficient.cpp
  # src/smoothing.cpp
  src/trajectory.cpp
  src/heading_interpolate.cpp
  src/position_interpolate.cpp
  src/height.cpp
  src/angular_velocity_offset_stop.cpp
  src/rolling.cpp
  # src/velocity_estimator.cpp
)

target_include_directories(
    eagleye_navigation
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(eagleye_navigation
  eagleye_coordinate
)
add_dependencies(eagleye_navigation eagleye_coordinate)

install(DIRECTORY include/navigation/
  DESTINATION include/navigation/
  FILES_MATCHING PATTERN "*.hpp"
)

install(TARGETS eagleye_navigation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

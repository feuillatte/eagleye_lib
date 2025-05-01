cmake_minimum_required(VERSION 2.8.3)
project(eagleye_coordinate)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_LIBRARIES
  NAMES Geographic
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${GeographicLib_INCLUDE_DIRS}
)
link_directories(
  /usr/local/lib
)

add_library(eagleye_coordinate
  src/ecef2llh.cpp
  src/enu2llh.cpp
  src/enu2xyz_vel.cpp
  src/ll2xy.cpp
  src/llh2xyz.cpp
  src/xyz2enu.cpp
  src/xyz2enu_vel.cpp
  src/ll2xy_mgrs.cpp
  src/convertheight.cpp
)

target_include_directories(eagleye_coordinate
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(eagleye_coordinate
  Geographic
)

install(DIRECTORY include/coordinate/
  DESTINATION include/coordinate
  FILES_MATCHING PATTERN "*.hpp"
)

install(
  DIRECTORY
    data
  DESTINATION share
)

install(TARGETS eagleye_coordinate
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

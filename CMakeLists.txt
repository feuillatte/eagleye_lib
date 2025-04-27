cmake_minimum_required(VERSION 3.16)
project(eagleye_localization)

find_package(Eigen3 QUIET)
set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_LIBRARIES
  NAMES Geographic
)

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}/lib/include
  ${CMAKE_CURRENT_LIST_DIR}/eagleye_core/coordinate/include
  ${CMAKE_CURRENT_LIST_DIR}/eagleye_core/navigation/include
  ${EIGEN3_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  ${eagleye_navigation_INCLUDE_DIRS}
)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/eagleye_core/coordinate)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/eagleye_core/navigation)

add_library(eagleye
    lib/src/localization.cpp
)

target_link_libraries(eagleye
    eagleye_coordinates
    eagleye_navigation
    Geographic
)

install(
    DIRECTORY lib/include/
    DESTINATION include/
    FILES_MATCHING PATTERN "*.hpp"
)

install(
    TARGETS eagleye
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib
)

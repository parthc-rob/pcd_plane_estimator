cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_plane_estimator)

find_package(PCL 1.2 REQUIRED)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost)

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -std=c++17")
set(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_plane_calculation src/pcl_plane_calculation.cpp)
target_link_libraries (pcl_plane_calculation ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable (plane_extractor src/plane_extractor.cpp)
target_link_libraries (plane_extractor ${PCL_LIBRARIES} ${Boost_LIBRARIES})

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(tests src/tests.cpp)
target_link_libraries(tests ${GTEST_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} pthread)

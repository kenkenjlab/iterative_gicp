#### CMake Settings ####

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
cmake_policy(SET CMP0015 OLD)	#// Do not add relative path if the designated path is not exist
add_definitions(-DUNICODE -D_UNICODE)	#// Set Unicode, not MultiBytes


#### Solution Settings ####

project(iterative_gicp)
if(MSVC AND NOT PLATFORM EQUAL 64)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /LARGEADDRESSAWARE")
  message(STATUS "MSVC: Enabled large address awareness")
endif()


#### Includes and Libraries ####

find_package(PCL 1.7 REQUIRED)
find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  message(STATUS "MSVC: Enabled OpenMP")
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



#### Required files ####

set(ITERATIVE_GICP iterative_gicp.hpp)
set(GENERIC_POINT_CLOUD_IO generic_point_cloud_io.h generic_point_cloud_io.cpp)

#### Projects Settings ####

add_executable(iterative_gicp_for_two_files main_two_files.cpp ${ITERATIVE_GICP} ${GENERIC_POINT_CLOUD_IO})
target_link_libraries(iterative_gicp_for_two_files ${PCL_LIBRARIES})

add_executable(iterative_gicp_for_folder main_folder.cpp ${ITERATIVE_GICP} ${GENERIC_POINT_CLOUD_IO})
target_link_libraries(iterative_gicp_for_folder ${PCL_LIBRARIES})
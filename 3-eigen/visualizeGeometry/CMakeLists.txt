cmake_minimum_required( VERSION 3.0 )
project( visualizeGeometry )

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package( Pangolin )
find_package(Eigen3 3.3 REQUIRED MO_MODULE)
include_directories( ${Pangolin_INCLUDE_DIRS} )

add_executable( visualizeGeometry visualizeGeometry.cpp )
target_link_libraries( visualizeGeometry ${Pangolin_LIBRARIES} )

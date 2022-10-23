cmake_minimum_required( VERSION 3.0 )
project( geometry )

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(eigenGeometry eigenGeometry.cpp)
target_link_libraries(eigenGeometry Eigen3::Eigen)


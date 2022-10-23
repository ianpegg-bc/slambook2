cmake_minimum_required(VERSION 3.0)
project(useSophus)

find_package(Sophus REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(fmt REQUIRED)

add_executable(useSophus useSophus.cpp)
target_link_libraries(useSophus Sophus::Sophus)
target_link_libraries(useSophus Eigen3::Eigen)

# add_subdirectory(example)

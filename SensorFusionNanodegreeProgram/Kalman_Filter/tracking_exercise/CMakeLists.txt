cmake_minimum_required(VERSION 3.18)


set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(kalman_filter)
find_package(Eigen3 REQUIRED)

add_executable (kf src/main.cpp src/kalman_filter.cpp src/tracking.cpp)
target_link_libraries(kf Eigen3::Eigen)



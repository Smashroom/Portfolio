cmake_minimum_required(VERSION 3.18)


set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(kalman_filter)
find_package(Eigen3 REQUIRED)

add_executable (ukf src/main.cpp src/ukf.cpp)
target_link_libraries(ukf Eigen3::Eigen)



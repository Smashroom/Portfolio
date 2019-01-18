cmake_minimum_required(VERSION 2.8.3)
project(test_driver)


add_compile_options(-std=c++11)


find_package(catkin REQUIRED 
roscpp
std_msgs
genmsg 
message_generation
)
find_package(Boost REQUIRED COMPONENTS system )

add_message_files(FILES Combined.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)



add_executable(velocity_driver velocity_driver.cpp)



add_dependencies(velocity_driver test_driver_generate_messages_cpp)

 

 target_link_libraries(velocity_driver
   ${catkin_LIBRARIES} 
   ${boost_LIBRARIES}
 )

cmake_minimum_required(VERSION 3.5)
project(jtbot_foc_driver)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include
	            ${Boost_INCLUDE_DIRS})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(jt_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread program_options)

add_executable(jtbot_foc_driver src/jtbot_foc_driver.cpp)
target_link_libraries(jtbot_foc_driver
	              ${Boost_LIBRARIES})
ament_target_dependencies(jtbot_foc_driver 
			         rclcpp 
              geometry_msgs
              nav_msgs
              tf2
              tf2_ros
			        std_msgs
              serial
             
              )



install(TARGETS
  jtbot_foc_driver
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})


ament_package()

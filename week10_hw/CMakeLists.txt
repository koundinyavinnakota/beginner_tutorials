cmake_minimum_required(VERSION 3.8)
project(week10_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
         # CHANGE
find_package(rosidl_default_generators REQUIRED)

set(msg_files 
 "msg/Details.msg")

set(srv_files
  "srv/Details.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)
ament_export_dependencies(rosidl_default_runtime)



add_executable(publish_details
  src/publish_details.cpp
)
ament_target_dependencies(publish_details
  "rclcpp"
)
install(TARGETS publish_details
 DESTINATION lib/${PROJECT_NAME})

rosidl_get_typesupport_target(cpp_typesupport_target
${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(publish_details "${cpp_typesupport_target}")



ament_package()

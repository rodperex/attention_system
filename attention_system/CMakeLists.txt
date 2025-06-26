cmake_minimum_required(VERSION 3.8)
project(attention_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
# find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(attention_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)

find_package(rclcpp REQUIRED)


set(dependencies
  ament_index_cpp
  # geometry_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
  attention_msgs
  std_srvs  
  rclcpp
  rclcpp_cascade_lifecycle
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/attention_system/BaseController.cpp
  src/attention_system/Tester.cpp
  src/attention_system/PID.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(attention_system_base src/base_main.cpp)
ament_target_dependencies(attention_system_base ${dependencies})
target_link_libraries(attention_system_base ${PROJECT_NAME})

add_executable(test_attention src/test_attention.cpp)
ament_target_dependencies(test_attention ${dependencies})
target_link_libraries(test_attention ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  attention_system_base
  test_attention
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()

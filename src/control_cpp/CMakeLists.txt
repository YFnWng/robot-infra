cmake_minimum_required(VERSION 3.8)
project(control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_interface REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(shadow_contract src/shadow_contract.cpp)
target_compile_features(shadow_contract PUBLIC cxx_std_17)
target_include_directories(shadow_contract PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(control_shadow src/control_shadow_node.cpp)
target_compile_features(control_shadow PUBLIC cxx_std_17)
target_link_libraries(control_shadow shadow_contract)
ament_target_dependencies(control_shadow
  control_interface diagnostic_msgs geometry_msgs rclcpp sensor_msgs)

install(TARGETS shadow_contract control_shadow
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_shadow_contract test/test_shadow_contract.cpp)
  target_link_libraries(test_shadow_contract shadow_contract)
  target_compile_definitions(test_shadow_contract PRIVATE
    SHADOW_FIXTURE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../catheter_control/test/fixtures/shadow_decision_cases.csv")
endif()

ament_export_include_directories(include)
ament_export_libraries(shadow_contract)
ament_package()

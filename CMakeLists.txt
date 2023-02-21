cmake_minimum_required(VERSION 3.16)
project(topic_based_ros2_control CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Werror -Wall -Wextra -Wpedantic -Wshadow
    -Wsign-conversion -Winit-self -Wredundant-decls)
endif()

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  hardware_interface
  sensor_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

###########
## Build ##
###########

# Declare a C++ library
add_library(
  ${PROJECT_NAME}
  src/topic_based_system.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(${PROJECT_NAME}
  PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  # ROS2 linters, but disable copyright test. PickNik's copyright's may not conform
  # to this test
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # GTests
  ament_add_gtest(topic_based_system_test test/topic_based_system_test.cpp)
  target_link_libraries(topic_based_system_test
    ${PROJECT_NAME})
  ament_target_dependencies(topic_based_system_test ${THIS_PACKAGE_INCLUDE_DEPENDS})
endif()

pluginlib_export_plugin_description_file(hardware_interface topic_based_ros2_control_plugin_description.xml)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

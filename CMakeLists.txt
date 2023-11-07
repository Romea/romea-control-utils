cmake_minimum_required(VERSION 3.8)
project(romea_following_utils)

if(NOT CMAKE_CXX_STANDARD)
 set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(romea_core_common REQUIRED)
find_package(romea_core_control REQUIRED)
find_package(romea_common_utils REQUIRED)

# create a header only library -> INTERACE
add_library(${PROJECT_NAME} INTERFACE
)

ament_target_dependencies(${PROJECT_NAME} INTERFACE
  rclcpp
  romea_core_common
  romea_core_control
  romea_common_utils  
)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# install(TARGETS ${PROJECT_NAME} 
#   LIBRARY DESTINATION lib
# )

install(
  DIRECTORY include/
  DESTINATION include
)


ament_export_include_directories(include)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_package()

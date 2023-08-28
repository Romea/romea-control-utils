find_package(ament_cmake_gtest REQUIRED)

get_filename_component(TEST_DIR_SRC ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
configure_file(test_helper.h.in test_helper.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

ament_add_gtest(test_command_factory test_command_factory.cpp)
target_link_libraries(test_command_factory ${PROJECT_NAME})

ament_add_gtest(test_inertia_factory test_inertia_factory.cpp)
target_link_libraries(test_inertia_factory ${PROJECT_NAME})

ament_add_gtest(test_observer_factory test_observer_factory.cpp)
target_link_libraries(test_observer_factory ${PROJECT_NAME})

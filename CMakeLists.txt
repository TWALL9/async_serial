cmake_minimum_required(VERSION 3.5)
project(async_serial)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_library(
  ${PROJECT_NAME}_kiss
  SHARED
  src/kiss_serial/kiss_decode.cpp
  src/kiss_serial/kiss_encode.cpp
)

target_include_directories(
  ${PROJECT_NAME}_kiss
  PRIVATE
  include
)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/serial_port.cpp
)

# include_directories(include ${Boost_INCLUDE_DIRS})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS}
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Boost)
ament_target_dependencies(${PROJECT_NAME} rclcpp Boost)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

add_executable(demo_node src/demo_node.cpp)
ament_target_dependencies(demo_node rclcpp std_msgs Boost)
target_link_libraries(demo_node ${PROJECT_NAME})


install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_kiss
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS demo_node DESTINATION bin/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(
  ${PROJECT_NAME}_kiss
)

ament_package()

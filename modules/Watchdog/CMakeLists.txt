set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_VERBOSE_MAKEFILE ON)

set(HEADER_FILES
  include/Watchdog.hpp
  include/WatchdogNode.hpp)
set(SOURCE_FILES
  src/Watchdog.cpp
  src/WatchdogNode.cpp)

add_library(${PROJECT_NAME}
  ${HEADER_FILES}
  ${SOURCE_FILES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME} PUBLIC
  ${ament_cmake_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${ament_cmake_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
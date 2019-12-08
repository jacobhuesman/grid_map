cmake_minimum_required(VERSION 3.5)
project(grid_map_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(${PROJECT_NAME}_dependencies
  rclcpp
  grid_map_core
  grid_map_msgs
  grid_map_cv
  sensor_msgs
  nav_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  rosbag2
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  visualization_msgs
)

find_package(ament_cmake REQUIRED)
foreach(dep ${${PROJECT_NAME}_dependencies})
  find_package(${dep} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/GridMapRosConverter.cpp
  src/GridMapMsgHelpers.cpp
  src/PolygonRosConverter.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_dependencies})
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
)

#############
## Install ##
#############
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${${PROJECT_NAME}_dependencies})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Mark cpp header files for installation
install(
  DIRECTORY include/
  DESTINATION include
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  ## Add gtest based cpp test target and link libraries
  #ament_add_gtest(${PROJECT_NAME}-test
  #  test/test_grid_map_ros.cpp
  #  test/GridMapRosTest.cpp
  #)
  #target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

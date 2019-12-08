cmake_minimum_required(VERSION 3.5)
project(grid_map_cv)

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

find_package(ament_cmake REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(filters REQUIRED) TODO

find_package(OpenCV REQUIRED
  COMPONENTS
  opencv_photo
)

###########
## Build ##
###########

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/GridMapCvProcessing.cpp
  #src/InpaintFilter.cpp TODO
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME}
  grid_map_core
  cv_bridge
)

#############
## Install ##
#############

# Export dependencies
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
#ament_export_dependencies(some_dependency)
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

# TODO reintegrate filter plugins
# Mark other files for installation.
#install(
#  FILES filter_plugins.xml
#  DESTINATION share
#)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  ## Add gtest based cpp test target and link libraries
  ament_add_gtest(${PROJECT_NAME}-test
    test/test_grid_map_cv.cpp
    test/GridMapCvTest.cpp
    test/GridMapCvProcessingTest.cpp)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

ament_package()
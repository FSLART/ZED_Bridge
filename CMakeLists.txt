cmake_minimum_required(VERSION 3.10)
project(zed_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
# Try to find the ZED SDK, but allow building without it (for CI)
find_package(ZED 3 QUIET)
if(ZED_FOUND)
  add_definitions(-DWITH_ZED)
  message(STATUS "Building with ZED SDK support")
  # find the correct CUDA version only when ZED is present
  find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)
else()
  message(WARNING "ZED SDK not found - building in mock mode (no camera)")
endif()
find_package(OpenCV REQUIRED)
find_package(lart_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# setup the include directories
include_directories(include)
if(ZED_FOUND)
  include_directories(${CUDA_INCLUDE_DIRS})
  include_directories(${ZED_INCLUDE_DIRS})
endif()
include_directories(${OpenCV_INCLUDE_DIRS})

# link the ZED and CUDA libraries
if(ZED_FOUND)
  link_directories(${ZED_LIBRARY_DIR})
  link_directories(${CUDA_LIBRARY_DIRS})
endif()
link_directories(${OpenCV_LIBRARY_DIRS})


add_executable(zed_bridge src/zed_bridge.cpp)
ament_target_dependencies(zed_bridge rclcpp sensor_msgs tf2 tf2_ros geometry_msgs lart_msgs visualization_msgs image_transport)

target_link_libraries(zed_bridge
  ${OpenCV_LIBS}
)
if(ZED_FOUND)
  target_link_libraries(zed_bridge
    ${ZED_LIBRARIES}
    ${CUDA_LIBRARIES}
  )
endif()

install(TARGETS
  zed_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

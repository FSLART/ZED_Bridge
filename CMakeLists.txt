cmake_minimum_required(VERSION 3.8)
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
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
# find the ZED SDK
find_package(ZED 3 REQUIRED)
# find the correct CUDA version
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)
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
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

# link the ZED and CUDA libraries
link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})


# Mudamos para SHARED library para ser um componente
add_library(zed_bridge SHARED src/zed_bridge.cpp)

# Adicionamos rclcpp_components às dependências
ament_target_dependencies(zed_bridge rclcpp rclcpp_components sensor_msgs tf2 tf2_ros geometry_msgs lart_msgs visualization_msgs image_transport)

# Registamos o componente (Isto cria o "plugin")
rclcpp_components_register_nodes(zed_bridge "ZedBridge")

target_link_libraries(zed_bridge
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS zed_bridge
  EXPORT zed_bridgeExport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Adicionamos 'config' à lista
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Instalamos o script Python da ponte
install(PROGRAMS
  scripts/recording_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES zed_bridge_plugins.xml DESTINATION share/${PROJECT_NAME})
ament_package()

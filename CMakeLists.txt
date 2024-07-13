cmake_minimum_required(VERSION 3.8)
project(thermal_image_transport)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(image_republisher src/ImageRepublisher.cpp)

ament_target_dependencies(image_republisher
  rclcpp
  image_transport
  sensor_msgs
  OpenCV
)

install(TARGETS
  image_republisher
  DESTINATION lib/${PROJECT_NAME}
)
# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()

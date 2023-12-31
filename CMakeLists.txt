cmake_minimum_required(VERSION 3.8)
project(stream_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(usb_cam REQUIRED)

find_package(OpenCV REQUIRED)

include_directories(SYSTEM 
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(split_image src/split_image.cpp)
ament_target_dependencies(split_image
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  cv_bridge
  image_transport)
target_link_libraries(split_image ${OpenCV_LIBRARIES})

add_executable(show_images src/show_images.cpp)
ament_target_dependencies(show_images
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  cv_bridge
  image_transport)
target_link_libraries(show_images ${OpenCV_LIBRARIES})

install(TARGETS split_image show_images
  DESTINATION lib/${PROJECT_NAME})

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


install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}/
)

ament_package()

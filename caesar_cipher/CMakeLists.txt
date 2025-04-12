cmake_minimum_required(VERSION 3.8)
project(caesar_cipher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message and service generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CipherMessage.msg"
  "srv/CipherAnswer.srv"
  DEPENDENCIES std_msgs
)

# Build nodes
add_executable(cipher_node src/cipher_node.cpp)
add_executable(decoder_node src/decoder_node.cpp)

ament_target_dependencies(cipher_node rclcpp)
ament_target_dependencies(decoder_node rclcpp)

rosidl_target_interfaces(cipher_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(decoder_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install targets
install(TARGETS 
  cipher_node
  decoder_node
  DESTINATION lib/${PROJECT_NAME}
)

# Linting for tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(remote_encoder)

# Compiler settings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(remote_actuator REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Logic shared by all targets
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PositionGet.srv"
  "srv/VelocityGet.srv"
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(
  include
  ${remote_actuator_INCLUDE_DIRS}
)

set(project_SOURCE_FILES
  src/interface.cpp
  src/interface_remote.cpp
  src/implementation.cpp
)

set(terminal_SOURCE_FILES
  ${project_SOURCE_FILES}
  src/main.cpp
  src/node.cpp
)

set(fake_SOURCE_FILES
  ${project_SOURCE_FILES}
  src/fake_implementation.cpp
  src/fake_node.cpp
  src/fake.cpp
)

set(project_DEPENDENCIES
  rclcpp
  std_msgs
  remote_actuator
)

# Executable targets
add_executable(${PROJECT_NAME}_terminal ${terminal_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}_terminal ${rclcpp_LIBRARIES} "${cpp_typesupport_target}")
ament_target_dependencies(${PROJECT_NAME}_terminal ${project_DEPENDENCIES})
install(TARGETS
  ${PROJECT_NAME}_terminal
  DESTINATION lib/${PROJECT_NAME}
)

add_executable("fake" ${fake_SOURCE_FILES})
target_link_libraries("fake" ${rclcpp_LIBRARIES} "${cpp_typesupport_target}")
ament_target_dependencies("fake" ${project_DEPENDENCIES})
install(TARGETS
  "fake"
  DESTINATION lib/${PROJECT_NAME}
)

# Library target
add_library(${PROJECT_NAME}_native src/factory.cpp ${project_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}_native ${rclcpp_LIBRARIES} "${cpp_typesupport_target}")
ament_target_dependencies(${PROJECT_NAME}_native ${project_DEPENDENCIES})
ament_export_targets(${PROJECT_NAME}_native_library HAS_LIBRARY_TARGET)
install(TARGETS ${PROJECT_NAME}_native
  EXPORT ${PROJECT_NAME}_native_library
  LIBRARY DESTINATION lib
)

add_library(${PROJECT_NAME}_fake src/fake_factory.cpp ${fake_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}_fake ${rclcpp_LIBRARIES} "${cpp_typesupport_target}")
ament_target_dependencies(${PROJECT_NAME}_fake ${project_DEPENDENCIES})
ament_export_targets(${PROJECT_NAME}_fake_library HAS_LIBRARY_TARGET)
install(TARGETS ${PROJECT_NAME}_fake
  EXPORT ${PROJECT_NAME}_fake_library
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
ament_export_dependencies(${project_DEPENDENCIES})
ament_export_include_directories(include)


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

ament_export_dependencies(rosidl_default_runtime)
ament_package()

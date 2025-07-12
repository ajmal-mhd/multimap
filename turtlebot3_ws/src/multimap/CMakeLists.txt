cmake_minimum_required(VERSION 3.8)
project(multimap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PkgConfig REQUIRED)

# Find PostgreSQL (libpqxx)
pkg_check_modules(PQXX REQUIRED libpqxx)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToWaypoint.action"
)

add_executable(multi_map_navigator src/multi_map_navigator.cpp src/db_interface.cpp)

# Add include directories
target_include_directories(multi_map_navigator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PQXX_INCLUDE_DIRS}
)

# Add dependencies
ament_target_dependencies(multi_map_navigator 
  rclcpp 
  rclcpp_action 
  nav2_msgs 
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# Link the generated interfaces (updated syntax)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(multi_map_navigator "${cpp_typesupport_target}" ${PQXX_LIBRARIES})

install(TARGETS
  multi_map_navigator
  DESTINATION lib/${PROJECT_NAME})

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
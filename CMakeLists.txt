cmake_minimum_required(VERSION 3.8)
project(lite3_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# The plugin is pure Python (no custom ROS interfaces — it speaks the Lite3
# Motion Host UDP protocol directly and maps telemetry onto standard ROS types).
ament_python_install_package(lite3_plugin)

# The Livox Mid-360 user_config JSON ships with the plugin so the robot adapter
# and its sensor network config stay versioned together.
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

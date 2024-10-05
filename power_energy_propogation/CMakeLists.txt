cmake_minimum_required(VERSION 3.8)
project(power_energy_propogation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)

  # Debugging flags
  # add_compile_options(-march=native -g3 -ggdb3 -O0 -fno-inline -fno-omit-frame-pointer)

  # add_compile_options(-fsanitize=thread)
  # add_link_options(-fsanitize=thread)
  # Optimization flags
  add_compile_options(-O3 -march=native -DNDEBUG -fomit-frame-pointer)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cc_msgs REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(
  DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

ament_export_dependencies(include)
include_directories(include)

add_executable(power_node
  src/power_node.cpp
  src/power_solution.cpp
  src/planner.cpp
)
ament_target_dependencies(power_node rclcpp cc_msgs)
target_include_directories(power_node PUBLIC include)

set(HEADER_FILES
  include/power_energy_propogation/power_solution.hpp
  include/power_energy_propogation/data_struct.hpp
  include/power_energy_propogation/power_node.hpp
)

add_library(bat_lib
  src/power_solution.cpp
  src/power_node.cpp
  ${HEADER_FILES}
)
ament_target_dependencies(bat_lib rclcpp cc_msgs)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # find gtest
  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES
    test/main.cpp
    test/test_battery_aggregate.cpp
  )
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  target_link_libraries(${PROJECT_NAME}_test bat_lib)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS
  scripts/plotter_node.py
  scripts/external_load_gen.py
  scripts/filter_solutions_node.py
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  power_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

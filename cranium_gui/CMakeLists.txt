cmake_minimum_required(VERSION 3.8)
project(cranium_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)
set(THREEPP_BUILD_TESTS OFF)
set(THREEPP_BUILD_EXAMPLES OFF)
FetchContent_Declare(
  cranium_gui
  GIT_REPOSITORY https://github.com/markaren/threepp.git
  GIT_TAG 96778c41d9b396b28fef5e45b18636b1ef5926ba
)
FetchContent_MakeAvailable(cranium_gui)

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(imgui CONFIG REQUIRED)

add_executable(cranium_gui src/gui_main.cpp)
ament_target_dependencies(cranium_gui rclcpp std_msgs)
target_link_libraries(cranium_gui imgui::imgui threepp::threepp)

install(TARGETS
  cranium_gui
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

ament_package()
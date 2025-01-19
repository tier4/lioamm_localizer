cmake_minimum_required(VERSION 3.5)
project(global_initializer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fopenmp)
endif()

find_package(Eigen3 REQUIRED)
find_package(teaserpp REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "global_initializer::GlobalInitializer"
  EXECUTABLE ${PROJECT_NAME}_node
)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen teaserpp::teaser_registration teaserpp::teaser_features teaserpp::teaser_io)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)

install(FILES DESTINATION share/${PROJECT_NAME})

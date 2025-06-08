cmake_minimum_required(VERSION 3.8)
project(mpc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp_vendor REQUIRED)

# インクルードディレクトリの設定
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# ライブラリの作成（vehicle_modelのみ）
add_library(mpc_controller_lib
  src/lib/vehicle_model.cpp
  src/lib/mpc_osqp_solver.cpp
)
ament_target_dependencies(mpc_controller_lib
  rclcpp
  autoware_auto_planning_msgs
  nav_msgs
  autoware_auto_control_msgs
  osqp_vendor
)

# ノードの作成
add_executable(mpc_controller_node src/mpc_controller_node.cpp)
target_link_libraries(mpc_controller_node
  mpc_controller_lib
)

# インストールルール
install(TARGETS
  mpc_controller_node
  mpc_controller_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mpc_controller
    test/test_mpc_controller.cpp
  )
  target_link_libraries(test_mpc_controller
    mpc_controller_lib
  )

  ament_add_gtest(test_mpc_osqp_solver
    test/test_mpc_osqp_solver.cpp
  )
  target_link_libraries(test_mpc_osqp_solver
    mpc_controller_lib
  )
endif()

ament_package() 
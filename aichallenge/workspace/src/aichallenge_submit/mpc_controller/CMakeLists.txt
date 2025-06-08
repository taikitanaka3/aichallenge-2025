cmake_minimum_required(VERSION 3.8)
project(mpc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

ament_auto_find_build_dependencies()

ament_auto_add_library(mpc_controller_lib
  src/lib/mpc_osqp_solver.cpp
  src/lib/vehicle_model.cpp
  src/lib/visualization.cpp
)

# テストのみをビルド
ament_auto_add_gtest(test_mpc_osqp_solver
  test/test_mpc_osqp_solver.cpp
)
target_link_libraries(test_mpc_osqp_solver mpc_controller_lib)

install(TARGETS
  test_mpc_osqp_solver
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
)
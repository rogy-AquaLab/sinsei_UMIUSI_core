cmake_minimum_required(VERSION 3.20)
project(sinsei_umiusi_core)

# default compiler flags
add_compile_options(
  -Wall
  -Wextra
  -Wpedantic

  $<$<CONFIG:Release>:-O3>
  $<$<CONFIG:Debug>:-O0>
)

# C/C++ version required by ROS2 jazzy
# require C99
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
# require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set sccache
find_program(SCCACHE_EXE sccache)
if(SCCACHE_EXE)
  set(CMAKE_C_COMPILER_LAUNCHER ${SCCACHE_EXE})
  set(CMAKE_CXX_COMPILER_LAUNCHER ${SCCACHE_EXE})
endif()

# for clang-tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ament_auto_add_library(...)

ament_auto_add_executable(
  ${PROJECT_NAME}_behaviortree
  src/${PROJECT_NAME}/behaviortree/main.cpp
  src/${PROJECT_NAME}/behaviortree/ros_stateful_action_node.cpp
  src/${PROJECT_NAME}/behaviortree/ros_sync_action_node.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # the following line skips the linter which checks for cmake files
  set(ament_cmake_lint_cmake_FOUND TRUE)
  # the following line skips uncrustify (replaced with clang_format)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
)

# ! 以下、後ろの方に書かないと無限ループに陥るので注意
# ! ref: https://stackoverflow.com/questions/32306648/changing-compiler-with-cmake-creates-infinite-loop

# set clang as the default compiler if available
# TODO: gcovまわりのバグが解消できたら有効化
# find_program(CLANG_EXE clang)
# if(CLANG_EXE)
#   set(CMAKE_C_COMPILER ${CLANG_EXE})
#   set(CMAKE_CXX_COMPILER ${CLANG_EXE}++)
#   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fuse-ld=lld")
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fuse-ld=lld")
# else()
  set(CMAKE_C_COMPILER gcc)
  set(CMAKE_CXX_COMPILER g++)
# endif()

cmake_minimum_required(VERSION 3.5)

project(arena_camera_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	# linked GenICam hearders give warnings No such file or directory
	# GENICAM_USER_ACCEPTS_ANY_COMPILER flag need tro be defined 
	# 

	  add_compile_options(-Wall -Wextra -Wpedantic -Wunknown-pragmas)
	  message(STATE ********************************************)
endif()

# -----------------------------------------------------------------------------
#
# ALL PKGS SETTIGNS
#
# -----------------------------------------------------------------------------

# add arenasdk find file to the path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

#
# FIND DEPS
#

# finds dependencies
# - same as the ones mentioned in package.xml under
#   the tags <buildtool_depend>, <build_depend>, <exec_depend>
# - for pakages used in #include "1234.h"
# - even if a dependency is used in one package, it has to be found
#find_package(ament_cmake REQUIRED) # ?? why needed
#find_package(ament_cmake_python REQUIRED) # why needed ?
find_package(arena_sdk REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(ament_cmake REQUIRED)
#find_package(ament_cmake_python REQUIRED)
#find_package(rclpy REQUIRED)
#find_package(sensor_msgs REQUIRED)
#find_package(std_srvs REQUIRED)

#
# EXPORT DEPS
#


# export dependencies
# For packages that depends on arena_camera_node
# basically the dependecies of the depencey
# these are the one mentioned in
# - "fine_package"
# - "ament_target_dependencies"
# - forgetting to export a depencey would not show at the time of
#   building the package but when other packages use this package
#ament_export_dependencies(ament_cmake)
#ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
# Export package dependencies
#ament_export_dependencies(ament_cmake)
#ament_export_dependencies(ament_cmake_python)
#ament_export_dependencies(rclcpp)
#ament_export_dependencies(rclpy)
#ament_export_dependencies(sensor_msgs)
#ament_export_dependencies(std_srvs)

#
# LAUNCH FILES
#

# install launch files.
#install(
#  DIRECTORY launch
#  DESTINATION share/${PROJECT_NAME}/
# )

#
# HEADERS
#

# MV .hpp (s) to include dir
#install(
#  DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION include/${PROJECT_NAME}
#  FILES_MATCHING PATTERN "*.hpp")

#
# EXPORT HEADERS should it before the "ament_export_dependencies" ?
#
ament_export_include_directories(include)
#ament_export_include_directories(include ${rclcpp_INCLUDE_DIRS})


# -----------------------------------------------------------------------------
#
# ROS2 RUN ARENA_CAMERA_NODE <>
#
# -----------------------------------------------------------------------------

set(target_name some_file)
#
#
# CPP
#
#

# TARGET/EXCUTABLE : CPP -> EXECUTABLE
add_executable(${target_name} src/some_file.cpp)


# TARGET/EXCUTABLE DEPS PKGS
# a helper function that makes sure your executable or library has everything
# it needs to use. so it adds target dependencies and sets them up.
#
# - Target:
#     - first define it using add_excutable(traget)
#     - if uses another package, that package should
#       be listed as a dependency in the "ament_target_dependencies"
# - Dependencies:
#     - should be found first via "find_package(dep_package REQUIRED)"
ament_target_dependencies(${target_name}  rclcpp std_msgs)

#
# ${target_name}  TARGET HEADERS
#
target_include_directories(${target_name}  
	PRIVATE
		${arena_sdk_INCLUDE_DIRS})

#
# ${target_name}  TARGET LIBS
#
target_link_libraries(${target_name} ${arena_sdk_LIBS} )

# MV EXEC to lib dir
install(TARGETS
${target_name} 
  DESTINATION lib/${PROJECT_NAME}
)

# TARGET/LIBRARY : CPP -> .a file
# creates :
# - build/arena_camera_node/CMakeFiles/arena_camera_node.dir and sub dirs
# - build/arena_camera_node/libarena_camera_node.a
# add_library(${PROJECT_NAME} src/some_file.cpp)
# target_link_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES})

# EXPORT .a file for other pakges ?
#ament_export_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES})


# -----------------------------------------------------------------------------

#
#
# PYTHON
#
#

#ament_cmake_python()


# invistifate ----------------------------
# Install python scripts
#install(DIRECTORY arena_camera_node
#        DESTINATION lib
#        FILES_MATCHING REGEX "arena_camera_node/.+\.py")

#install(DIRECTORY arena_camera_node
#        DESTINATION ../build
#        FILES_MATCHING REGEX "arena_camera_node/.+\.py")
# TESTING ---------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

#message(FATAL_ERROR ********************************************)
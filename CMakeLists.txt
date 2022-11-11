cmake_minimum_required(VERSION 3.5)
project(lqr_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
include_directories(
      ${PROJECT_SOURCE_DIR}/include
      /usr/local/include/OsqpEigen
)

# # 设置静态库文件目录
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# # 动态库文件目录
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# # 可执行文件目录
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# find dependencies
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
# find_package(PythonLibs REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(carla_msgs REQUIRED)

############################################################################
# matplotlib-cpp
include(GNUInstallDirs)
set(PACKAGE_NAME matplotlib_cpp)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/${PACKAGE_NAME}/cmake)

# Library target
add_library(matplotlib_cpp INTERFACE)
target_include_directories(matplotlib_cpp
                           INTERFACE
                           $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
                           $<INSTALL_INTERFACE:include>
                           )
target_compile_features(matplotlib_cpp INTERFACE
                        cxx_std_20
                        )
# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(matplotlib_cpp INTERFACE
                      Python3::Python
                      Python3::Module
                      )
find_package(Python3 COMPONENTS NumPy)
if (Python3_NumPy_FOUND)
	target_link_libraries(matplotlib_cpp INTERFACE
	                      Python3::NumPy
	                      )
else ()
	target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
endif ()
install(
		TARGETS matplotlib_cpp
		EXPORT install_targets
)
###########################################################################
add_executable(laow_demo  src/main.cpp
                          src/refercenceline_smooth.cpp 
                          src/read_data.cpp 
                          src/reference_line.cpp 
                          src/matching_line.cpp
                          src/extract_point.cpp
                          src/LQR_control.cpp
                          src/lqr_control.cpp)

target_include_directories(laow_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(laow_demo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(laow_demo "std_msgs" "rclcpp" "nav_msgs" "visualization_msgs" "tf2" "tf2_ros" "tf2_geometry_msgs" "carla_msgs")# ipopt z ssl uv uWS
# ament_target_dependencies(laow_demo "rclcpp")

install(TARGETS 
  laow_demo
  DESTINATION lib/${PROJECT_NAME})

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${PYTHON_INCLUDE_DIRS})

target_link_libraries(laow_demo 
  ${PYTHON_LIBRARIES}
  matplotlib_cpp
  osqp::osqp
  OsqpEigen::OsqpEigen
)

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

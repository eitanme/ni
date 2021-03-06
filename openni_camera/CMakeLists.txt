cmake_minimum_required (VERSION 2.4.6)
include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set (ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init ()
rosbuild_add_boost_directories ()
rosbuild_check_for_sse ()

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
include_directories (${CMAKE_CURRENT_BINARY_DIR})

# Dynamic reconfigure
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

# ---[ OpenNI Driver
rosbuild_add_library (openni_driver src/openni_driver.cpp)
rosbuild_add_compile_flags (openni_driver ${SSE_FLAGS})

rosbuild_add_library (openni_nodelets src/openni_nodelet.cpp)
rosbuild_add_compile_flags (openni_nodelets ${SSE_FLAGS})
target_link_libraries (openni_nodelets openni_driver)

rosbuild_add_executable (openni_node src/openni_node.cpp)
target_link_libraries (openni_node openni_driver)

cmake_minimum_required(VERSION 3.8)
project(openni_tracker_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(orocos_kdl REQUIRED)

# Find OpenNI
find_package(PkgConfig)
pkg_check_modules(OpenNI REQUIRED libopenni)

include_directories(
    include
    ${OpenNI_INCLUDEDIR}
    ${orocos_kdl_INCLUDE_DIRS}
)

link_directories(${orocos_kdl_LIBRARY_DIRS})

# openni_tracker

add_executable(openni_tracker src/openni_tracker.cpp)

target_link_libraries(
    openni_tracker
	${OpenNI_LIBRARIES}
	${orocos_kdl_LIBRARIES}
)

ament_target_dependencies(
    openni_tracker
    geometry_msgs
    rclcpp
    tf2_ros
    tf2
)

# skeleton_markers

add_executable(skeleton_markers src/skeleton_markers.cpp)

ament_target_dependencies(
    skeleton_markers
    geometry_msgs
    visualization_msgs
    rclcpp
    tf2_ros
    tf2
)

# install

install(TARGETS
    openni_tracker
    skeleton_markers
    DESTINATION lib/${PROJECT_NAME}
)

install(FILES
    openni_tracker.xml
    DESTINATION share/${PROJECT_NAME}
)

# testing

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()

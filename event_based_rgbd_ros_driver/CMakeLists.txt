cmake_minimum_required(VERSION 2.8.3)
project(event_based_rgbd_ros_driver)
add_compile_options(-std=c++14)

execute_process(COMMAND lsb_release -cs OUTPUT_VARIABLE RELEASE_CODENAME OUTPUT_STRIP_TRAILING_WHITESPACE)

find_package(OpenCV 3.2.0 COMPONENTS core imgproc calib3d highgui REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        cv_bridge
        image_transport
        event_based_rgbd_ros_msgs
	event_based_rgbd_ros_srvs
        genmsg
        message_generation
	OpenCV
)

find_package(MetavisionSDK COMPONENTS driver REQUIRED)
find_package(MetavisionHAL REQUIRED)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME}/
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport event_based_rgbd_ros_msgs event_based_rgbd_ros_srvs
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
)

# Publisher
add_executable(publisher
    src/event_based_rgbd_ros_publisher.cpp
)

target_link_libraries(
    publisher
    MetavisionSDK::driver
    Metavision::HAL_discovery
    ${catkin_LIBRARIES}
)

add_dependencies(
    publisher
    event_based_rgbd_ros_msgs_generate_messages_cpp
    event_based_rgbd_ros_srvs_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
)

# Viewer
add_executable(viewer
    src/event_based_rgbd_ros_frame_viewer.cpp
)

target_link_libraries(
    viewer
    MetavisionSDK::driver
    ${catkin_LIBRARIES}
)

# Frame Generator
add_executable(frame_generator
    src/event_based_rgbd_ros_frame_generator.cpp
)

target_link_libraries(
    frame_generator
    MetavisionSDK::driver
    ${catkin_LIBRARIES}
)

add_dependencies(
    viewer
    frame_generator
    event_based_rgbd_ros_msgs_generate_messages_cpp
    event_based_rgbd_ros_srvs_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
)

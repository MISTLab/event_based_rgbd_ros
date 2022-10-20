cmake_minimum_required(VERSION 2.8.3)

project(event_based_rgbd_ros_dlp_gui)

set(ROS_DEPENDS
  roscpp
  std_msgs
  cv_bridge
  image_transport
  )

set(EXTERNAL_DEPENDS
  Qt5Widgets
  Qt5PrintSupport
  OpenCV
  )

set(EXTERNAL_DEPENDS_LIB
  Qt5::Widgets
  Qt5::PrintSupport
  Qt5::Core
  ${OpenCV_LIBS}
  )

set(INCLUDE_DIRECTORIES
  ${OPENCV_INCLUDE_DIRS}
  ${OpenCV_LIBS}
  )

add_compile_options(-std=c++14)

set(CMAKE_INCLUDE_CURRENT_DIR ON)


foreach(DEPEND ${EXTERNAL_DEPENDS})
  find_package(${DEPEND} REQUIRED)
endforeach(DEPEND)
find_package(OpenCV 3.2 COMPONENTS  core imgproc calib3d highgui  REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
        event_based_rgbd_ros_msgs
	event_based_rgbd_ros_srvs
        genmsg
        cv_bridge
        image_transport
        message_generation
	OpenCV	
	${ROS_DEPENDS})

catkin_package(
  INCLUDE_DIRS
    include/${PROJECT_NAME}/
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    event_based_rgbd_ros_msgs
    event_based_rgbd_ros_srvs
    ${ROS_DEPENDS}
  DEPENDS
    ${EXTERNAL_DEPENDS}
)


include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIRECTORIES}
)


file(GLOB HDRS
  "include/${PROJECT_NAME}/*.h"
)
file(GLOB SRCS
  "src/*.cpp"
  "src/*.hpp"
)
file(GLOB NODES
  "nodes/*.cpp"
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)


file(GLOB_RECURSE UI
    "include/${PROJECT_NAME}/*.ui"
)

message("Adding ${PROJECT_NAME} UI files: " )
foreach(source ${UI})
    message("  - " ${source})
endforeach(source)


qt5_wrap_ui (UI_HDRS  ${UI})

message("Generating ${PROJECT_NAME} UI Headers: " )
foreach(source ${UI_HDRS})
    message("  - " ${source})
endforeach(source)


file(GLOB QRC
    "rc/*.qrc"
)


message("buliding ${PROJECT_NAME} node: ")
foreach(NODE_FILE ${NODES})
  get_filename_component(NODE_NAME ${NODE_FILE} NAME_WE)
  message("  - " ${NODE_NAME})

  add_executable(${NODE_NAME} ${NODE_FILE} ${HDRS} ${SRCS} ${UI} ${QRC})
  add_dependencies(${NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_dependencies(
    ${NODE_NAME}
    event_based_rgbd_ros_msgs_generate_messages_cpp
    event_based_rgbd_ros_srvs_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS})
  target_link_libraries (${NODE_NAME} ${catkin_LIBRARIES}  ${EXTERNAL_DEPENDS_LIB})
  target_include_directories(${NODE_NAME} PRIVATE "include/${PROJECT_NAME}/")

  list(APPEND NODE_LIST ${NODE_NAME})

endforeach(NODE_FILE)


install(TARGETS ${NODE_LIST}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )


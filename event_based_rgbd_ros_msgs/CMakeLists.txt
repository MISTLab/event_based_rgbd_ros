cmake_minimum_required(VERSION 2.8.3)
project(event_based_rgbd_ros_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        )

add_message_files(
        FILES
        Event.msg
        EventArray.msg
	DLPColor.msg
	DLPFrequency.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS message_runtime std_msgs
)

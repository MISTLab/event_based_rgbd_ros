cmake_minimum_required(VERSION 2.8.3)
project(event_based_rgbd_ros_srvs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        )

add_service_files(
   FILES
   ChangeBiases.srv
   ChangeExposure.srv
   ChangeROI.srv
   ChangeAccTime.srv
   ChangeRefFlag.srv
   ChangePublishTopics.srv
)


generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS message_runtime std_msgs
)

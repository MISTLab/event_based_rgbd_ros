#ifndef EVENT_BASED_RGBD_ROS_FRAME_GENERATOR_H
#define EVENT_BASED_RGBD_ROS_FRAME_GENERATOR_H

#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#if CV_MAJOR_VERSION >= 4
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui.hpp>
#endif
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <mutex>

#include <event_based_rgbd_ros_msgs/Event.h>
#include <event_based_rgbd_ros_msgs/EventArray.h>
#include <event_based_rgbd_ros_srvs/ChangeAccTime.h>
#include <event_based_rgbd_ros_srvs/ChangeRefFlag.h>
#include <event_based_rgbd_ros_srvs/ChangePublishTopics.h>


class ERGBD_Frame_Generator {
public:
    /// \brief Constructor
    ERGBD_Frame_Generator();

    /// \brief Destructor
    ~ERGBD_Frame_Generator();

    /// \brief Checks if the frame generator class is initialized or not
    ///
    /// @return true if initialized and false otherwise
    bool isInitialized();

    /// \brief Checks if the reference frame is created or not
    ///
    /// @return true if created and false otherwise
    bool isRefCreated();

    /// \brief Return current display_accumulation_time
    int getDisplayAccTime();

    /// \brief Return initial display_accumulation_time
    int getInitialDisplayAccTime();

    /// \brief Set display_accumulation_time
    void setDisplayAccTime(int new_value);

    /// \brief Reset frames
    void reset_frames();

    /// \brief Creating reference frame
    void creatingRefImg();

    /// \brief Publish frames    
    void publishImages();

    /// \brief Return one publish topic flag from the publish_topics which is in binary = 0b publish_ref publish_mono publish_red publish_green publish_blue publish_bgr
    bool getOnePublishTopic(std::string topic_flag);

    /// \brief Set one publish topic flag from the publish_topics which is in binary = 0b publish_ref publish_mono publish_red publish_green publish_blue publish_bgr
    void setOnePublishTopic(std::string topic_flag, bool status);


private:
    /// \brief Callback triggered when data are received from the camera info topic
    ///
    /// It gets width and height of the sensor and calls init() function
    ///
    /// @param msg : ROS message with the camera info
    void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg);

    /// \brief Merge frames of red, green, and blue
    void mergechannels();

    /// \brief Normalize and swap frame
    void normalize_swap(cv::Mat &input_mat, int channels_number, int max_band);
    
    /// \brief If frames are initialized with the sensor width and height
    bool initialized;

    /// \brief If frame is reseting
    bool frame_reset = false;

    /// \brief Normalize frame
    void fast_normalize(cv::Mat & ref, int &max_);
    
    /// \brief Adding event to the related frame
    void add_event(event_based_rgbd_ros_msgs::Event event);
    
    void eventsArrayCallback(const event_based_rgbd_ros_msgs::EventArray::ConstPtr &msgs);

    bool captureRef(event_based_rgbd_ros_srvs::ChangeRefFlag::Request &req,
                    event_based_rgbd_ros_srvs::ChangeRefFlag::Response &res);

    bool changeAccTime(event_based_rgbd_ros_srvs::ChangeAccTime::Request &req,
                        event_based_rgbd_ros_srvs::ChangeAccTime::Response &res);

    bool changePublishTopics(event_based_rgbd_ros_srvs::ChangePublishTopics::Request &req,
                        event_based_rgbd_ros_srvs::ChangePublishTopics::Response &res);

    void init(const unsigned int &sensor_width, const unsigned int &sensor_height);


    /// \brief Node handler - the access point to communication with ROS
    ros::NodeHandle nh;

    /// \brief Subscriber to the camera info topic
    ros::Subscriber sub_info;

    /// \brief Subscriber for CD events topic
    ros::Subscriber sub_events;

    ros::ServiceServer srv_captureRefImg;
    ros::ServiceServer srv_changeAccTime;
    ros::ServiceServer srv_changePublishTopics;
    /// \brief Instance of CDFrameGenerator class that generates a frame from CD events
    //CDFrameGenerator cd_frame_generator_;

    int average_index=0;
    /// The time interval to display events up to the current time, in us
    int display_acc_time=1400;
    int input_display_acc_time=30;

    std::string camera_name;

    std::string ref_img_addr;

    image_transport::Publisher pub_bgr, pub_ref, pub_mono, pub_blue, pub_green, pub_red;

    cv::Mat mono_frame;
    cv::Mat red_frame, green_frame, blue_frame;
    cv::Mat bgr_frame;
    cv::Mat average_red, average_green, average_blue, average_ref_img;

    int frame_width=0;
    int frame_height=0;

    int max_blue=0;
    int max_green=0;
    int max_red=0;
    bool ref_created;

    int publish_topics=0 ; // = 0b publish_ref publish_mono publish_red publish_green publish_blue publish_bgr
    static constexpr std::uint8_t mask_bgr{    0b00000001 }; // represents bit 0
    static constexpr std::uint8_t mask_blue{   0b00000010 }; // represents bit 1
    static constexpr std::uint8_t mask_green{  0b00000100 }; // represents bit 2
    static constexpr std::uint8_t mask_red{    0b00001000 }; // represents bit 3
    static constexpr std::uint8_t mask_mono{   0b00010000 }; // represents bit 4
    static constexpr std::uint8_t mask_ref{    0b00100000 }; // represents bit 5

    static constexpr int THR_BIN = 0;
    static constexpr int THR_BIN_INV = 1;
    static constexpr int THR_TRUNCATE = 2;     // Keep below threshold (cut the above and keep the rest as is)
    static constexpr int THR_TO_ZERO = 3;      // Turn below threshold to zero (keep the rest as is)
    static constexpr int THR_TO_ZERO_INV = 4;  // Turn above threshold to zero (keep the rest as is)

    static constexpr int WINDOW_3X3 = 1;
    static constexpr int WINDOW_5X5 = 2;
    static constexpr int WINDOW_7X7 = 3;

    static constexpr bool ALONE_PIXEL = true;
    static constexpr bool SOLID_WINDOW = false;

};

#endif /* EVENT_BASED_RGBD_ROS_FRAME_GENERATOR_H */

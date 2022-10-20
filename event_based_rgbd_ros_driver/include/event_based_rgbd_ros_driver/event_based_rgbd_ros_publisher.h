#ifndef EVENT_BASED_RGBD_ROS_PUBLISHER_H
#define EVENT_BASED_RGBD_ROS_PUBLISHER_H

#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <metavision/sdk/driver/camera.h>
#include <metavision/hal/facilities/i_trigger_in.h>

#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <mutex>
#include <thread>

#include <event_based_rgbd_ros_msgs/Event.h>
#include <event_based_rgbd_ros_msgs/EventArray.h>
#include <event_based_rgbd_ros_srvs/ChangeBiases.h>
#include <event_based_rgbd_ros_srvs/ChangeExposure.h>
#include <event_based_rgbd_ros_srvs/ChangeROI.h>
#include <std_msgs/UInt32.h>

/// \brief Main class for ROS publisher
///
/// Publishes data from Prophesee sensor to ROS topics
class ERGBD_Publisher {
public:
    /// \brief Constructor
    ERGBD_Publisher();

    /// \brief Destructor
    ~ERGBD_Publisher();

    /// \brief Starts the camera and starts publishing data
    void startPublishing();

private:
    /// \brief Opens the camera
    bool cameraConnected();
    
    /// \brief Enable trigger-in
    bool triggerEnable(uint32_t channel);

    /// \brief Trigger-in callback
    void callbackTriggerEvents();

    /// \brief ROS Service callback function for changing camera bias
    bool changeBiase(event_based_rgbd_ros_srvs::ChangeBiases::Request  &req,
                      event_based_rgbd_ros_srvs::ChangeBiases::Response &res);

    /// \brief ROS Service callback function for changing color exposure time (DLP pattern exposure time)
    bool changeExposure(event_based_rgbd_ros_srvs::ChangeExposure::Request  &req,
                      event_based_rgbd_ros_srvs::ChangeExposure::Response &res);

    /// \brief ROS Service callback function for changing the Region Of Interest
    bool changeROI(event_based_rgbd_ros_srvs::ChangeROI::Request  &req,
                      event_based_rgbd_ros_srvs::ChangeROI::Response &res);

    /// \brief Read and print camera biases values
    void printBiases();

    /// \brief Set camera biases with default values
    void setDefaultBiases();

    /// \brief Set camera Region Of Interest
    void setROI(int width, int height, int X, int Y, bool status);
   
    /// \brief Publishes color-stamped events 1=Red, 2=Green, 3=Blue
    void publishEvents();

    /// \brief Timer callback function for calculate and publishing event rate
    void timerRateCallback(const ros::TimerEvent& event);

    /// \brief Timer callback function for publishing the camera info
    void timerInfoCallback(const ros::TimerEvent& event);

    /// \brief Node handler - the access point to communication with ROS
    ros::NodeHandle nh;

    /// \brief Publisher for camera info
    ros::Publisher pub_info;

    /// \brief Publisher for events
    ros::Publisher pub_events;

    /// \brief Publisher for event rate
    ros::Publisher pub_rate;

    /// \brief ROS Service for changing camera bias
    ros::ServiceServer srv_bias;

    /// \brief ROS Service for changing color exposure time (DLP pattern exposure time)
    ros::ServiceServer srv_expo;

    /// \brief ROS Service for changing the Region Of Interest
    ros::ServiceServer srv_roi;

    /// \brief The Region Of Interest rectangle
    Metavision::Roi::Rectangle region_rect;

    /// \brief The Region Of Interest status
    bool region_flag;

    /// \brief Instance of Camera class
    /// Used to access data from a camera
    Metavision::Camera camera;

    /// \brief Instance of Events Array
    /// Accumulated Array of events
    std::vector<Metavision::EventCD> event_buffer;

    /// \brief Message for publishing the camera info
    sensor_msgs::CameraInfo cam_info_msg;

    /// \brief Message for publishing the event rate per ms
    std_msgs::UInt32 rate_msg;

    /// \brief Path to the file with the camera settings (biases)
    std::string biases_addr;

    /// \brief Camera name in string format
    std::string camera_name;

    /// \brief Start time stamp
    ros::Time start_timestamp;

    /// \brief Timer callback time stamp
    ros::Time last_timer_callBack;

    /// \brief Color time stamps
    ros::Time red_start_time, blue_start_time, green_start_time;

    /// \brief Instance of Timers
    /// Timers for event rate calculation and publishing
    ros::Timer timer_event_rate;
    /// Timers for camera info publishing
    ros::Timer timer_cam_info;

    /// \brief Color exposure time (DLP pattern exposure time)
    int exposure_time;

    /// \brief event rate per ms
    std::uint32_t event_rate =0;

    /// \brief event numbers in each cycle of 1 ms
    std::uint32_t event_counts = 0 ;

    /// \brief If publishing events
    bool publishing;

    /// \brief DLP light color 1 = Red, 2 = Green, 3 = Blue
    int dlp_color_ = 0;

    /// \brief Time of the last received triiger event in us
    int last_t = 0;

    /// \brief If the frequency of the trigger-in is correct
    bool trig_freq_correct;

    /// \brief If number of event counted in timer callback function
    bool event_counted = false;

    /// \brief Last event color 1 = Red, 2 = Green, 3 = Blue
    int eventLightColor=1;

    /// \brief delta_time for packages cd_events
    /// Time step for packaging events in an array
    ros::Duration buffer_duration;

    /// Time stem for calculating and publishing the event rate in ns
    unsigned int timer_rate_duration_ns;

    /// \brief Event buffer time stamps
    ros::Time event_buffer_start_time, event_buffer_current_time;

    /// \brief Event rate thresholds
    static constexpr int THR_FIRST  = 5000;
    static constexpr int THR_SECOND = 8000;
};

#endif /* EVENT_BASED_RGBD_ROS_PUBLISHER_H */
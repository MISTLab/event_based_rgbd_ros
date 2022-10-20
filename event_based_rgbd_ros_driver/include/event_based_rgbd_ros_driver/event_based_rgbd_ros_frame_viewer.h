#ifndef EVENT_BASED_RGBD_ROS_FRAME_VIEWER_H_
#define EVENT_BASED_RGBD_ROS_FRAME_VIEWER_H_

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

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
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/base.hpp>

#include <event_based_rgbd_ros_msgs/Event.h>
#include <event_based_rgbd_ros_msgs/EventArray.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>


/// \brief Main class ROS listener and viewer
///
/// Listens ROS topics publishing data from frame_generator node and visualizes them on a screen
class ERGBD_Frame_Viewer {
public:
    /// \brief Constructor
    ERGBD_Frame_Viewer();

    /// \brief Destructor
    ~ERGBD_Frame_Viewer();

    /// \brief Shows currently available frames
    void showFrames();

    /// \brief Checks if the frame generator class is initialized or not
    ///
    /// @return true if initialized and false otherwise
    bool isInitialized();
    
    /// \brief  If visualizing frames
    bool show_frames = true;
    bool analyze_frames = false;

    /// \brief If the frame generators are initialized with the sensor width and height
    bool initialized;

    cv::Mat bgr_frame, ref_frame;
    cv::Mat norm_ref, filtered_norm_ref;
    cv::Mat norm_img, filtered_norm_img;
    std::string cd_window_name_;

    ros::Time last_frame_time;


private:
    /// \brief Callback triggered when data are received from the camera info topic
    /// It gets width and height of the sensor and calls init() function
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg);

    void bgrCallback(const sensor_msgs::ImageConstPtr& msg);
    void refCallback(const sensor_msgs::ImageConstPtr& msg);
    void monoCallback(const sensor_msgs::ImageConstPtr& msg);
    void blueCallback(const sensor_msgs::ImageConstPtr& msg);
    void greenCallback(const sensor_msgs::ImageConstPtr& msg);
    void redCallback(const sensor_msgs::ImageConstPtr& msg);

    int check_window(int window_size, cv::Mat frame, cv::Point pixel, int min_val, bool MODE);
    void noise_remover (cv::Mat &input_mat, int minimum_neighbour);

    void fast_normalize(cv::Mat & ref, int &max_);
 
    /// \brief Initializes the frame generators and the displayers
    bool init(const unsigned int &sensor_width, const unsigned int &sensor_height);

    /// \brief Creates a displayer
    void create_window(const std::string &window_name, const unsigned int &sensor_width,
                       const unsigned int &sensor_height, const int &shift_x = 0, const int &shift_y = 0);

    /// \brief Node handler - the access point to communication with ROS
    ros::NodeHandle nh;

    /// \brief Subscriber to the camera info topic
    ros::Subscriber sub_cam_info_;

    /// \brief Subscriber for events topic
    ros::Subscriber sub_cd_events_;

    int average_index=0;

    image_transport::Subscriber sub_bgr, sub_ref, sub_mono, sub_blue, sub_green, sub_red;

    std::string camera_name;

    cv::Mat mono_frame;
    cv::Mat red_frame, green_frame, blue_frame;
 
    cv::Mat average_red, average_green, average_blue;

    int frame_width=0;
    int frame_height=0;
    int total_pixels=0;

    int max_blue=0;
    int full_blue_pixels=0;
    int max_green=0;
    int full_green_pixels=0;
    int max_red=0;
    int full_red_pixels=0;
    int total_frames;
    int start_frames;
    int max_frames;

    bool showing_ref, showing_mono, showing_red, showing_green, showing_blue, showing_bgr;


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

    
    cv::Mat scanned_images;
    int circle_numbers=0;

    //cv::Mat frequency;
    static constexpr int TOTAL_IMAGE_NEEDED = 10;

    int num_scaned_image=0;
    bool calibrated = false;
    bool events_received = false;
    static constexpr int LED_COLUMNS = 7;
    static constexpr int LED_ROWS = 5;

    // 3D coordinates of chessboard points
    std::vector<cv::Point3f> objectPoints;
    // One vector of chessboard points for each chessboard image
    std::vector<std::vector<cv::Point3f>> arrayObjectPoints;
    std::vector<std::vector<cv::Point2f>> arrayImagePoints;
    cv::Mat cameraMatrix, distCoeffs, rvecs, tvecs;

};

#endif /* EVENT_BASED_RGBD_ROS_FRAME_VIEWER_H_ */

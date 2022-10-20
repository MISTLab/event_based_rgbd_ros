#ifndef CAMERA_GUI_H
#define CAMERA_GUI_H

#include <QWidget>
#include <ros/ros.h>
#include <qtimer.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <string.h>
#include <sensor_msgs/CameraInfo.h>


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

#include <opencv2/opencv.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>


namespace Ui {
class CameraGui;
}

class CameraGui : public QWidget
{
  Q_OBJECT

public:
  explicit CameraGui(QWidget *parent = nullptr);
  ~CameraGui();
  QImage imdisplay;
  void rateCallback(const std_msgs::UInt32::ConstPtr& msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg);
  void monoCallback(const sensor_msgs::ImageConstPtr& msg);
  void init_gui();
  bool getOnePublishTopic(int publish_topics, std::string topic_flag);
  
  cv::Mat find_centers(cv::Mat image);
  void calibrate_camera();
  bool check_num_images();
  void analyze_events(cv::Mat mono_image);

public slots:
  void spinOnce();
  void displayGrayImage();
  void displayCalibImage();
  void displayCircleImage();

private slots:
  void on_send_topic_srv_clicked();
  void on_browse_bias_file_clicked();
  void on_capture_ref_clicked();
  void on_text_fps_returnPressed();
  void on_text_exposure_returnPressed();
  void on_save_bias_file_clicked();
  
  void on_text_diff_off_returnPressed();
  void on_slide_diff_off_valueChanged();
  
  void on_text_diff_on_returnPressed();
  void on_slide_diff_on_valueChanged();
  
  void on_text_fo_returnPressed();
  void on_slide_fo_valueChanged();

  void on_text_hpf_returnPressed();
  void on_slide_hpf_valueChanged();
  
  void on_text_pr_returnPressed();
  void on_slide_pr_valueChanged();
  
  void on_text_refr_returnPressed();
  void on_slide_refr_valueChanged();

  void on_text_x_returnPressed();
  void on_text_y_returnPressed();
  void on_slide_width_valueChanged();
  void on_slide_height_valueChanged();
  void on_enable_roi_clicked();

  void on_button_down_clicked();
  void on_button_up_clicked();
  void on_button_left_clicked();
  void on_button_right_clicked();

  void on_button_select_accepted();
  void on_button_select_rejected();
  void on_button_calibrate_clicked();

  void on_text_led_row_returnPressed();
  void on_text_led_col_returnPressed();

  void on_enable_calibration_clicked();

private:
  Ui::CameraGui *ui;
  QTimer *ros_timer;

  ros::NodeHandlePtr nh_;
  ros::Subscriber rate_sub;
  ros::Subscriber cam_info_sub;
  ros::ServiceClient roi_client;
  ros::ServiceClient exposure_client;
  ros::ServiceClient bias_client;
  ros::ServiceClient topics_client;
  ros::ServiceClient capture_ref_client;
  ros::ServiceClient acc_client;
  image_transport::Subscriber sub_mono;

  cv::Mat mono_frame_msg;
  cv::Mat dis_gray_img, dis_calib_img;

  int event_rate;
  int event_rate_index;

  bool responded=true;
  bool patternfound=false;
  std::vector<cv::Point2f> centers; //this will be filled by the detected centers

  static constexpr std::uint8_t mask_bgr{    0b00000001 }; // represents bit 0
  static constexpr std::uint8_t mask_blue{   0b00000010 }; // represents bit 1
  static constexpr std::uint8_t mask_green{  0b00000100 }; // represents bit 2
  static constexpr std::uint8_t mask_red{    0b00001000 }; // represents bit 3
  static constexpr std::uint8_t mask_mono{   0b00010000 }; // represents bit 4
  static constexpr std::uint8_t mask_ref{    0b00100000 }; // represents bit 5

  /// \brief If frames are initialized with the sensor width and height
  bool initialized = false;
  int camera_width, camera_height;
  std::string camera_name;

  /// \brief Event rate thresholds
  static constexpr int THR_FIRST  = 5000;
  static constexpr int THR_SECOND = 8000;
  
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

  static constexpr int TOTAL_IMAGE_NEEDED = 10;

  int num_scaned_image=0;
  bool calibrated = false;
  bool events_received = false;
    
  int led_columns = 10;
  int led_rows = 6;

  // 3D coordinates of chessboard points
  std::vector<cv::Point3f> objectPoints;
  // One vector of chessboard points for each chessboard image
  std::vector<std::vector<cv::Point3f>> arrayObjectPoints;
  std::vector<std::vector<cv::Point2f>> arrayImagePoints;
  cv::Mat cameraMatrix, distCoeffs, rvecs, tvecs;

};

#endif // CAMERA_GUI_H
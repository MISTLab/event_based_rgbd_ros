#include "event_based_rgbd_ros_frame_viewer.h"

ERGBD_Frame_Viewer::ERGBD_Frame_Viewer() :
    nh("~"),
    initialized(false){
    camera_name = "camera";

    // Load Parameters
    nh.getParam("camera_name", camera_name);
    nh.getParam("show_frames", show_frames);
    nh.getParam("analyze_frames", analyze_frames);

    const std::string topic_cam_info        = "/ergbd/" + camera_name + "/cam_info";
    const std::string topic_event_buffer    = "/ergbd/" + camera_name + "/events_buffer";

    const std::string topic_ref_img         = "/ergbd/" + camera_name + "/frame_reference";
    const std::string topic_bgr             = "/ergbd/" + camera_name + "/frame_bgr";
    const std::string topic_mono            = "/ergbd/" + camera_name + "/frame_mono";
    const std::string topic_blue            = "/ergbd/" + camera_name + "/frame_blue";
    const std::string topic_green           = "/ergbd/" + camera_name + "/frame_green";
    const std::string topic_red             = "/ergbd/" + camera_name + "/frame_red";

    // Subscribe to camera info topic
    sub_cam_info_ = nh.subscribe(topic_cam_info, 1, &ERGBD_Frame_Viewer::cameraInfoCallback, this);
    
    image_transport::ImageTransport it(nh);
    sub_bgr     = it.subscribe(topic_bgr, 30,&ERGBD_Frame_Viewer::bgrCallback, this);
    sub_ref     = it.subscribe(topic_ref_img, 30,&ERGBD_Frame_Viewer::refCallback, this);
    sub_mono    = it.subscribe(topic_mono, 30,&ERGBD_Frame_Viewer::monoCallback, this);
    sub_blue    = it.subscribe(topic_blue, 30,&ERGBD_Frame_Viewer::blueCallback, this);
    sub_green   = it.subscribe(topic_green, 30,&ERGBD_Frame_Viewer::greenCallback, this);
    sub_red     = it.subscribe(topic_red, 30,&ERGBD_Frame_Viewer::redCallback, this);

}

ERGBD_Frame_Viewer::~ERGBD_Frame_Viewer() {
    if (!initialized)
        return;

    // Destroy the windows
    cv::destroyAllWindows();

    nh.shutdown();
}

bool ERGBD_Frame_Viewer::isInitialized() {
    return initialized;
}

void ERGBD_Frame_Viewer::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg) {
    if (initialized)
        return;

    if ((msg->width != 0) && (msg->height != 0))
        init(msg->width, msg->height);
}

void ERGBD_Frame_Viewer::monoCallback(const sensor_msgs::ImageConstPtr& msg){
    if (showing_mono)
        return;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    if (!initialized){
        init(cv_ptr->image.cols, cv_ptr->image.rows);
        return;
    }
    cv::Mat frame  =cv::Mat(frame_height, frame_width, CV_8UC1, cv::Scalar(127));
    (cv_ptr -> image).copyTo(frame);
    frame.copyTo(mono_frame);
    showing_mono = true;
}

void ERGBD_Frame_Viewer::blueCallback(const sensor_msgs::ImageConstPtr& msg){
    if (showing_blue)
        return;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    if (!initialized){
        init(cv_ptr->image.cols, cv_ptr->image.rows);
        return;
    }
    cv::Mat frame  =cv::Mat(frame_height, frame_width, CV_8UC1, cv::Scalar(127));
    (cv_ptr -> image).copyTo(frame);

    cv::normalize( frame, frame, 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );
    noise_remover(frame, 1);
    cv::normalize( frame, frame, 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );

    frame.copyTo(blue_frame);
    showing_blue = true;
}

void ERGBD_Frame_Viewer::greenCallback(const sensor_msgs::ImageConstPtr& msg){
    if (showing_green)
        return;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    if (!initialized){
        init(cv_ptr->image.cols, cv_ptr->image.rows);
        return;
    }
    cv::Mat frame  =cv::Mat(frame_height, frame_width, CV_8UC1, cv::Scalar(127));
    (cv_ptr -> image).copyTo(frame);

    cv::normalize( frame, frame, 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );
    noise_remover(frame, 1);
    cv::normalize( frame, frame, 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );

    frame.copyTo(green_frame);
    showing_green = true;
}

void ERGBD_Frame_Viewer::redCallback(const sensor_msgs::ImageConstPtr& msg){
    if (showing_red)
        return;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    if (!initialized){
        init(cv_ptr->image.cols, cv_ptr->image.rows);
        return;
    }
    cv::Mat frame  =cv::Mat(frame_height, frame_width, CV_8UC1, cv::Scalar(127));
    (cv_ptr -> image).copyTo(frame);

    cv::normalize( frame, frame, 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );
    noise_remover(frame, 1);
    cv::normalize( frame, frame, 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );

    frame.copyTo(red_frame);
    showing_red = true;
}

void ERGBD_Frame_Viewer::bgrCallback(const sensor_msgs::ImageConstPtr& msg){
    if (showing_bgr)
        return;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    if (!initialized){
        init(cv_ptr->image.cols, cv_ptr->image.rows);
        return;
    }
    
    (cv_ptr -> image).copyTo(norm_img);

    cv::Mat channels_bgr[3];

    cv::split(norm_img, channels_bgr);  
    
    cv::normalize( channels_bgr[0], channels_bgr[0], 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );

    cv::normalize( channels_bgr[1], channels_bgr[1], 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );

    cv::normalize( channels_bgr[2], channels_bgr[2], 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );

    noise_remover(channels_bgr[0], 1);
    noise_remover(channels_bgr[1], 1);
    noise_remover(channels_bgr[2], 1);

    std::vector<cv::Mat> channels;
    channels.push_back(channels_bgr[0]);
    channels.push_back(channels_bgr[1]);
    channels.push_back(channels_bgr[2]);
    cv::merge(channels,filtered_norm_img);

    cv::normalize( filtered_norm_img, filtered_norm_img, (0,0,0), (255,255,255), cv::NORM_MINMAX, -1, cv::Mat() );

    filtered_norm_img.copyTo(bgr_frame);
    showing_bgr = true;
}


void ERGBD_Frame_Viewer::refCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    if (!initialized){
        init(cv_ptr->image.cols, cv_ptr->image.rows);
        return;
    }
    (cv_ptr -> image).copyTo(norm_ref);

    cv::Mat channels_ref[3];

    cv::split(norm_ref, channels_ref);

    cv::normalize( channels_ref[0], channels_ref[0], 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );
    cv::normalize( channels_ref[1], channels_ref[1], 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );
    cv::normalize( channels_ref[2], channels_ref[2], 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );
    
    noise_remover(channels_ref[0], 1);
    noise_remover(channels_ref[1], 1);
    noise_remover(channels_ref[2], 1);

    cv::normalize( channels_ref[0], channels_ref[0], 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );
    cv::normalize( channels_ref[1], channels_ref[1], 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );
    cv::normalize( channels_ref[2], channels_ref[2], 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );

    std::vector<cv::Mat> channels;
    channels.push_back(channels_ref[0]);
    channels.push_back(channels_ref[1]);
    channels.push_back(channels_ref[2]);
    cv::merge(channels,filtered_norm_ref);
    cv::normalize( filtered_norm_ref, filtered_norm_ref, (0,0,0), (255,255,255), cv::NORM_MINMAX, -1, cv::Mat() );

    filtered_norm_ref.copyTo(ref_frame);
}


bool ERGBD_Frame_Viewer::init(const unsigned int &sensor_width, const unsigned int &sensor_height) {
    if (show_frames) {
        // Define the display window for each frame
        create_window("BGR Frame", sensor_width, sensor_height, 1300, 0);
        create_window("Reference Image", sensor_width, sensor_height, 0, 0);
        create_window("Mono Frame", sensor_width, sensor_height, 710, 0);
        create_window("Red Frame", sensor_width, sensor_height, 0, 780);
        create_window("Green Frame", sensor_width, sensor_height, 710,780);
        create_window("Blue Frame", sensor_width, sensor_height, 1300, 780);
    }

    // Initialize frames
    mono_frame  =cv::Mat(sensor_height, sensor_width, CV_8UC1, cv::Scalar(127));
        
    red_frame   =cv::Mat(sensor_height, sensor_width, CV_8UC1, cv::Scalar(0));
    green_frame =cv::Mat(sensor_height, sensor_width, CV_8UC1, cv::Scalar(0));
    blue_frame  =cv::Mat(sensor_height, sensor_width, CV_8UC1, cv::Scalar(0));

    bgr_frame      = cv::Mat(sensor_height, sensor_width, CV_8UC3, cv::Scalar(0,0,0));
    ref_frame      = cv::Mat(sensor_height, sensor_width, CV_8UC3, cv::Scalar(0,0,0));
    
    norm_ref           = cv::Mat(sensor_height, sensor_width, CV_8UC3, cv::Scalar(0,0,0));
    filtered_norm_ref  = cv::Mat(sensor_height, sensor_width, CV_8UC3, cv::Scalar(0,0,0));
    
    norm_img           = cv::Mat(sensor_height, sensor_width, CV_8UC3, cv::Scalar(0,0,0));
    filtered_norm_img  = cv::Mat(sensor_height, sensor_width, CV_8UC3, cv::Scalar(0,0,0));
    
    initialized = true;

    frame_width=sensor_width;
    frame_height=sensor_height;
    total_pixels= frame_width * frame_height;

    return true;
}

void ERGBD_Frame_Viewer::create_window(const std::string &window_name, const unsigned int &sensor_width,
                                           const unsigned int &sensor_height, const int &shift_x, const int &shift_y) {
    cv::namedWindow(window_name, CV_GUI_EXPANDED);
    cv::resizeWindow(window_name, sensor_width, sensor_height);
    cv::moveWindow(window_name, shift_x, shift_y);
}


void ERGBD_Frame_Viewer::showFrames() {
    if (!show_frames)
        return;
    if (!initialized)
        return;
    if (!bgr_frame.empty()){
        cv::imshow("BGR Frame", bgr_frame);
        showing_bgr = false;
    }
    if (!ref_frame.empty()){
        cv::imshow("Reference Image", ref_frame);
        showing_ref = false;
    }
    if (!red_frame.empty()){
        cv::Mat cv_bgr;
        cv_bgr=cv::Mat(frame_height, frame_width, CV_8UC3, cv::Scalar(0));
        cv::cvtColor( red_frame, cv_bgr, cv::COLOR_GRAY2BGR );
        cv::imshow("Red Frame", cv_bgr);
        showing_red = false;
    }
    if (!green_frame.empty()){
        cv::Mat cv_bgr;
        cv_bgr=cv::Mat(frame_height, frame_width, CV_8UC3, cv::Scalar(0));
        cv::cvtColor( green_frame, cv_bgr, cv::COLOR_GRAY2BGR );
        cv::imshow("Green Frame", cv_bgr);
        showing_green = false; 
    }
    if (!blue_frame.empty()){
        cv::Mat cv_bgr;
        cv_bgr=cv::Mat(frame_height, frame_width, CV_8UC3, cv::Scalar(0));
        cv::cvtColor( blue_frame, cv_bgr, cv::COLOR_GRAY2BGR );
        cv::imshow("Blue Frame", cv_bgr);
        showing_blue = false;
    }
    if (!mono_frame.empty()){
        cv::Mat cv_bgr;
        cv_bgr=cv::Mat(frame_height, frame_width, CV_8UC3, cv::Scalar(0));
        cv::cvtColor( mono_frame, cv_bgr, cv::COLOR_GRAY2BGR );
        cv::imshow("Mono Frame", cv_bgr);
        showing_mono = false;
    }
}


int ERGBD_Frame_Viewer::check_window(int window_size, cv::Mat frame, cv::Point pixel, int min_val, bool MODE){
  int result=0;
  
  int index_x_low = -1*(window_size);
  int index_x_high = window_size;
  int index_y_low = -1*(window_size);
  int index_y_high = window_size;

  if (pixel.x<window_size)
    index_x_low = -1*(pixel.x);
  else if (((frame_width-1)-pixel.x)<window_size)
    index_x_high = ((frame_width-1)-pixel.x);

  if (pixel.y<window_size)
    index_y_low = -1*(pixel.y);
  else if (((frame_height-1)-pixel.y)<window_size)
    index_y_high = ((frame_height-1)-pixel.y);
  
  for( int i = (pixel.x+index_x_low); i<=(pixel.x+index_x_high); i++ ) {
      for( int j = (pixel.y+index_y_low); j<=(pixel.y+index_y_high); j++ ) {
        if ((i==pixel.x)&&(j==pixel.y)) continue;
        if (MODE==ALONE_PIXEL){
          if (frame.at<uint8_t>(j, i) >= min_val) {
            result+=1;
          }
        } else if (MODE==SOLID_WINDOW){
          if (frame.at<uint8_t>(j, i) <= min_val) {
            result=1;
            return result;
          }
        }
      }
    }
  return result;
}

void ERGBD_Frame_Viewer::noise_remover(cv::Mat &input_mat, int minimum_neighbour){
    //Remove alone pixels
    cv::Point pixel;
    cv::Mat output_mat;
    output_mat=cv::Mat(frame_height, frame_width, CV_8UC1);
    input_mat.copyTo(output_mat);
    for ( int index_x = 0; index_x< frame_width; index_x++ ) {
        for ( int index_y = 0; index_y< frame_height; index_y++ ) {
            if (output_mat.at<uint8_t>(index_y, index_x) != 0){
                pixel.x=index_x;
                pixel.y=index_y;
                if ((check_window(WINDOW_3X3, output_mat, pixel, minimum_neighbour, ALONE_PIXEL))<=1){
                    output_mat.at<uint8_t>(index_y, index_x) = 0;
                }
            }
        }
    }
    std::swap(input_mat, output_mat);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ergbd_ros_frame_viewer");
    ERGBD_Frame_Viewer wv;
    ros::Rate loop_rate(30);
    while (ros::ok() && !wv.isInitialized()) {
        ros::spinOnce();
    }
    wv.last_frame_time = ros::Time::now();
    
    loop_rate = ros::Rate(30);

    int key = 255;
    
    while (ros::ok()) {
       
        if ((key & 0xff) == 'q') {
            ROS_INFO("\n\nExit!!");
            wv.show_frames=false;
            wv.initialized=false;
            break;
        }
        
        if (wv.show_frames){
            wv.showFrames();
            key = cv::waitKey(1);
        }
        ros::spinOnce();
        loop_rate.sleep();
        
    }
    cv::destroyAllWindows();
    ros::shutdown();

    return 0;
}

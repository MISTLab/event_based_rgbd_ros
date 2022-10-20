#include "event_based_rgbd_ros_frame_generator.h"

std::mutex m;

ERGBD_Frame_Generator::ERGBD_Frame_Generator() :
    nh("~"),
    display_acc_time(1),
    initialized(false) {

    // default values
    camera_name = "camera";

    bool publish_ref = false;
    bool publish_mono = false;
    bool publish_red = false;
    bool publish_green = false;
    bool publish_blue = false;
    bool publish_bgr = true;

    // Load Parameters
    nh.getParam("camera_name", camera_name);
    nh.getParam("display_accumulation_time", input_display_acc_time);
    nh.getParam("ref_img_addr", ref_img_addr);

    nh.getParam("publish_ref", publish_ref);
    nh.getParam("publish_mono", publish_mono);
    nh.getParam("publish_red", publish_red);
    nh.getParam("publish_green", publish_green);
    nh.getParam("publish_blue", publish_blue);
    nh.getParam("publish_bgr", publish_bgr);


    const std::string topic_cam_info        = "/ergbd/" + camera_name + "/cam_info";
    const std::string topic_event_buffer    = "/ergbd/" + camera_name + "/events_buffer";

    const std::string topic_ref_img         = "/ergbd/" + camera_name + "/frame_reference";
    const std::string topic_bgr             = "/ergbd/" + camera_name + "/frame_bgr";
    const std::string topic_mono            = "/ergbd/" + camera_name + "/frame_mono";
    const std::string topic_blue            = "/ergbd/" + camera_name + "/frame_blue";
    const std::string topic_green           = "/ergbd/" + camera_name + "/frame_green";
    const std::string topic_red             = "/ergbd/" + camera_name + "/frame_red";

    average_ref_img=cv::imread(ref_img_addr, cv::IMREAD_COLOR);
    if (!average_ref_img.empty()){
        ref_created=true;
        ROS_INFO("Reference image loaded!");
    }
    else{
        ref_created=false;        
    }

    publish_topics=0; // Publish topics flags in binary = 0b publish_ref publish_mono publish_red publish_green publish_blue publish_bgr
    setOnePublishTopic("publish_ref", publish_ref);
    setOnePublishTopic("publish_mono", publish_mono);
    setOnePublishTopic("publish_red", publish_red);
    setOnePublishTopic("publish_green", publish_green);
    setOnePublishTopic("publish_blue", publish_blue);
    setOnePublishTopic("publish_bgr", publish_bgr);

    // Subscribe to camera info topic
    sub_info = nh.subscribe(topic_cam_info, 1, &ERGBD_Frame_Generator::camInfoCallback, this);

    // Subscribe to events buffer topic
    sub_events = nh.subscribe(topic_event_buffer, 50000, &ERGBD_Frame_Generator::eventsArrayCallback, this);

    srv_captureRefImg = nh.advertiseService("capture_ref", &ERGBD_Frame_Generator::captureRef, this);
    srv_changeAccTime = nh.advertiseService("change_acc_time", &ERGBD_Frame_Generator::changeAccTime, this);
    srv_changePublishTopics = nh.advertiseService("publish_topics_flag", &ERGBD_Frame_Generator::changePublishTopics, this);

    image_transport::ImageTransport it(nh);
    pub_ref = it.advertise(topic_ref_img, 1);
    pub_bgr = it.advertise(topic_bgr, 1);
    pub_mono = it.advertise(topic_mono, 1);
    pub_blue = it.advertise(topic_blue, 1);
    pub_green = it.advertise(topic_green, 1);
    pub_red = it.advertise(topic_red, 1);
}

ERGBD_Frame_Generator::~ERGBD_Frame_Generator() {
    nh.shutdown();
}

bool ERGBD_Frame_Generator::isInitialized() {
    return initialized;
}

bool ERGBD_Frame_Generator::isRefCreated() {
    return ref_created;
}

int ERGBD_Frame_Generator::getInitialDisplayAccTime() {
    return input_display_acc_time;
}

int ERGBD_Frame_Generator::getDisplayAccTime() {
    return display_acc_time;
}

void ERGBD_Frame_Generator::setDisplayAccTime(int new_value) {
    display_acc_time = new_value;
}


bool ERGBD_Frame_Generator::captureRef(event_based_rgbd_ros_srvs::ChangeRefFlag::Request &req,
                event_based_rgbd_ros_srvs::ChangeRefFlag::Response &res){
    if ((req.capture_ref)&&(initialized)){
        ref_created = false;
        res.result = true;
        ROS_INFO("Capturing a new reference requested.");
        average_index = 0; 
        average_red     = cv::Mat(frame_height, frame_width, CV_8UC1, cv::Scalar(0));
        average_green   = cv::Mat(frame_height, frame_width, CV_8UC1, cv::Scalar(0));
        average_blue    = cv::Mat(frame_height, frame_width, CV_8UC1, cv::Scalar(0));
        return true;
    } else {
        res.result = false;
        return false;
    }
    return true;

}

bool ERGBD_Frame_Generator::changeAccTime(event_based_rgbd_ros_srvs::ChangeAccTime::Request &req,
                    event_based_rgbd_ros_srvs::ChangeAccTime::Response &res){
    if (req.get){
        res.result = input_display_acc_time;
        return true;
    }
    ROS_INFO("display_accumulation_time changed from %d Hz to %d Hz", input_display_acc_time, req.input_display_acc_time);
    input_display_acc_time = req.input_display_acc_time;
    res.result = input_display_acc_time;
    return true;
}

bool ERGBD_Frame_Generator::changePublishTopics(event_based_rgbd_ros_srvs::ChangePublishTopics::Request &req,
                        event_based_rgbd_ros_srvs::ChangePublishTopics::Response &res){

    if (req.get){
        res.result = publish_topics;
        return true;
    }

    const std::string topic_ref_img         = "/ergbd/" + camera_name + "/frame_reference";
    const std::string topic_bgr             = "/ergbd/" + camera_name + "/frame_bgr";
    const std::string topic_mono            = "/ergbd/" + camera_name + "/frame_mono";
    const std::string topic_blue            = "/ergbd/" + camera_name + "/frame_blue";
    const std::string topic_green           = "/ergbd/" + camera_name + "/frame_green";
    const std::string topic_red             = "/ergbd/" + camera_name + "/frame_red";

    publish_topics = req.publish_topics;
    ROS_INFO("Requested to publish topis:");
    if (publish_topics & mask_bgr)
        ROS_INFO("%s", topic_bgr.c_str());

    if (publish_topics & mask_blue)
        ROS_INFO("%s", topic_blue.c_str());
    
    if (publish_topics & mask_green)
        ROS_INFO("%s", topic_green.c_str());
    
    if (publish_topics & mask_red)
        ROS_INFO("%s", topic_red.c_str());
    
    if (publish_topics & mask_mono)
        ROS_INFO("%s", topic_mono.c_str());
    
    if (publish_topics & mask_ref){
        if (ref_created)
            ROS_INFO("%s", topic_ref_img.c_str());
        else {
            ROS_ERROR("Error occured: %s ", topic_ref_img.c_str());
            ROS_WARN("Reference image is not created yet!");
        }
    }
    res.result = publish_topics;
    return true;
}

bool ERGBD_Frame_Generator::getOnePublishTopic(std::string topic_flag){
    bool flag=false;
    if (topic_flag == "publish_bgr")
        flag = (publish_topics & mask_bgr) ? true : false;
    else if (topic_flag == "publish_blue")
        flag = (publish_topics & mask_blue) ? true : false;
    else if (topic_flag == "publish_green")
        flag = (publish_topics & mask_green) ? true : false;
    else if (topic_flag == "publish_red")
        flag = (publish_topics & mask_red) ? true : false;
    else if (topic_flag == "publish_mono")
        flag = (publish_topics & mask_mono) ? true : false;
    else if (topic_flag == "publish_ref")
        flag = (publish_topics & mask_ref) ? true : false;
    return flag;
}

void ERGBD_Frame_Generator::setOnePublishTopic(std::string topic_flag, bool status){
    if (topic_flag == "publish_bgr"){
        if (status)
            publish_topics |= mask_bgr;
        else
            publish_topics &= ~mask_bgr; 
    } else if (topic_flag == "publish_blue"){
        if (status)
            publish_topics |= mask_blue;
        else
            publish_topics &= ~mask_blue; 
    } else if (topic_flag == "publish_green"){
        if (status)
            publish_topics |= mask_green;
        else
            publish_topics &= ~mask_green;
    } else if (topic_flag == "publish_red"){
        if (status)
            publish_topics |= mask_red;
        else
            publish_topics &= ~mask_red;
    } else if (topic_flag == "publish_mono"){
        if (status)
            publish_topics |= mask_mono;
        else
            publish_topics &= ~mask_mono;
    } else if (topic_flag == "publish_ref"){
        if (status)
            publish_topics |= mask_ref;
        else
            publish_topics &= ~mask_ref;
    }
}

void ERGBD_Frame_Generator::camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg) {
    try {
        if (initialized)
            return;
        if ((msg->width != 0) && (msg->height != 0))
            init(msg->width, msg->height);
    } catch (ros::Exception &e) {
        ROS_ERROR("Error occured: %s ", e.what());
    }
}

void ERGBD_Frame_Generator::init(const unsigned int &sensor_width, const unsigned int &sensor_height) {
    // Initialize frames
    mono_frame  =cv::Mat(sensor_height, sensor_width, CV_8UC1, cv::Scalar(127));
        
    red_frame   =cv::Mat(sensor_height, sensor_width, CV_8UC1, cv::Scalar(0));
    
    green_frame =cv::Mat(sensor_height, sensor_width, CV_8UC1, cv::Scalar(0));
            
    blue_frame  =cv::Mat(sensor_height, sensor_width, CV_8UC1, cv::Scalar(0));
        
    bgr_frame   =cv::Mat(sensor_height, sensor_width, CV_8UC3, cv::Scalar(0,0,0));

    average_red     = cv::Mat(sensor_height, sensor_width, CV_8UC1, cv::Scalar(0));
    average_green   = cv::Mat(sensor_height, sensor_width, CV_8UC1, cv::Scalar(0));
    average_blue    = cv::Mat(sensor_height, sensor_width, CV_8UC1, cv::Scalar(0));
    if (!ref_created)
        average_ref_img = cv::Mat(sensor_height, sensor_width, CV_8UC3, cv::Scalar(0,0,0));

    initialized = true;

    frame_width=sensor_width;
    frame_height=sensor_height;
    ROS_INFO("Initialized frames size of (width %d, height %d).", frame_width, frame_height);
}


void ERGBD_Frame_Generator::normalize_swap(cv::Mat &input_mat, int channels_number, int max_band){
    cv::Mat output_CV_8UC1, output_CV_8UC3;
    if (channels_number==1){
        output_CV_8UC1=cv::Mat(frame_height, frame_width, CV_8UC1, cv::Scalar(0));
        input_mat.copyTo(output_CV_8UC1);
        fast_normalize(output_CV_8UC1,max_band);
        std::swap(input_mat, output_CV_8UC1);
    }
    if (channels_number==3){
        output_CV_8UC3=cv::Mat(frame_height, frame_width, CV_8UC3, cv::Scalar(0,0,0));
        input_mat.copyTo(output_CV_8UC3);
        cv::normalize( output_CV_8UC3, output_CV_8UC3, (0,0,0), (255,255,255), cv::NORM_MINMAX, -1, cv::Mat() );
        std::swap(input_mat, output_CV_8UC3);
    }
}


void ERGBD_Frame_Generator::mergechannels(){
    if (!initialized)
        return;
    std::vector<cv::Mat> channels;
    normalize_swap(red_frame,1, max_red);
    normalize_swap(green_frame,1, max_green);
    normalize_swap(blue_frame,1, max_blue);
    channels.push_back(blue_frame);
    channels.push_back(green_frame);
    channels.push_back(red_frame);
    cv::merge(channels,bgr_frame);
}

void ERGBD_Frame_Generator::publishImages(){
    try {
        if (!initialized)
            return;
    
        ros::Time frame_time = ros::Time::now();

        if ((getOnePublishTopic("publish_bgr"))&&(pub_bgr.getNumSubscribers() > 0) ) {
            mergechannels();
            sensor_msgs::ImagePtr msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr_frame).toImageMsg();
            msg_image->header.stamp = frame_time;
            pub_bgr.publish(msg_image);
        }

        if ((ref_created)&&(getOnePublishTopic("publish_ref")) && (pub_ref.getNumSubscribers() > 0) ) {
            sensor_msgs::ImagePtr msg_ref = cv_bridge::CvImage(std_msgs::Header(), "bgr8", average_ref_img).toImageMsg();
            msg_ref->header.stamp = frame_time;
            pub_ref.publish(msg_ref);
        }

        if ((getOnePublishTopic("publish_mono"))&&(pub_mono.getNumSubscribers() > 0) ) {
            sensor_msgs::ImagePtr msg_mono = cv_bridge::CvImage(std_msgs::Header(), "mono8", mono_frame).toImageMsg();
            msg_mono->header.stamp = frame_time;
            pub_mono.publish(msg_mono);
        }

        if ((getOnePublishTopic("publish_blue"))&&(pub_blue.getNumSubscribers() > 0) ) {
            if (!getOnePublishTopic("publish_bgr")) // did not normilized yet
                normalize_swap(blue_frame,1, max_blue);
            sensor_msgs::ImagePtr msg_blue = cv_bridge::CvImage(std_msgs::Header(), "mono8", blue_frame).toImageMsg();
            msg_blue->header.stamp = frame_time;
            pub_blue.publish(msg_blue);
        }

        if ((getOnePublishTopic("publish_green"))&&(pub_green.getNumSubscribers() > 0) ) {
            if (!getOnePublishTopic("publish_bgr")) // did not normilized yet
                normalize_swap(green_frame,1, max_green);
            sensor_msgs::ImagePtr msg_green = cv_bridge::CvImage(std_msgs::Header(), "mono8", green_frame).toImageMsg();
            msg_green->header.stamp = frame_time;
            pub_green.publish(msg_green);
        }

        if ((getOnePublishTopic("publish_red"))&&(pub_red.getNumSubscribers() > 0) ) {
            if (!getOnePublishTopic("publish_bgr")) // did not normilized yet
                normalize_swap(red_frame,1, max_red);
            sensor_msgs::ImagePtr msg_red = cv_bridge::CvImage(std_msgs::Header(), "mono8", red_frame).toImageMsg();
            msg_red->header.stamp = frame_time;
            pub_red.publish(msg_red);
        }

    } catch (ros::Exception &e) {
        ROS_ERROR("Error occured: %s ", e.what());
    }
    
}

void ERGBD_Frame_Generator::reset_frames(){
    if (!initialized)
        return;
    
    cv::Mat output_channel_mat,output_bgr_mat,output_mono_mat;
    output_channel_mat=cv::Mat(frame_height, frame_width, CV_8UC1, cv::Scalar(0));
    output_bgr_mat=cv::Mat(frame_height, frame_width, CV_8UC3, cv::Scalar(0,0,0));
    output_mono_mat=cv::Mat(frame_height, frame_width, CV_8UC1, cv::Scalar(127));
    
    std::swap(output_mono_mat, mono_frame);
    
    std::swap(output_channel_mat, red_frame);        
    
    output_channel_mat=cv::Mat(frame_height, frame_width, CV_8UC1, cv::Scalar(0));
    std::swap(output_channel_mat, green_frame);
    
    output_channel_mat=cv::Mat(frame_height, frame_width, CV_8UC1, cv::Scalar(0));
    std::swap(output_channel_mat, blue_frame);
    
    std::swap(output_bgr_mat, bgr_frame);
    
    max_red=0;
    max_blue=0;
    max_green=0;
}


void ERGBD_Frame_Generator::fast_normalize(cv::Mat & ref, int &max_){
    int height_ = ref.rows;
    int width_ = ref.cols;
    cv::Mat img = cv::Mat(height_, width_, CV_8UC1);
    ref.copyTo(img);
    
    std::uint8_t norm_weight;
    if (max_!=0)
        norm_weight = static_cast<uint8_t>(255/max_);
    else
        norm_weight = 1;
    img = (img * norm_weight);
    std::swap(ref, img);
}


void ERGBD_Frame_Generator::creatingRefImg(){
    if (!initialized)
        return;
    if (ref_created)
        return;
    if (!getOnePublishTopic("publish_ref"))
        return;
    if (average_index ==0)
        ROS_INFO("Creating the reference image!");
    if (average_index<10){
        if((red_frame.empty())||(green_frame.empty())||(blue_frame.empty()))
            return;
        if (display_acc_time != 1)
            display_acc_time = 1;
        normalize_swap(red_frame,1, max_red);
        normalize_swap(green_frame,1, max_red);
        normalize_swap(blue_frame,1, max_red);
        average_index +=1;
        average_red = average_red + (red_frame * 0.1);
        average_green= average_green + (green_frame * 0.1);
        average_blue = average_blue + (blue_frame * 0.1);
        ROS_INFO("Captured %d images!", average_index);
        return;
    } else {

        std::vector<cv::Mat> bgr_channels;
        
        double minVal; 
        double maxVal; 
        cv::Point minLoc; 
        cv::Point maxLoc;
        double threshold_high = 0.30;

        cv::normalize( average_blue, average_blue, 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );
        minMaxLoc( average_blue, &minVal, &maxVal, &minLoc, &maxLoc );
        cv::threshold(average_blue, average_blue, maxVal*threshold_high, 255, THR_TRUNCATE);
        cv::normalize( average_blue, average_blue, 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );
        
        cv::normalize( average_green, average_green, 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );
        minMaxLoc( average_green, &minVal, &maxVal, &minLoc, &maxLoc );
        cv::threshold(average_green, average_green, maxVal*threshold_high, 255, THR_TRUNCATE);
        cv::normalize( average_green, average_green, 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );

        cv::normalize( average_red, average_red, 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );
        minMaxLoc( average_red, &minVal, &maxVal, &minLoc, &maxLoc );
        cv::threshold(average_red, average_red, maxVal*threshold_high, 255, THR_TRUNCATE);
        cv::normalize( average_red, average_red, 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );

        bgr_channels.clear();
        bgr_channels.push_back(average_blue);
        bgr_channels.push_back(average_green);
        bgr_channels.push_back(average_red);
        cv::merge(bgr_channels,average_ref_img);

        ROS_INFO("The reference image created!");
        display_acc_time = input_display_acc_time;

        ref_created = true;
    }

}

void ERGBD_Frame_Generator::add_event(event_based_rgbd_ros_msgs::Event event){
    try {

        if (!initialized)
            return;
        if (event.polarity){
            mono_frame.at<uint8_t>(event.y, event.x)=255;
            if ((event.x<(frame_width-1))&&(event.x>0)&&(event.y<(frame_height-1))&&(event.y>0))
            switch (event.lightColor){
                case 1:
                if (red_frame.at<uint8_t>(event.y, event.x) == 255) {
                    red_frame.at<uint8_t>(event.y, event.x) = 254;
                }
                red_frame.at<uint8_t>(event.y, event.x)+=1;
                if (red_frame.at<uint8_t>(event.y, event.x) > max_red)
                    max_red = red_frame.at<uint8_t>(event.y, event.x);
                break;
                case 2:
                if (green_frame.at<uint8_t>(event.y, event.x) == 255) {
                    green_frame.at<uint8_t>(event.y, event.x) = 254;
                }
                green_frame.at<uint8_t>(event.y, event.x)+=1;
                if (green_frame.at<uint8_t>(event.y, event.x) > max_green)
                    max_green = green_frame.at<uint8_t>(event.y, event.x);
                break;
                case 3:
                if (blue_frame.at<uint8_t>(event.y, event.x) == 255) {
                    blue_frame.at<uint8_t>(event.y, event.x) = 254;
                }
                blue_frame.at<uint8_t>(event.y, event.x)+=1;
                if (blue_frame.at<uint8_t>(event.y, event.x) > max_blue)
                    max_blue = blue_frame.at<uint8_t>(event.y, event.x);
                break;
            }
        } else {
            mono_frame.at<uint8_t>(event.y, event.x)=0;
        }
    } catch (ros::Exception &e) {
        ROS_ERROR("Error occured: %s ", e.what());
    }
}


void ERGBD_Frame_Generator::eventsArrayCallback(const event_based_rgbd_ros_msgs::EventArray::ConstPtr &msgs) {
    try {
        if (!initialized)
            return;
        std::unique_lock<std::mutex> lock(m);
        int events_number=0;
        events_number=msgs -> events.size();

        if (initialized) {
            #pragma omp parallel for
                for (int i=0; i<events_number; i++)
                    add_event(msgs -> events[i]);
        }
    } catch (ros::Exception &e) {
        ROS_ERROR("Error occured: %s ", e.what());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ergbd_ros_frame_generator");

    ERGBD_Frame_Generator wv;
    ros::Rate loop_rate(1);
    while ((ros::ok()) && !(wv.isInitialized()) ) {
        ros::spinOnce();
    }
    
    wv.setDisplayAccTime(1);
    
    loop_rate = ros::Rate(wv.getDisplayAccTime());

    while (ros::ok()) {
                
        int last_acc_time=wv.getDisplayAccTime();
        wv.creatingRefImg();
        if (last_acc_time != wv.getDisplayAccTime()){
            loop_rate = ros::Rate(wv.getDisplayAccTime());
        }
        if ((wv.isRefCreated()) || (!wv.getOnePublishTopic("publish_ref")) ){
            if (wv.getInitialDisplayAccTime() != wv.getDisplayAccTime()) {
                wv.setDisplayAccTime(wv.getInitialDisplayAccTime());
                loop_rate = ros::Rate(wv.getDisplayAccTime());
            }
        }
        
        wv.publishImages();

        wv.reset_frames();
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    cv::destroyAllWindows();
    ros::shutdown();

    return 0;
}
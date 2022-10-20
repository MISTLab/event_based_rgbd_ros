#include "event_based_rgbd_ros_publisher.h"

std::mutex m;

ERGBD_Publisher::ERGBD_Publisher() :
    nh("~"),
    biases_addr("") {
    
    // default values
    camera_name = "camera";
    region_rect.width = 60;
    region_rect.height= 40;
    region_rect.x=300;
    region_rect.y=200;
    region_flag= false;
    exposure_time=235;
    publishing = true;
    trig_freq_correct = false;
    event_counted = false;

    nh.getParam("camera_name", camera_name);
    nh.getParam("biases_addr", biases_addr);
    nh.getParam("exposure_time", exposure_time);

    nh.getParam("roi_width", region_rect.width);
    nh.getParam("roi_height", region_rect.height);
    nh.getParam("roi_x", region_rect.x);
    nh.getParam("roi_y", region_rect.y);
    nh.getParam("region_flag", region_flag);

    buffer_duration = ros::Duration(nh.param<double>("buffer_duration", 350.0e-6));
    timer_rate_duration_ns = 1000000; // = 1ms

    // timer every 1 ms to calculate event rate
    timer_event_rate = nh.createTimer(ros::Duration(0, timer_rate_duration_ns), &ERGBD_Publisher::timerRateCallback, this);

    // timer every 1 s to publish camera info
    timer_cam_info = nh.createTimer(ros::Duration(1,0), &ERGBD_Publisher::timerInfoCallback, this);

    const std::string topic_cam_info        = "/ergbd/" + camera_name + "/cam_info";
    const std::string topic_event_buffer    = "/ergbd/" + camera_name + "/events_buffer";
    const std::string topic_event_rate      = "/ergbd/" + camera_name + "/events_rate";

    pub_info = nh.advertise<sensor_msgs::CameraInfo>(topic_cam_info, 1);
    pub_events = nh.advertise<event_based_rgbd_ros_msgs::EventArray>(topic_event_buffer, 1);
    pub_rate = nh.advertise<std_msgs::UInt32>(topic_event_rate, 1);

    srv_bias = nh.advertiseService("change_bias", &ERGBD_Publisher::changeBiase, this);
    srv_expo = nh.advertiseService("change_exposure", &ERGBD_Publisher::changeExposure, this);
    srv_roi = nh.advertiseService("change_roi", &ERGBD_Publisher::changeROI, this);
    
    while (!cameraConnected()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ROS_WARN("Searching for cameras...");
    }

    camera.add_runtime_error_callback([](const Metavision::CameraException &e) { ROS_WARN("%s", e.what()); });

    cam_info_msg.width           = camera.geometry().width();
    cam_info_msg.height          = camera.geometry().height();
    cam_info_msg.header.frame_id = camera_name;

    Metavision::CameraConfiguration config = camera.get_camera_configuration();

    ROS_INFO("Connected camera: %s", camera_name.c_str());
    ROS_INFO("Width:%i, Height:%i", cam_info_msg.width, cam_info_msg.height);
    ROS_INFO("Serial number: %s", config.serial_number.c_str());

    // Enable trigger. channel = 0
    triggerEnable(0);

    if (!biases_addr.empty()) {
        ROS_INFO("Bias file loaded: %s", biases_addr.c_str());
        camera.biases().set_from_file(biases_addr);
    } else {
        ROS_INFO("Set default biase values!");
        setDefaultBiases();
    }

    printBiases();

    setROI(region_rect.width, region_rect.height, region_rect.x, region_rect.y, region_flag);
}

ERGBD_Publisher::~ERGBD_Publisher() {
    camera.stop();
    nh.shutdown();
}

void ERGBD_Publisher::timerRateCallback(const ros::TimerEvent& event){
    std::unique_lock<std::mutex> lock(m);
    unsigned int delta_t = ros::Time::now().toNSec() - last_timer_callBack.toNSec();
    last_timer_callBack = ros::Time::now();

    event_rate = event_counts;
    
    /*
    if ( (delta_t > (timer_rate_duration_ns + 1000000)) || (delta_t < (timer_rate_duration_ns - 1000000))) 
        ROS_WARN("Timer duration was %d ns instead of %d ns", delta_t, timer_rate_duration_ns);
    */

    /*
    if (event_rate == 0){
        if ((pub_events.getNumSubscribers() > 0) && publishing )
            ROS_INFO("Event counts: %d , delta_t: %d ", event_counts, delta_t);
    } else if (event_rate < THR_FIRST)
        ROS_INFO("Event rate: %d ev/ms", event_rate); //event rate is normal
    else if ((event_rate >= THR_FIRST)&&(event_rate < THR_SECOND))
        ROS_WARN("Event rate: %d ev/ms", event_rate); //event rate is near to the limit (events will be published by delay)
    else
        ROS_ERROR("Event rate: %d ev/ms", event_rate); //event rate is too high
    */
    if (pub_rate.getNumSubscribers() > 0) {
        rate_msg.data = event_rate;
        pub_rate.publish(rate_msg);
    }
    event_counted = true;
}

void ERGBD_Publisher::timerInfoCallback(const ros::TimerEvent& event){
    if (pub_info.getNumSubscribers() > 0) {
        /** Get and publish camera info **/
        cam_info_msg.header.stamp = ros::Time::now();
        pub_info.publish(cam_info_msg);
    }
}

bool ERGBD_Publisher::changeBiase(event_based_rgbd_ros_srvs::ChangeBiases::Request  &req,
                                  event_based_rgbd_ros_srvs::ChangeBiases::Response &res) {
    std::string bias_name;
    int bias_value_current, bias_value_new;
    
    bias_name = req.bias_name;
    bias_value_new = req.bias_value;

    Metavision::I_LL_Biases *i_ll_biases = camera.get_device().get_facility<Metavision::I_LL_Biases>();

    if (req.get){
        if ((bias_name!= "bias_diff") && (bias_name!= "bias_diff_off") && (bias_name!= "bias_diff_on") && (bias_name!= "bias_fo") && (bias_name!= "bias_hpf") && (bias_name!= "bias_pr") && (bias_name!= "bias_refr")){
            res.result = false;
            return false;
        } else {
            bias_value_current = i_ll_biases-> get(bias_name);
            res.result = bias_value_current;
            return true;
        }
    }

    if ((bias_name!= "bias_diff") && (bias_name!= "bias_diff_off") && (bias_name!= "bias_diff_on") && (bias_name!= "bias_fo") && (bias_name!= "bias_hpf") && (bias_name!= "bias_pr") && (bias_name!= "bias_refr")){
        try{
            ROS_INFO("Bias file loaded: \n%s", bias_name.c_str());
            camera.biases().set_from_file(bias_name);
            biases_addr=bias_name;
            return true;
        } catch (Metavision::CameraException &e) {
            ROS_WARN("%s", e.what());
            if (biases_addr.empty()){
                ROS_WARN("Set default biase values!");
                setDefaultBiases();
            } else {
                ROS_WARN("Bias file loaded: %s", biases_addr.c_str());
                camera.biases().set_from_file(biases_addr);
            }
            return false;
        }
    }
        
    bias_value_current = i_ll_biases-> get(bias_name);
    try{
        i_ll_biases-> set(bias_name, bias_value_new);
    } catch (Metavision::CameraException &e) {
        ROS_WARN("%s", e.what());
        return false;
    }
    ROS_INFO("Previous %s: %d ", bias_name.c_str(), bias_value_current);
    bias_value_new = i_ll_biases-> get(bias_name);
    ROS_INFO("New %s: %d ", bias_name.c_str(), bias_value_new);
    res.result = bias_value_new;
    return true;
}


bool ERGBD_Publisher::changeExposure(event_based_rgbd_ros_srvs::ChangeExposure::Request  &req,
                                  event_based_rgbd_ros_srvs::ChangeExposure::Response &res) {
    int exposure_time_new;
    if (req.get){
        res.result= exposure_time;
        return true;
    }
    exposure_time_new = req.exposure_time;

    ROS_INFO("Previous exposure time: %d ", exposure_time);
    ROS_INFO("New exposure time: %d ", exposure_time_new);
    exposure_time = exposure_time_new;
    res.result = exposure_time;
    return true;
}


bool ERGBD_Publisher::changeROI(event_based_rgbd_ros_srvs::ChangeROI::Request  &req,
                                  event_based_rgbd_ros_srvs::ChangeROI::Response &res) {

    if (req.get){
        res.result_width    = region_rect.width;
        res.result_height   = region_rect.height;
        res.result_x        = region_rect.x;
        res.result_y        = region_rect.y;
        res.result_status   = region_flag;
        return true;
    }

    if ((req.x + req.width) > camera.geometry().width()){
        ROS_WARN("Requested x + width > camera width");
        res.result_width    = region_rect.width;
        res.result_height   = region_rect.height;
        res.result_x        = region_rect.x;
        res.result_y        = region_rect.y;
        res.result_status   = region_flag;
        return true;
    }

    if ((req.y + req.height) > camera.geometry().height()){
        ROS_WARN("Requested y + height > camera height");
        res.result_width    = region_rect.width;
        res.result_height   = region_rect.height;
        res.result_x        = region_rect.x;
        res.result_y        = region_rect.y;
        res.result_status   = region_flag;
        return true;
    }

    ROS_INFO("Previous ROI Width: %d,  Height: %d", region_rect.width, region_rect.height);
    ROS_INFO("Previous ROI X: %d,  Y: %d", region_rect.x, region_rect.y);
    ROS_INFO("Previous ROI flag: %d ", region_flag);

    region_rect.width = req.width;
    region_rect.height= req.height;
    region_rect.x=req.x;
    region_rect.y=req.y;
    region_flag = req.status;

    setROI(region_rect.width, region_rect.height, region_rect.x, region_rect.y, region_flag);

    res.result_width    = region_rect.width;
    res.result_height   = region_rect.height;
    res.result_x        = region_rect.x;
    res.result_y        = region_rect.y;
    res.result_status   = region_flag;

    return true;
}


bool ERGBD_Publisher::triggerEnable(uint32_t channel){
    Metavision::I_TriggerIn *i_trigger_in = camera.get_device().get_facility<Metavision::I_TriggerIn>();
    if (i_trigger_in->enable(channel)) {
        ROS_INFO("Trigger-in is enabled!");
    } else {
        ROS_WARN("Unable to enable the Trigger-in!");
    }
    return i_trigger_in->is_enabled(channel);
}

void ERGBD_Publisher::printBiases(){
    Metavision::I_LL_Biases *i_ll_biases = camera.get_device().get_facility<Metavision::I_LL_Biases>();
    if (i_ll_biases) {
        auto biases_to_check = i_ll_biases->get_all_biases();
        for (auto &b : biases_to_check) {
            auto v = i_ll_biases->get(b.first);
            ROS_INFO("Biase value: %s %d", b.first.c_str(), v);
        }
    }
}

void ERGBD_Publisher::setDefaultBiases(){
    Metavision::I_LL_Biases *i_ll_biases = camera.get_device().get_facility<Metavision::I_LL_Biases>();
    if (i_ll_biases) {
        i_ll_biases-> set("bias_diff", 300);
        i_ll_biases-> set("bias_diff_off", 0);
        i_ll_biases-> set("bias_diff_on", 370);
        i_ll_biases-> set("bias_fo", 1650);
        i_ll_biases-> set("bias_hpf", 1290);
        i_ll_biases-> set("bias_pr", 1400);
        i_ll_biases-> set("bias_refr", 1300);
    }
}

bool ERGBD_Publisher::cameraConnected(){
    bool camera_is_connected = false;
    try {
        camera = Metavision::Camera::from_first_available();
        camera_is_connected = true;
    } catch (Metavision::CameraException &e) { ROS_WARN("%s", e.what()); }
    return camera_is_connected;
}

void ERGBD_Publisher::setROI(int width, int height, int X, int Y, bool status){
    Metavision::I_ROI *i_roi = camera.get_device().get_facility<Metavision::I_ROI>();
    region_rect.width = width;
    region_rect.height= height;
    region_rect.x=X;
    region_rect.y=Y;
    i_roi -> set_ROI(Metavision::DeviceRoi(region_rect.x,region_rect.y, region_rect.width,region_rect.height),status);
    region_flag = status;

    ROS_INFO("New ROI Width: %d,  Height: %d", region_rect.width, region_rect.height);
    ROS_INFO("New ROI X: %d,  Y: %d", region_rect.x, region_rect.y);
    ROS_INFO("New ROI flag: %d ", region_flag);
}


void ERGBD_Publisher::startPublishing() {

    camera.start();
    start_timestamp = ros::Time::now();
    last_timer_callBack = ros::Time::now();
    timer_event_rate.start();
    timer_cam_info.start();

    callbackTriggerEvents();

    if (publishing)
        publishEvents();

    ros::spin();
}

void ERGBD_Publisher::callbackTriggerEvents() {
    try {
        std::unique_lock<std::mutex> lock(m);
        Metavision::CallbackId trig_callback = 
            camera.ext_trigger().add_callback([this](const Metavision::EventExtTrigger *trig_begin, const Metavision::EventExtTrigger *trig_end){
                for (auto ev = trig_begin; ev != trig_end; ++ev) {
                    if (!(ev->p)){
                        if ((((ev->t)-last_t)>(exposure_time+2))&&(((ev->t)-last_t)<(exposure_time+6))) {
                            dlp_color_ = 1; // Red color
                            blue_start_time.fromNSec(start_timestamp.toNSec() + (ev->t * 1000.00) );
                            event_buffer.clear();
                        } else if ((((ev->t)-last_t)>(exposure_time-2)) && (((ev->t)-last_t)<(exposure_time+2))){
                            if (dlp_color_!=0){
                                if (!trig_freq_correct)
                                    ROS_INFO("Trigger_in frequency is correct.");
                                trig_freq_correct = true;
                            }
                            if (dlp_color_==1){
                                event_buffer.clear();
                                dlp_color_ = 2; // Green color
                                red_start_time.fromNSec(start_timestamp.toNSec() + (ev->t * 1000.00) );
                            } else if (dlp_color_==2){
                                event_buffer.clear();
                                dlp_color_ = 3; // Blue color
                                green_start_time.fromNSec(start_timestamp.toNSec() + (ev->t * 1000.00) );
                            } else if (dlp_color_ == 3){
                                dlp_color_ = 1;
                                trig_freq_correct = false;
                                ROS_WARN("Error: Did not detect the blue color\nred exposure time= %d\ngreen exposure time= %d\nblue exposure time should be %d + 4!",
                                    exposure_time,exposure_time,exposure_time);
                            }
                        } else {
                            dlp_color_ = 0;
                            trig_freq_correct = false;
                            ROS_WARN("Diffrent frequency on trriger_in detected (exposure time on DLP is not %d)", exposure_time);
                        }
                    } else {
                        last_t = ev->t;
                    }
                    break;
                }
            });
    } catch (Metavision::CameraException &e) {
        ROS_WARN("%s", e.what());
        publishing = false;
    }
}


void ERGBD_Publisher::publishEvents() {
    try {
        std::unique_lock<std::mutex> lock(m);
        Metavision::CallbackId events_callback =
            camera.cd().add_callback([this](const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) {
                if (pub_events.getNumSubscribers() <= 0){
                    if (event_counted){
                        event_counts = 0;
                        event_counted = false;
                    }
                    return;
                }
                if (ev_begin < ev_end) {
                    unsigned int buffer_size = ev_end - ev_begin;

                    // Get the current time
                    event_buffer_current_time.fromNSec(start_timestamp.toNSec() + (ev_begin->t * 1000.00));

                    /** In case the buffer is empty we set the starting time stamp **/
                    if (event_buffer.empty()) {
                        // Get starting time
                        event_buffer_start_time = event_buffer_current_time;
                    }

                    /** Insert the events to the buffer **/
                    auto inserter = std::back_inserter(event_buffer);

                    /** When there is not activity filter **/
                    std::copy(ev_begin, ev_end, inserter);

                    /** Get the last time stamp **/
                    event_buffer_current_time.fromNSec(start_timestamp.toNSec() + (ev_end - 1)->t * 1000.00);
                    
                }

                if ((event_buffer_current_time - event_buffer_start_time) >= buffer_duration) {
                    /** Create the message **/
                    event_based_rgbd_ros_msgs::EventArray event_buffer_bgr_msg;

                    // Sensor geometry in header of the message
                    event_buffer_bgr_msg.header.stamp = event_buffer_current_time;
                    event_buffer_bgr_msg.height       = camera.geometry().height();
                    event_buffer_bgr_msg.width        = camera.geometry().width();

                    /** Set the buffer size for the msg **/
                    event_buffer_bgr_msg.events.resize(event_buffer.size());
                    int light_color = 0;
                    // Copy the events to the ros buffer format
                    auto buffer_msg_it = event_buffer_bgr_msg.events.begin();

                    if (event_counted){
                        event_counts = 0;
                        event_counted = false;
                    }
                    for (const Metavision::EventCD *it = std::addressof(event_buffer[0]);
                         it != std::addressof(event_buffer[event_buffer.size()]); ++it, ++buffer_msg_it) {
                        event_based_rgbd_ros_msgs::Event &event = *buffer_msg_it;
                        event.x                            = it->x;
                        event.y                            = it->y;
                        event.polarity                     = it->p;
                        event.ts.fromNSec(start_timestamp.toNSec() + (it->t * 1000.00));
                        light_color = 0;
                        event_counts +=1;
                        
                        if (((start_timestamp.toNSec() + (it ->t * 1000.00)) - red_start_time.toNSec()) <= 350000){
                            event.lightColor = 1;
                            light_color = 1;
                            eventLightColor = 1;                    
                        }
                        if (((start_timestamp.toNSec() + (it ->t * 1000.00)) - green_start_time.toNSec()) <= 350000){
                            event.lightColor = 2;
                            light_color = 2;
                            eventLightColor = 2;
                        }
                        if (((start_timestamp.toNSec() + (it ->t * 1000.00)) - blue_start_time.toNSec()) <= 350000){
                            event.lightColor = 3;
                            light_color = 3;
                            eventLightColor = 3;
                        }
                        
                        if (light_color==0){
                            if (eventLightColor==1){
                                event.lightColor = 1;
                            }
                            else if (eventLightColor==2){
                                event.lightColor = 2;
                            }
                            else if (eventLightColor==3){
                                event.lightColor = 3;
                            }
                        }
                    }

                    if (event_rate < THR_SECOND){
                        // Publish the message
                        if (publishing)
                            pub_events.publish(event_buffer_bgr_msg);
                    }
                    
                    // Clean the buffer for the next itteration
                    event_buffer.clear();
                    
                    ROS_DEBUG("CD data available, buffer size: %d at time: %lui",
                              static_cast<int>(event_buffer_bgr_msg.events.size()), event_buffer_bgr_msg.header.stamp.toNSec());
                }

            });
    } catch (Metavision::CameraException &e) {
        ROS_WARN("%s", e.what());
        publishing = false;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ergbd_ros_publisher");

    ERGBD_Publisher wp;
    wp.startPublishing();
    ros::shutdown();
    return 0;
}
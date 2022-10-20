#include "camera_gui.h"
#include "ui_camera_gui.h"
#include <event_based_rgbd_ros_srvs/ChangeAccTime.h>
#include <event_based_rgbd_ros_srvs/ChangeRefFlag.h>
#include <event_based_rgbd_ros_srvs/ChangePublishTopics.h>
#include <event_based_rgbd_ros_srvs/ChangeBiases.h>
#include <event_based_rgbd_ros_srvs/ChangeExposure.h>
#include <event_based_rgbd_ros_srvs/ChangeROI.h>


CameraGui::CameraGui(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::CameraGui)
{
  ui->setupUi(this);

  nh_.reset(new ros::NodeHandle("~"));

  // setup the timer
  ros_timer = new QTimer(this);
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(displayGrayImage()));
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(displayCalibImage()));
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(displayCircleImage()));
  ros_timer->start(10); 
  ros::Duration(0.2).sleep();
  event_rate_index = 0;

  // setup subscriber
  std::string listen_rate_topic;
  nh_->param<std::string>("listen_rate_topic",listen_rate_topic,"/ergbd/Gen3_VGA/events_rate");
  rate_sub = nh_->subscribe<std_msgs::UInt32>(listen_rate_topic, 1, &CameraGui::rateCallback, this);

  // Subscribe to camera info topic
  std::string topic_cam_info;
  nh_->param<std::string>("topic_cam_info",topic_cam_info,"/ergbd/Gen3_VGA/cam_info");
  cam_info_sub = nh_->subscribe<sensor_msgs::CameraInfo>(topic_cam_info, 1, &CameraGui::cameraInfoCallback, this);

  roi_client = nh_->serviceClient<event_based_rgbd_ros_srvs::ChangeROI>("/publisher/change_roi");
  exposure_client = nh_->serviceClient<event_based_rgbd_ros_srvs::ChangeExposure>("/publisher/change_exposure");
  bias_client = nh_->serviceClient<event_based_rgbd_ros_srvs::ChangeBiases>("/publisher/change_bias");
  
  topics_client = nh_->serviceClient<event_based_rgbd_ros_srvs::ChangePublishTopics>("/frame_generator/publish_topics_flag");
  capture_ref_client = nh_->serviceClient<event_based_rgbd_ros_srvs::ChangeRefFlag>("/frame_generator/capture_ref");
  acc_client = nh_->serviceClient<event_based_rgbd_ros_srvs::ChangeAccTime>("/frame_generator/change_acc_time");

  std::string topic_mono;
  nh_->param<std::string>("topic_mono",topic_mono,"/ergbd/Gen3_VGA/frame_mono");
  image_transport::ImageTransport it(*nh_);

  sub_mono    = it.subscribe(topic_mono, 1,&CameraGui::monoCallback, this);

  for(int y=0; y<led_rows; ++y) {
    for(int x=0; x<led_columns; ++x)
      objectPoints.push_back(cv::Point3f(x*33.5,y*33.5,0));
  }

  for(int n=0; n<TOTAL_IMAGE_NEEDED; ++n)
    arrayObjectPoints.push_back(objectPoints);

  init_gui();

}

CameraGui::~CameraGui()
{
  delete ui;
  delete ros_timer;
}


void CameraGui::spinOnce(){
  if(ros::ok()){
    ros::spinOnce();
    if (!mono_frame_msg.empty()){
      if (responded)
        analyze_events(mono_frame_msg);
    }
  }
  else
      QApplication::quit();
}

void CameraGui::monoCallback(const sensor_msgs::ImageConstPtr& msg){
    if (!(initialized))
      return;
    if (!(ui->enable_calibration->isChecked()))
      return;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat frame  =cv::Mat(camera_height, camera_width, CV_8UC1, cv::Scalar(0));
    (cv_ptr -> image).copyTo(frame);
    frame.copyTo(mono_frame_msg);
}

bool CameraGui::getOnePublishTopic(int publish_topics, std::string topic_flag){
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

void CameraGui::init_gui(){
  event_based_rgbd_ros_srvs::ChangePublishTopics srv_publish_topics;
  srv_publish_topics.request.get = true;
  int publish_topics=0;

  ui->text_number_scanned->setText(QString::number(num_scaned_image));
  ui->text_number_remained->setText(QString::number(TOTAL_IMAGE_NEEDED-num_scaned_image));

  ui->text_led_col->setText(QString::number(led_columns));
  ui->text_led_row->setText(QString::number(led_rows));

  if (topics_client.call(srv_publish_topics)){
    publish_topics = srv_publish_topics.response.result;
    if (getOnePublishTopic(publish_topics,"publish_bgr"))
      ui->bgr_frame->setChecked(true);
    else
      ui->bgr_frame->setChecked(false);

    if (getOnePublishTopic(publish_topics,"publish_blue"))
      ui->blue_frame->setChecked(true);
    else
      ui->blue_frame->setChecked(false);

    if (getOnePublishTopic(publish_topics,"publish_green"))
      ui->green_frame->setChecked(true);
    else
      ui->green_frame->setChecked(false);

    if (getOnePublishTopic(publish_topics,"publish_red"))
      ui->red_frame->setChecked(true);
    else
      ui->red_frame->setChecked(false);

    if (getOnePublishTopic(publish_topics,"publish_ref"))
      ui->ref_frame->setChecked(true);
    else
      ui->ref_frame->setChecked(false);

    if (getOnePublishTopic(publish_topics,"publish_mono"))
      ui->mono_frame->setChecked(true);
    else
      ui->mono_frame->setChecked(false);
  }


  event_based_rgbd_ros_srvs::ChangeROI srv_roi;
  srv_roi.request.get = true;
  if (roi_client.call(srv_roi)){
    ui->text_x->setText(QString::number(srv_roi.response.result_x));
    ui->text_y->setText(QString::number(srv_roi.response.result_y));
    ui->slide_width->setSliderPosition(srv_roi.response.result_width);
    ui->slide_height->setSliderPosition(srv_roi.response.result_height);
    ui->enable_roi->setChecked(srv_roi.response.result_status);
  }


  event_based_rgbd_ros_srvs::ChangeAccTime srv_acc_time;
  srv_acc_time.request.get = true;
  int acc_time=0;
  if (acc_client.call(srv_acc_time)){
    acc_time = srv_acc_time.response.result;
    ui->text_fps->setText(QString::number(acc_time));
  }

  event_based_rgbd_ros_srvs::ChangeExposure srv_exposure;
  srv_exposure.request.get = true;
  int exposure=0;
  if (exposure_client.call(srv_exposure)){
    exposure = srv_exposure.response.result;
    ui->text_exposure->setText(QString::number(exposure));
  }

  event_based_rgbd_ros_srvs::ChangeBiases srv_bias;
  srv_bias.request.get = true;

  srv_bias.request.bias_name="bias_diff_off";
  int bias_value=0;
  if (bias_client.call(srv_bias)){
    bias_value = srv_bias.response.result;
    ui->text_diff_off->setText(QString::number(bias_value));
    ui->slide_diff_off->setSliderPosition((int)bias_value);

  }

  srv_bias.request.bias_name="bias_diff_on";
  if (bias_client.call(srv_bias)){
    bias_value = srv_bias.response.result;
    ui->text_diff_on->setText(QString::number(bias_value));
    ui->slide_diff_on->setSliderPosition((int)bias_value);
  }

  srv_bias.request.bias_name="bias_fo";
  if (bias_client.call(srv_bias)){
    bias_value = srv_bias.response.result;
    ui->text_fo->setText(QString::number(bias_value));
    ui->slide_fo->setSliderPosition((int)bias_value);
  }

  srv_bias.request.bias_name="bias_hpf";
  if (bias_client.call(srv_bias)){
    bias_value = srv_bias.response.result;
    ui->text_hpf->setText(QString::number(bias_value));
    ui->slide_hpf->setSliderPosition((int)bias_value);
  }

  srv_bias.request.bias_name="bias_pr";
  if (bias_client.call(srv_bias)){
    bias_value = srv_bias.response.result;
    ui->text_pr->setText(QString::number(bias_value));
    ui->slide_pr->setSliderPosition((int)bias_value);
  }

  srv_bias.request.bias_name="bias_refr";
  if (bias_client.call(srv_bias)){
    bias_value = srv_bias.response.result;
    ui->text_refr->setText(QString::number(bias_value));
    ui->slide_refr->setSliderPosition((int)bias_value);
  }
}

void CameraGui::rateCallback(const std_msgs::UInt32::ConstPtr &msg){
  auto qstring_msg = QString::number( msg->data );

  if (event_rate_index==100){
    event_rate_index=0;
    auto qstring_msg_2 = QString::number( event_rate / 100 );
    ui->text_rate_s->setText(qstring_msg_2);
    if (( event_rate / 100 ) < THR_FIRST)
      ui->text_rate_s_box->setStyleSheet("QLabel { background-color : green;}");
    else if ((( event_rate / 100 ) > THR_FIRST) && (( event_rate / 100 ) < THR_SECOND))
      ui->text_rate_s_box->setStyleSheet("QLabel { background-color : yellow;}");
    else
      ui->text_rate_s_box->setStyleSheet("QLabel { background-color : red;}");
    event_rate = 0;
  }
  event_rate += msg->data;
  event_rate_index+=1;
  ui->text_rate_ms->setText(qstring_msg);
  
}

void CameraGui::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg){
  if (initialized)
    return;

  if ((msg->width != 0) && (msg->height != 0)){
    camera_width = msg->width;
    camera_height = msg->height;
    camera_name = msg->header.frame_id.c_str();
    initialized = true;

    auto qstring_msg = QString::number( camera_width);
    ui->text_width->setText(qstring_msg);
    qstring_msg = QString::number( camera_height);
    ui->text_height->setText(qstring_msg);
    ui->text_camera_name->setText( QString::fromStdString(camera_name));

    ui->slide_height->setMaximum(camera_height);
    ui->slide_width->setMaximum(camera_width);
    mono_frame_msg  = cv::Mat(camera_height, camera_width, CV_8UC1, cv::Scalar(0));
    scanned_images  = cv::Mat(camera_height, camera_width, CV_8UC3, cv::Scalar(0,0,0));
    dis_gray_img    = cv::Mat(camera_height, camera_width, CV_8UC1, cv::Scalar(0));
    dis_calib_img   = cv::Mat(camera_height, camera_width, CV_8UC3, cv::Scalar(0,0,0));
  }
}

void CameraGui::on_send_topic_srv_clicked()
{

  event_based_rgbd_ros_srvs::ChangePublishTopics srv_publish_topics;

  int publish_topics=0 ; // = 0b publish_ref publish_mono publish_red publish_green publish_blue publish_bgr
  

  if (ui->bgr_frame->isChecked())
	publish_topics |= mask_bgr;
  else
	publish_topics &= ~mask_bgr;

  if (ui->blue_frame->isChecked())
	publish_topics |= mask_blue;
  else
	publish_topics &= ~mask_blue;

  if (ui->green_frame->isChecked())
	publish_topics |= mask_green;
  else
	publish_topics &= ~mask_green;

  if (ui->red_frame->isChecked())
	publish_topics |= mask_red;
  else
	publish_topics &= ~mask_red;

  if (ui->mono_frame->isChecked())
	publish_topics |= mask_mono;
  else
	publish_topics &= ~mask_mono;

  if (ui->ref_frame->isChecked())
	publish_topics |= mask_ref;
  else
	publish_topics &= ~mask_ref;


  srv_publish_topics.request.publish_topics = publish_topics;

  if (!(topics_client.call(srv_publish_topics))){
    ROS_ERROR("Failed to call service ChangePublishTopics");
  }

  event_based_rgbd_ros_srvs::ChangeAccTime srv_acc_time;
  srv_acc_time.request.get = false;
  srv_acc_time.request.input_display_acc_time= ui->text_fps->text().toInt();
  if (!(acc_client.call(srv_acc_time))){
    ROS_ERROR("Failed to call service ChangeAccTime");
  }
}
void CameraGui::on_save_bias_file_clicked(){
  QString fileName;
  fileName = QFileDialog::getSaveFileName(this, QString("Enter name of the .bias file"), fileName, tr("bias files(*.bias)"));
  if(fileName.isEmpty())
    return;
  QFile file( fileName );
  if ( file.open(QIODevice::ReadWrite) ){
    QTextStream stream( &file );
    stream << ui->text_diff_off->text() << " % " << "bias_diff_off" << '\n';
    stream << ui->text_diff_on->text() << " % " << "bias_diff_on" << '\n';
    stream << ui->text_fo->text() << " % " << "bias_fo" << '\n';
    stream << ui->text_hpf->text() << " % " << "bias_hpf" << '\n';
    stream << ui->text_pr->text() << " % " << "bias_pr" << '\n';
    stream << ui->text_refr->text() << " % " << "bias_refr" << '\n';
    file.close();
  }
}

void CameraGui::on_browse_bias_file_clicked(){
  QString fileName;
  fileName = QFileDialog::getOpenFileName(this, QString("Select .bias file"), fileName, tr("bias files(*.bias)"));
  if(fileName.isEmpty())
    return;
  
  std::string absolutePath = fileName.toUtf8().constData();
  ROS_INFO("Load bias file from: %s", absolutePath.c_str());

  event_based_rgbd_ros_srvs::ChangeBiases srv_bias;
  srv_bias.request.get = false;

  srv_bias.request.bias_name=absolutePath;
  if (bias_client.call(srv_bias)){
    init_gui();
  }
}

void CameraGui::on_capture_ref_clicked(){
  if (ui->ref_frame->isChecked())
    on_send_topic_srv_clicked();

  event_based_rgbd_ros_srvs::ChangeRefFlag srv_capture_ref;
  srv_capture_ref.request.capture_ref = true;

  if (!(capture_ref_client.call(srv_capture_ref)) ){
    ROS_ERROR("Failed to call service ChangeRefFlag");
  }
}

void CameraGui::on_text_fps_returnPressed(){
  event_based_rgbd_ros_srvs::ChangeAccTime srv_acc_time;
  srv_acc_time.request.get = false;
  srv_acc_time.request.input_display_acc_time= ui->text_fps->text().toInt();
  if (!(acc_client.call(srv_acc_time))){
    ROS_ERROR("Failed to call service ChangeAccTime");
  }
}

void CameraGui::on_text_exposure_returnPressed(){
  event_based_rgbd_ros_srvs::ChangeExposure srv_exposure;
  srv_exposure.request.get = false;
  srv_exposure.request.exposure_time= ui->text_exposure->text().toInt();
  if (!(exposure_client.call(srv_exposure))){
    ROS_ERROR("Failed to call service ChangeExposure");
  }
}

void CameraGui::on_text_diff_off_returnPressed(){
  event_based_rgbd_ros_srvs::ChangeBiases srv_bias;
  srv_bias.request.get = false;
  srv_bias.request.bias_name="bias_diff_off";
  srv_bias.request.bias_value= ui->text_diff_off->text().toInt();
  if (bias_client.call(srv_bias)){
    int bias_value = srv_bias.response.result;
    ui->text_diff_off->setText(QString::number(bias_value));
    ui->slide_diff_off->setSliderPosition((int)bias_value);
  }
}

void CameraGui::on_slide_diff_off_valueChanged(){
  event_based_rgbd_ros_srvs::ChangeBiases srv_bias;
  srv_bias.request.get = false;
  srv_bias.request.bias_name="bias_diff_off";
  int bias_value =(ui->slide_diff_off->sliderPosition());
  srv_bias.request.bias_value= bias_value;
  if (bias_client.call(srv_bias)){
    int bias_value = srv_bias.response.result;
    ui->text_diff_off->setText(QString::number(bias_value));
    ui->slide_diff_off->setSliderPosition((int)bias_value);
  }
}


void CameraGui::on_text_diff_on_returnPressed(){
  event_based_rgbd_ros_srvs::ChangeBiases srv_bias;
  srv_bias.request.get = false;
  srv_bias.request.bias_name="bias_diff_on";
  srv_bias.request.bias_value= ui->text_diff_on->text().toInt();
  if (bias_client.call(srv_bias)){
    int bias_value = srv_bias.response.result;
    ui->text_diff_on->setText(QString::number(bias_value));
    ui->slide_diff_on->setSliderPosition((int)bias_value);
  }
}

void CameraGui::on_slide_diff_on_valueChanged(){
  event_based_rgbd_ros_srvs::ChangeBiases srv_bias;
  srv_bias.request.get = false;
  srv_bias.request.bias_name="bias_diff_on";
  int bias_value =(ui->slide_diff_on->sliderPosition());
  srv_bias.request.bias_value= bias_value;
  if (bias_client.call(srv_bias)){
    int bias_value = srv_bias.response.result;
    ui->text_diff_on->setText(QString::number(bias_value));
    ui->slide_diff_on->setSliderPosition((int)bias_value);
  }
}


void CameraGui::on_text_fo_returnPressed(){
  event_based_rgbd_ros_srvs::ChangeBiases srv_bias;
  srv_bias.request.get = false;
  srv_bias.request.bias_name="bias_fo";
  srv_bias.request.bias_value= ui->text_fo->text().toInt();
  if (bias_client.call(srv_bias)){
    int bias_value = srv_bias.response.result;
    ui->text_fo->setText(QString::number(bias_value));
    ui->slide_fo->setSliderPosition((int)bias_value);
  }
}

void CameraGui::on_slide_fo_valueChanged(){
  event_based_rgbd_ros_srvs::ChangeBiases srv_bias;
  srv_bias.request.get = false;
  srv_bias.request.bias_name="bias_fo";
  int bias_value =((ui->slide_fo->sliderPosition()));
  srv_bias.request.bias_value= bias_value;
  if (bias_client.call(srv_bias)){
    int bias_value = srv_bias.response.result;
    ui->text_fo->setText(QString::number(bias_value));
    ui->slide_fo->setSliderPosition((int)bias_value);
  }
}

void CameraGui::on_text_hpf_returnPressed(){
  event_based_rgbd_ros_srvs::ChangeBiases srv_bias;
  srv_bias.request.get = false;
  srv_bias.request.bias_name="bias_hpf";
  srv_bias.request.bias_value= ui->text_hpf->text().toInt();
  if (bias_client.call(srv_bias)){
    int bias_value = srv_bias.response.result;
    ui->text_hpf->setText(QString::number(bias_value));
    ui->slide_hpf->setSliderPosition((int)bias_value);
  }
}

void CameraGui::on_slide_hpf_valueChanged(){
  event_based_rgbd_ros_srvs::ChangeBiases srv_bias;
  srv_bias.request.get = false;
  srv_bias.request.bias_name="bias_hpf";
  int bias_value =((ui->slide_hpf->sliderPosition()));
  srv_bias.request.bias_value= bias_value;
  if (bias_client.call(srv_bias)){
    int bias_value = srv_bias.response.result;
    ui->text_hpf->setText(QString::number(bias_value));
    ui->slide_hpf->setSliderPosition((int)bias_value);
  }
}

void CameraGui::on_text_pr_returnPressed(){
  event_based_rgbd_ros_srvs::ChangeBiases srv_bias;
  srv_bias.request.get = false;
  srv_bias.request.bias_name="bias_pr";
  srv_bias.request.bias_value= ui->text_pr->text().toInt();
  if (bias_client.call(srv_bias)){
    int bias_value = srv_bias.response.result;
    ui->text_pr->setText(QString::number(bias_value));
    ui->slide_pr->setSliderPosition((int)bias_value);
  }
}

void CameraGui::on_slide_pr_valueChanged(){
  event_based_rgbd_ros_srvs::ChangeBiases srv_bias;
  srv_bias.request.get = false;
  srv_bias.request.bias_name="bias_pr";
  int bias_value =((ui->slide_pr->sliderPosition()));
  srv_bias.request.bias_value= bias_value;
  if (bias_client.call(srv_bias)){
    int bias_value = srv_bias.response.result;
    ui->text_pr->setText(QString::number(bias_value));
    ui->slide_pr->setSliderPosition((int)bias_value);
  }
}

void CameraGui::on_text_refr_returnPressed(){
  event_based_rgbd_ros_srvs::ChangeBiases srv_bias;
  srv_bias.request.get = false;
  srv_bias.request.bias_name="bias_refr";
  srv_bias.request.bias_value= ui->text_refr->text().toInt();
  if (bias_client.call(srv_bias)){
    int bias_value = srv_bias.response.result;
    ui->text_refr->setText(QString::number(bias_value));
    ui->slide_refr->setSliderPosition((int)bias_value);
  }
}

void CameraGui::on_slide_refr_valueChanged(){
  event_based_rgbd_ros_srvs::ChangeBiases srv_bias;
  srv_bias.request.get = false;
  srv_bias.request.bias_name="bias_refr";
  int bias_value =((ui->slide_refr->sliderPosition()));
  srv_bias.request.bias_value= bias_value;
  if (bias_client.call(srv_bias)){
    int bias_value = srv_bias.response.result;
    ui->text_refr->setText(QString::number(bias_value));
    ui->slide_refr->setSliderPosition((int)bias_value);
  }
}

void CameraGui::on_text_x_returnPressed(){
  if (ui->enable_roi->isChecked()){
    event_based_rgbd_ros_srvs::ChangeROI srv_roi;
    srv_roi.request.get = false;
    srv_roi.request.x = ui->text_x->text().toInt();
    srv_roi.request.y = ui->text_y->text().toInt();
    srv_roi.request.width = (ui->slide_width->sliderPosition());
    srv_roi.request.height = (ui->slide_height->sliderPosition());
    srv_roi.request.status = ui->enable_roi->isChecked();
    if (roi_client.call(srv_roi)){
      ui->text_x->setText(QString::number(srv_roi.response.result_x));
      ui->text_y->setText(QString::number(srv_roi.response.result_y));
      ui->slide_width->setSliderPosition(srv_roi.response.result_width);
      ui->slide_height->setSliderPosition(srv_roi.response.result_height);
      ui->enable_roi->setChecked(srv_roi.response.result_status);
    }
  }
}

void CameraGui::on_text_y_returnPressed(){
  on_text_x_returnPressed();
}

void CameraGui::on_slide_width_valueChanged(){
  on_text_x_returnPressed();
}

void CameraGui::on_slide_height_valueChanged(){
  on_text_x_returnPressed();
}

void CameraGui::on_enable_roi_clicked(){
  if (!(ui->enable_roi->isChecked()) ) {
    event_based_rgbd_ros_srvs::ChangeROI srv_roi;
    srv_roi.request.get = false;
    srv_roi.request.x = ui->text_x->text().toInt();
    srv_roi.request.y = ui->text_y->text().toInt();
    srv_roi.request.width = (ui->slide_width->sliderPosition());
    srv_roi.request.height = (ui->slide_height->sliderPosition());
    srv_roi.request.status = ui->enable_roi->isChecked();
    if (roi_client.call(srv_roi)){
      ui->text_x->setText(QString::number(srv_roi.response.result_x));
      ui->text_y->setText(QString::number(srv_roi.response.result_y));
      ui->slide_width->setSliderPosition(srv_roi.response.result_width);
      ui->slide_height->setSliderPosition(srv_roi.response.result_height);
      ui->enable_roi->setChecked(srv_roi.response.result_status);
    }
  } else
      on_text_x_returnPressed();
}

void CameraGui::on_button_up_clicked(){
  int y = ui->text_y->text().toInt();
  if (y>=20)
    y-=20;
  else
    y=0;
  ui->text_y->setText(QString::number(y));
  on_text_x_returnPressed();
}

void CameraGui::on_button_down_clicked(){
  int y = ui->text_y->text().toInt();
  if (y<=460)
    y+=20;
  ui->text_y->setText(QString::number(y));
  on_text_x_returnPressed();
}

void CameraGui::on_button_left_clicked(){
  int x = ui->text_x->text().toInt();
  if (x>=20)
    x-=20;
  else
    x=0;
  ui->text_x->setText(QString::number(x));
  on_text_x_returnPressed();
}

void CameraGui::on_button_right_clicked(){
  int x = ui->text_x->text().toInt();
  if (x<=620)
    x+=20;
  ui->text_x->setText(QString::number(x));
  on_text_x_returnPressed();
}

cv::Mat CameraGui::find_centers(cv::Mat image){
  try {
    cv::Mat im,buffer;
    buffer = cv::Mat(camera_height, camera_width, CV_8UC1, cv::Scalar(0));
    // detect edges using canny
    circle_numbers =0;
    if (!image.empty()){
      cv::Mat canny_output;

      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::dilate(image, buffer, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

      cv::Canny( buffer, canny_output, 50, 200, 3 );
      // find contours
      if (!canny_output.empty()){
        cv::findContours( canny_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
      }
      if (!contours.empty()){
        // get the moments
        std::vector<cv::Moments> mu(contours.size());
        for( int i = 0; i<contours.size(); i++ ) {
          mu[i] = cv::moments( contours[i], false );
        }
        // get the centroid of figures.
        std::vector<cv::Point> mc(contours.size());
        for( int i = 0; i<contours.size(); i++) {
          mc[i].x = cvRound(mu[i].m10/mu[i].m00);
          mc[i].y = cvRound(mu[i].m01/mu[i].m00 );
        }
        
        if ((!mc.empty())&&(mc.size()>0)){
          im = cv::Mat(camera_height, camera_width, CV_8UC1, cv::Scalar(0));
          for( int i = 0; i<mc.size(); i++ ) {
            if ((mc[i].x >=0) && (mc[i].y >=0) && (im.at<uint8_t>(mc[i]) == 0)){
              cv::circle( im, mc[i], 2, 5, -1, 8, 0 );
              circle_numbers+=1;
            }
          }
        }
      }
      buffer = cv::Mat(camera_height, camera_width, CV_8UC1, cv::Scalar(0));
      cv::threshold(im, buffer, 1, 150, 0);
    }
    return buffer;
  }
  catch ( ros::Exception &e ) {
    ROS_INFO("Catch find_centers!  %s", e.what());
    return image;
  }

  catch (cv::Exception &e ) {
    ROS_INFO("Catch find_centers!  %s", e.what());
    return image; 
  }
}

void CameraGui::calibrate_camera(){
  double RMS = 0.0;
  ROS_INFO("------ Calibration! -----");
  if (calibrated)
    RMS=cv::calibrateCamera(arrayObjectPoints, arrayImagePoints, cv::Size(camera_height,camera_width), cameraMatrix, distCoeffs, rvecs, tvecs, cv::CALIB_USE_INTRINSIC_GUESS );
  else
    RMS=cv::calibrateCamera(arrayObjectPoints, arrayImagePoints, cv::Size(camera_height,camera_width), cameraMatrix, distCoeffs, rvecs, tvecs);
  calibrated = true;
  std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
  std::cout << "distCoeffs : " << distCoeffs << std::endl;
  std::cout << "Rotation vector : " << rvecs << std::endl;
  std::cout << "Translation vector : " << tvecs << std::endl;
  std::cout << "RMS : " << RMS << std::endl;
  std::cout << "num_scaned_image : " << num_scaned_image << std::endl;
}

bool CameraGui::check_num_images(){
  if (num_scaned_image>=TOTAL_IMAGE_NEEDED){
    return true;
  } else {
    ROS_WARN("At least %d images needed. (scanned images:%d)", TOTAL_IMAGE_NEEDED, num_scaned_image);
  }
  return false;
}

void CameraGui::analyze_events(cv::Mat mono_image){
  cv::Mat binary_image_filtered;
  cv::Mat buffer_mat;
  int mc = -1;
  int keyboard = -1;
  
  binary_image_filtered = cv::Mat(camera_height, camera_width, CV_8UC1, cv::Scalar(0));

  cv::threshold(mono_image, binary_image_filtered, 150, 255, THR_BIN);
  
  while (true){
    buffer_mat = cv::Mat(camera_height, camera_width, CV_8UC1, cv::Scalar(0));
    buffer_mat = find_centers (binary_image_filtered);
    binary_image_filtered= buffer_mat.clone();
    if (mc == circle_numbers )
      break;
    mc = circle_numbers;
  }
  
  // Detected correct number of circles
  if ((mc>=(led_columns*led_rows)) && (binary_image_filtered.rows > 0) && (binary_image_filtered.cols >0)){
    buffer_mat = cv::Mat(camera_height, camera_width, CV_8UC1, cv::Scalar(0));
    cv::threshold(binary_image_filtered, buffer_mat, 10, 255, THR_BIN);
    
    dis_gray_img = buffer_mat.clone();
    binary_image_filtered= buffer_mat.clone();
  
    cv::SimpleBlobDetector::Params params;
    // Change thresholds
    params.minThreshold = 5;
    params.maxThreshold = 250;
    // Filter by Color
    params.filterByColor= true;
    params.blobColor= 255;
    // Filter by Area.
    params.filterByArea = false;
    //params.maxArea = 640*480;
    //params.minArea = 2;
    //params.minDistBetweenBlobs = 1;
    // Filter by Circularity
    params.filterByCircularity = false;
    //params.minCircularity = 0.1;
    // Filter by Convexity
    params.filterByConvexity = false;
    //params.minConvexity = 0.87;
    // Filter by Inertia
    params.filterByInertia = false;
    //params.minInertiaRatio = 0.01;
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    //check if the pattern detected
    if (responded){  
      patternfound = cv::findCirclesGrid(binary_image_filtered, cv::Size(led_columns,led_rows), centers, cv::CALIB_CB_SYMMETRIC_GRID, detector);
    }
    if (patternfound)
      responded = false;
  } else if (!mono_image.empty()) {
    cv::Mat buffer_frame1;
    cv::Mat buffer_frame2;
    cv::Mat mixed_frame;
    double alpha = 0.8; double beta;
    buffer_frame1 = cv::Mat(camera_height, camera_width, CV_8UC3, cv::Scalar(0,0,0));
    buffer_frame2 = cv::Mat(camera_height, camera_width, CV_8UC3, cv::Scalar(0,0,0));
    binary_image_filtered = cv::Mat(camera_height, camera_width, CV_8UC1, cv::Scalar(0));
    cv::threshold(mono_image, binary_image_filtered, 150, 255, THR_BIN);
    cv::cvtColor ( binary_image_filtered, buffer_frame1, CV_GRAY2BGR );
    buffer_frame2 = scanned_images.clone();
    beta = ( 1.0 - alpha );
    addWeighted( buffer_frame1, alpha, buffer_frame2, beta, 0.0, mixed_frame);
    dis_calib_img = mixed_frame.clone();
  } 
  binary_image_filtered = cv::Mat(camera_height, camera_width, CV_8UC1, cv::Scalar(0));
}

void CameraGui::displayGrayImage(){
  if (!(ui->enable_calibration->isChecked()))
    return;
  cv::Mat img = dis_gray_img.clone();
  if (img.empty())
    return;
    
  cv::resize(img, img, cv::Size(320, 240), 0, 0, cv::INTER_LINEAR);
  cv::cvtColor(img,img,CV_GRAY2RGB);
  QImage imdisplay((uchar*)img.data, img.cols, img.rows, img.step, QImage::Format_RGB888);
  ui->img_gray->setPixmap(QPixmap::fromImage(imdisplay));
    
}

void CameraGui::displayCalibImage(){
  if (!(ui->enable_calibration->isChecked()))
    return;
  cv::Mat corners_frame;
  corners_frame = cv::Mat(camera_height, camera_width, CV_8UC3, cv::Scalar(0));
  
  if (patternfound) {
    corners_frame = scanned_images.clone();
    cv::drawChessboardCorners(corners_frame, cv::Size(led_columns,led_rows), cv::Mat(centers), patternfound);
  }
  
  cv::Mat img = corners_frame.clone();
  if (img.empty())
    return;
  cv::resize(img, img, cv::Size(320, 240), 0, 0, cv::INTER_LINEAR);
  cv::cvtColor(img,img,CV_BGR2RGB);
  QImage imdisplay((uchar*)img.data, img.cols, img.rows, img.step, QImage::Format_RGB888);
  ui->img_calib->setPixmap(QPixmap::fromImage(imdisplay));
  
}

void CameraGui::displayCircleImage(){
  if (!(ui->enable_calibration->isChecked()))
    return;
  
  cv::Mat img = dis_calib_img.clone();
  if (img.empty())
    return;
  cv::resize(img, img, cv::Size(320, 240), 0, 0, cv::INTER_LINEAR);
  cv::cvtColor(img,img,CV_BGR2RGB);
  QImage imdisplay((uchar*)img.data, img.cols, img.rows, img.step, QImage::Format_RGB888);
  ui->img_circles->setPixmap(QPixmap::fromImage(imdisplay));

}
void CameraGui::on_button_select_accepted(){
  if (patternfound) {
    num_scaned_image+=1;
    arrayImagePoints.push_back(centers);
    for( int i = 0; i<centers.size(); i++ ) {
      cv::circle( scanned_images, centers[i], 10, cv::Scalar( 0, 50, 0 ), -1, 8, 0 );
    }
    if (num_scaned_image>TOTAL_IMAGE_NEEDED) {
      arrayObjectPoints.push_back(objectPoints);
    }
    ui->text_number_scanned->setText(QString::number(num_scaned_image));
    int remained = TOTAL_IMAGE_NEEDED-num_scaned_image;
    if (remained < 0)
      remained = 0;
    ui->text_number_remained->setText(QString::number(remained));
    if ((remained==0)&&(!ui->button_calibrate->isEnabled()))
      ui->button_calibrate->setEnabled(true);
    responded=true;
  }
}

void CameraGui::on_button_select_rejected(){
  responded=true;
}

void CameraGui::on_button_calibrate_clicked(){
  if (check_num_images())
    calibrate_camera();
}

void CameraGui::on_text_led_col_returnPressed(){
  int input;
  input= ui->text_led_col->text().toInt();
  if (input!=0)
    led_columns = input;
}

void CameraGui::on_text_led_row_returnPressed(){
  int input;
  input= ui->text_led_row->text().toInt();
  if (input!=0)
    led_rows = input;
}

void CameraGui::on_enable_calibration_clicked(){
  if (ui->enable_calibration->isChecked()){
    std::string absolutePath = "../catkin_ws/src/event_based_rgbd_ros/event_based_rgbd_ros_driver/cfg/bias_led.bias";
    ROS_INFO("Load bias file from: %s", absolutePath.c_str());

    event_based_rgbd_ros_srvs::ChangeBiases srv_bias;
    srv_bias.request.get = false;

    srv_bias.request.bias_name=absolutePath;
    if (bias_client.call(srv_bias)){
      init_gui();
    }
    ros::Duration(0.2).sleep();
    
    if (ui->enable_roi->isChecked()) {
      event_based_rgbd_ros_srvs::ChangeROI srv_roi;
      srv_roi.request.get = false;
      srv_roi.request.x = ui->text_x->text().toInt();
      srv_roi.request.y = ui->text_y->text().toInt();
      srv_roi.request.width = (ui->slide_width->sliderPosition());
      srv_roi.request.height = (ui->slide_height->sliderPosition());
      srv_roi.request.status = false;
      if (roi_client.call(srv_roi)){
        ui->text_x->setText(QString::number(srv_roi.response.result_x));
        ui->text_y->setText(QString::number(srv_roi.response.result_y));
        ui->slide_width->setSliderPosition(srv_roi.response.result_width);
        ui->slide_height->setSliderPosition(srv_roi.response.result_height);
        ui->enable_roi->setChecked(srv_roi.response.result_status);
      }
    }
    ui->enable_roi->setEnabled(false);
    ui->text_x->setEnabled(false);
    ui->text_y->setEnabled(false);
    ui->slide_width->setEnabled(false);
    ui->slide_height->setEnabled(false);
    ui->button_down->setEnabled(false);
    ui->button_up->setEnabled(false);
    ui->button_left->setEnabled(false);
    ui->button_right->setEnabled(false);
    ui->lbl_x->setEnabled(false);
    ui->lbl_y->setEnabled(false);

    ui->img_calib->setEnabled(true);
    ui->img_circles->setEnabled(true);
    ui->img_gray->setEnabled(true);
    ui->lbl_number_SP->setEnabled(true);
    ui->text_number_scanned->setEnabled(true);
    ui->lbl_number_SP_2->setEnabled(true);
    ui->text_number_remained->setEnabled(true);
    ui->button_select->setEnabled(true);
    ui->text_led_row->setEnabled(false);
    ui->lbl_led_row->setEnabled(false);
    ui->text_led_col->setEnabled(false);
    ui->lbl_led_col->setEnabled(false);
  } else {

    arrayObjectPoints.clear();
    for(int n=0; n<TOTAL_IMAGE_NEEDED; ++n)
      arrayObjectPoints.push_back(objectPoints);
    num_scaned_image = 0;
    int remained = TOTAL_IMAGE_NEEDED-num_scaned_image;
    ui->text_number_remained->setText(QString::number(remained));

    arrayImagePoints.clear();

    scanned_images  = cv::Mat(camera_height, camera_width, CV_8UC3, cv::Scalar(0,0,0));
    dis_gray_img    = cv::Mat(camera_height, camera_width, CV_8UC1, cv::Scalar(0));
    dis_calib_img   = cv::Mat(camera_height, camera_width, CV_8UC3, cv::Scalar(0,0,0));

    ui->enable_roi->setEnabled(true);
    ui->text_x->setEnabled(true);
    ui->text_y->setEnabled(true);
    ui->slide_width->setEnabled(true);
    ui->slide_height->setEnabled(true);
    ui->button_down->setEnabled(true);
    ui->button_up->setEnabled(true);
    ui->button_left->setEnabled(true);
    ui->button_right->setEnabled(true);
    ui->lbl_x->setEnabled(true);
    ui->lbl_y->setEnabled(true);

    ui->img_calib->setEnabled(false);
    ui->img_circles->setEnabled(false);
    ui->img_gray->setEnabled(false);
    ui->lbl_number_SP->setEnabled(false);
    ui->text_number_scanned->setEnabled(false);
    ui->lbl_number_SP_2->setEnabled(false);
    ui->text_number_remained->setEnabled(false);
    ui->button_select->setEnabled(false);
    ui->text_led_row->setEnabled(true);
    ui->lbl_led_row->setEnabled(true);
    ui->text_led_col->setEnabled(true);
    ui->lbl_led_col->setEnabled(true);

  }
}
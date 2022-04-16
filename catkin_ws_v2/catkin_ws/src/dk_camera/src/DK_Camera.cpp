//
// Created by slam on 2020/6/29.
//

#include "../include/DK_Camera.h"

// for log
#include <unistd.h>
bool log(std::string level, std::string msgs)
{
  // which user
  std::string user = "dk_camera";

  if (vfork() == 0)
  {
    if (execl("/home/slam/slam_log/RecordLog.sh", "RecordLog.sh", user.data(), level.data(), msgs.data(), nullptr) == -1)
      return false;
    return true;
  }
  return false;
}

DK_Camera::DK_Camera(std::shared_ptr<k4a::device> device)
{
  dev_ = device;

  index_ = 0;
  image_save_path_ = "";

  /**********flag**************/
  hdr_flag_ = false;
  bgra2bgr_flag_ = false;
  save_img_flag_ = false;
  data_pub_flag_ = false;
  camera_on_flag_ = false;
  stop_flag_ = true;

  configuration_ = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  // 5, 15, 30
  configuration_.camera_fps = k4a_fps_t::K4A_FRAMES_PER_SECOND_15;
  configuration_.color_format = k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_BGRA32;
  configuration_.color_resolution = k4a_color_resolution_t::K4A_COLOR_RESOLUTION_720P;
  configuration_.depth_mode = k4a_depth_mode_t::K4A_DEPTH_MODE_WFOV_UNBINNED;
  configuration_.synchronized_images_only = true;

  calibration_data_ = dev_->get_calibration(K4A_DEPTH_MODE_WFOV_UNBINNED, K4A_COLOR_RESOLUTION_720P);

  dk_camera_.camera_id = camera_no_;

  dk_color_image_pub_ = privateNh_.advertise<sensor_msgs::Image>("dk_color", 36);
  dk_depth_image_pub_ = privateNh_.advertise<sensor_msgs::Image>("dk_depth", 36);
  dk_imu_pub_ = privateNh_.advertise<sensor_msgs::Imu>("dk_imu", 1);
  dk_runtime_pub_ = privateNh_.advertise<std_msgs::String>("runtime_state", 10);
}

cv::Mat DK_Camera::getExposureImage(EXPOSURE_LEVEL exposure_time)
{
  if (exposure_time == EXPOSURE_LEVEL::LEVEL_AUTO)
  {
    dev_->set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_AUTO, 0);
  }
  else
  {
    dev_->set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_MANUAL, exposure_time);
  }
  dev_->start_cameras(&configuration_);
  sleep(0.2);
  // Get a capture and check
  k4a::capture cap;
  auto result = dev_->get_capture(&cap, std::chrono::milliseconds(K4A_WAIT_INFINITE));
  dev_->stop_cameras();

  if (!result)
  {
    std::abort();
    // return cv::Mat();
  }

  k4a::image k4a_color_img = cap.get_color_image();
  auto e_time = k4a_color_img.get_exposure();
  auto wb = k4a_color_img.get_white_balance();
  cv::Mat cv_color_img = get_mat(k4a_color_img);
  cv::cvtColor(cv_color_img, cv_color_img, cv::COLOR_BGRA2BGR);
  // cv::imwrite("/home/slam/data_cap/hdr/"+std::to_string(exposure_time)+".jpg",cv_color_img);
  return cv_color_img;
}

void DK_Camera::openCamera()
{
  /***************
  std::vector<cv::Mat> hdr_mat;
  hdr_mat.emplace_back(getOne(EXPOSURE_LEVEL::LEVEL_AUTO)) ;
  //hdr_mat.emplace_back(getOne(EXPOSURE_LEVEL::LEVEL_2)) ;
  hdr_mat.emplace_back(getOne(EXPOSURE_LEVEL::LEVEL_3)) ;
  hdr_mat.emplace_back(getOne(EXPOSURE_LEVEL::LEVEL_4)) ;
  hdr_mat.emplace_back(getOne(EXPOSURE_LEVEL::LEVEL_6)) ;
  hdr_mat.emplace_back(getOne(EXPOSURE_LEVEL::LEVEL_7)) ;
  mertens(hdr_mat,std::string(""));
   *************************/
}

void DK_Camera::getImage()
{
  std::vector<cv::Mat> hdr_mat_vec;
  k4a::capture cap;
  sleep(1);
  dev_->start_cameras(&configuration_);

  auto result = dev_->get_capture(&cap, std::chrono::milliseconds(K4A_WAIT_INFINITE));
  if (!result)
  {
    return;
  }
  k4a::image color_image = cap.get_color_image();
  int get_val;
  auto mode = K4A_COLOR_CONTROL_MODE_MANUAL;
  sleep(1);
  dev_->get_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, &mode, &get_val);
  std::cout << "get val: " << get_val << std::endl;
  // k4a::image depth_image = cap.get_depth_image();
  auto a_value = color_image.get_exposure();
  auto wb_value = color_image.get_white_balance();
  std::cout << "wb: " << wb_value << std::endl;
  dev_->set_color_control(K4A_COLOR_CONTROL_WHITEBALANCE, K4A_COLOR_CONTROL_MODE_MANUAL, wb_value);
  std::cout << "set exposure: " << a_value.count() << std::endl;
  cv::Mat color_mat = get_mat(color_image);
  cv::Mat color_mat_3;
  cv::cvtColor(color_mat, color_mat_3, cv::COLOR_BGRA2BGR);

  cv::imwrite("/home/slam/data_cap/001.jpg", color_mat_3);
}

DK_Camera::~DK_Camera()
{
  // google::ShutdownGoogleLogging();
  dev_->close();
}

void DK_Camera::closeDevice()
{
  this->dev_->close();
}

cv::Mat DK_Camera::get_mat(k4a::image &src, bool deep_copy)
{
  assert(src.get_size() != 0);

  cv::Mat mat;
  const int32_t width = src.get_width_pixels();
  const int32_t height = src.get_height_pixels();

  const k4a_image_format_t format = src.get_format();
  switch (format)
  {
  case k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_MJPG:
  {
    // NOTE: this is slower than other formats.
    std::vector<uint8_t> buffer(src.get_buffer(), src.get_buffer() + src.get_size());
    mat = cv::imdecode(buffer, cv::IMREAD_ANYCOLOR);
    cv::cvtColor(mat, mat, cv::COLOR_BGR2BGRA);
    break;
  }
  case k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_NV12:
  {
    cv::Mat nv12 = cv::Mat(height + height / 2, width, CV_8UC1, src.get_buffer()).clone();
    cv::cvtColor(nv12, mat, cv::COLOR_YUV2BGRA_NV12);
    break;
  }
  case k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_YUY2:
  {
    cv::Mat yuy2 = cv::Mat(height, width, CV_8UC2, src.get_buffer()).clone();
    cv::cvtColor(yuy2, mat, cv::COLOR_YUV2BGRA_YUY2);
    break;
  }
  case k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_BGRA32:
  {
    mat = deep_copy ? cv::Mat(height, width, CV_8UC4, src.get_buffer()).clone() // or CV_8UC4
                    : cv::Mat(height, width, CV_8UC4, src.get_buffer());
    break;
  }
  case k4a_image_format_t::K4A_IMAGE_FORMAT_DEPTH16:
  case k4a_image_format_t::K4A_IMAGE_FORMAT_IR16:
  {
    mat = deep_copy ? cv::Mat(height, width, CV_16UC1, reinterpret_cast<uint16_t *>(src.get_buffer())).clone()
                    : cv::Mat(height, width, CV_16UC1, reinterpret_cast<uint16_t *>(src.get_buffer()));
    break;
  }
  case k4a_image_format_t::K4A_IMAGE_FORMAT_CUSTOM8:
  {
    mat = cv::Mat(height, width, CV_8UC1, src.get_buffer()).clone();
    break;
  }
  case k4a_image_format_t::K4A_IMAGE_FORMAT_CUSTOM:
  {
    // NOTE: This is opencv_viz module format (cv::viz::WCloud).
    const int16_t *buffer = reinterpret_cast<int16_t *>(src.get_buffer());
    mat = cv::Mat(height, width, CV_32FC3, cv::Vec3f::all(std::numeric_limits<float>::quiet_NaN()));
    mat.forEach<cv::Vec3f>(
        [&](cv::Vec3f &point, const int32_t *position) {
          const int32_t index = (position[0] * width + position[1]) * 3;
          point = cv::Vec3f(buffer[index + 0], buffer[index + 1], buffer[index + 2]);
        });
    break;
  }
  default:
    throw k4a::error("Failed to convert this format!");
    break;
  }

  return mat;
}

void DK_Camera::sleep(float seconds)
{
  usleep(seconds * 1000000);
}

void DK_Camera::mertens(std::vector<cv::Mat> &exposure_images,
                        const std::string &image_save)
{
  auto merge_mertens = cv::createMergeMertens();
  cv::Mat fusion;
  merge_mertens->process(exposure_images, fusion);
  //进行伽马矫正，需根据实际图像调节参数，2.2f可满足大多数显示情况
  /*auto tonemap = createTonemap(2.2f);
      tonemap->process(fusion, fusion);*/
  fusion = fusion * 255;
  fusion.convertTo(fusion, CV_8UC3);
  imshow("Mertens", fusion);
  cv::waitKey(0);
  // cv::imwrite(image_save.data(), fusion);
}

void DK_Camera::setHdrFlag(bool value)
{
  hdr_flag_ = value;
}

std::string DK_Camera::getImageName(const size_t image_index)
{
  if (image_index < 10)
  {
    return "00" + std::to_string(image_index);
  }
  else if (image_index < 100)
  {
    return "0" + std::to_string(image_index);
  }
  else
  {
    return std::to_string(image_index);
  }
}

void DK_Camera::GetOneForTest()
{
  // TODO  for test
  k4a::capture cap;
  if (!dev_->get_capture(&cap, std::chrono::milliseconds(1000))) // wait 1000 ms
  {
    log("ERROR", camera_name_ + " no capture arrive in 3000 ms");
    return;
  }

  // Get image from capture
  k4a::image k4a_color_img = cap.get_color_image();
  k4a::image k4a_depth_img = cap.get_depth_image();
  log("INFO", "ready to save extra 12 ply");
  save_point_cloud(k4a_depth_img, k4a_color_img, "/home/slam/calibration/" + getImageName(12) + ".ply");
  log("INFO", "saved extra 12 ply");
  // end test
}

void DK_Camera::doOneStart(int station_num)
{
  take_count_ = 0;
  createDir(station_num, image_save_path_, dk_camera_);
  if (!camera_on_flag_)
  {
    dev_->start_cameras(&configuration_);
    if (camera_no_ == 1)
      dev_->start_imu();
    camera_on_flag_ = true;
    log("INFO", "device " + camera_name_ + " camera on flag: " + std::to_string(camera_on_flag_));
  }
  else
  {
    log("WARNING", "device already started");

    // TODO  for test
    k4a::capture cap;
    if (!dev_->get_capture(&cap, std::chrono::milliseconds(1000))) // wait 1000 ms
    {
      log("ERROR", camera_name_ + " no capture arrive in 3000 ms");
      return;
    }

    // Get image from capture
    k4a::image k4a_color_img = cap.get_color_image();
    k4a::image k4a_depth_img = cap.get_depth_image();
    // if(camera_no_==1)
    // {
    //     save_point_cloud(k4a_depth_img,k4a_color_img,"/home/slam/calibration/"+getImageName(12)+".ply");
    //     log("INFO","saved extra 12 ply");
    // }
    // end test
    return;
  }
}

void DK_Camera::doOneEnd()
{
  if (camera_on_flag_)
  {
    dev_->stop_cameras();
    if (this->camera_no_ == 1)
      dev_->stop_imu();
    camera_on_flag_ = false;
    log("INFO", "device " + camera_name_ + " camera on flag: " + std::to_string(camera_on_flag_));
  }
  index_++;
}

void DK_Camera::doOne()
{
  log("INFO", camera_name_ + "---" + getImageName(take_count_));
  k4a::capture cap;
  if (data_pub_flag_ && camera_no_ == 1)
  {
    std::ofstream os_pose(dk_camera_.imu_path + "/" + getImageName(index_) + "_" + std::to_string(camera_no_) + "_" + getImageName(take_count_) + ".txt", std::ios::app);
    auto t1 = std::chrono::system_clock::now();
    auto t2 = std::chrono::system_clock::now();

    while (std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() < 100) // 0.1 s
    {
      k4a_imu_sample_t imu_t;
      auto result = dev_->get_imu_sample(&imu_t);
      if (!result)
      {
        log("ERROR", camera_name_ + " imu has no data");
        return;
      }
      sensor_msgs::ImuPtr ros_imu_data(new sensor_msgs::Imu);
      std::string frame_id = getImageName(index_) + "_" + std::to_string(camera_no_) + "_" + getImageName(take_count_);
      getImuFrame(imu_t, ros_imu_data, frame_id);
      dk_imu_pub_.publish(ros_imu_data);
      std::string str_msg;
      str_msg =
          std::to_string(ros::Time::now().toSec()) + " " +
          std::to_string(imu_t.acc_sample.xyz.x) + " " +
          std::to_string(imu_t.acc_sample.xyz.y) + " " +
          std::to_string(imu_t.acc_sample.xyz.z) + " " +
          std::to_string(imu_t.gyro_sample.xyz.x) + " " +
          std::to_string(imu_t.gyro_sample.xyz.y) + " " +
          std::to_string(imu_t.gyro_sample.xyz.z) + " " + "\n";
      os_pose << str_msg;
      t2 = std::chrono::system_clock::now();
    }
    os_pose.close();
    sensor_msgs::ImuPtr ros_imu_data(new sensor_msgs::Imu);
    ros_imu_data->header.frame_id = "imu_ok";
    dk_imu_pub_.publish(ros_imu_data);
  }
  if (!dev_->get_capture(&cap, std::chrono::milliseconds(1000))) // wait 1000 ms
  {
    log("ERROR", camera_name_ + " no capture arrive in 3000 ms");
    return;
  }

  // Get image from capture
  k4a::image k4a_color_img = cap.get_color_image();
  k4a::image k4a_depth_img = cap.get_depth_image();
  if (camera_no_ == 2)
  {
    // save_point_cloud(k4a_depth_img,k4a_color_img,"/home/slam/calibration/"+getImageName(take_count_)+".ply");
  }

  auto color_width = k4a_color_img.get_width_pixels();
  auto color_height = k4a_color_img.get_height_pixels();

  k4a::transformation trans(dev_->get_calibration(depth_mode_, color_resolution_));

  k4a::image transformed_depth_image = k4a::image::create(k4a_depth_img.get_format(), color_width, color_height, color_width * (int)sizeof(uint16_t));
  trans.depth_image_to_color_camera(k4a_depth_img, &transformed_depth_image);

  // k4a::image xyz_img = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM, color_width, color_height,color_width * 3 * (int)sizeof(uint16_t));
  // trans.depth_image_to_point_cloud(transformed_depth_image,K4A_CALIBRATION_TYPE_COLOR,&xyz_img);

  // Turn to cv format
  cv::Mat cv_color_img = get_mat(k4a_color_img);
  cv::Mat cv_depth_img = get_mat(transformed_depth_image);

  if (bgra2bgr_flag_)
  {
    cv::cvtColor(cv_color_img, cv_color_img, cv::COLOR_BGRA2BGR);
  }

  if (save_img_flag_)
  { // 000_1_000.jpg
    cv::imwrite(dk_camera_.color_path + "/" + getImageName(index_) + "_" + std::to_string(camera_no_) + "_" + getImageName(take_count_) + ".jpg", cv_color_img);
    cv::imwrite(dk_camera_.depth_path + "/" + getImageName(index_) + "_" + std::to_string(camera_no_) + "_" + getImageName(take_count_) + ".tiff", cv_depth_img);
  }
  if (data_pub_flag_)
  {
    std::string frame_id = getImageName(index_) + "_" + std::to_string(camera_no_) + "_" + getImageName(take_count_);
    publishData(cv_color_img, cv_depth_img, frame_id);
  }
  take_count_++;
}

void DK_Camera::setBgra2bgrFlag(bool value)
{
  bgra2bgr_flag_ = value;
}

void DK_Camera::setSaveImgFlag(bool value)
{
  save_img_flag_ = value;
}

void DK_Camera::setImgSavePath(std::string &path)
{
  image_save_path_ = path;
}

void DK_Camera::createDir(const int &index, const std::string &dir_filepath, DKCamera &dk_camera)
{
  if ("" == dir_filepath)
  {
    log("ERROR", camera_name_ + " file path is valid");
    return;
  }
  // 创建工程目录
  if (access(dir_filepath.data(), 0))
  {
    mkdir(dir_filepath.data(), 0755);
  }

  // 创建一级目录
  std::string dir_filepath_image = dir_filepath + "/Image";
  std::string scan_current = dir_filepath_image + "/scan" + this->getImageName(index);
  dk_camera.color_path = scan_current + "/color";
  dk_camera.depth_path = scan_current + "/depth";
  dk_camera.imu_path = scan_current + "/imu";

  mkdir(dir_filepath_image.data(), 0755);
  if (NULL != opendir(scan_current.data()))
    return;
  mkdir(scan_current.data(), 0755);

  mkdir(dk_camera.color_path.data(), 0755);
  mkdir(dk_camera.depth_path.data(), 0755);
  mkdir(dk_camera.imu_path.data(), 0755);
}

std::shared_ptr<k4a::device> DK_Camera::GetDevice()
{
  return dev_;
}

void ColorizeGreyscale(cv::Mat &src)
{
  cv::Mat new_image = src.clone();

  uint16_t max = std::numeric_limits<uint16_t>::min();
  uint16_t min = std::numeric_limits<uint16_t>::max();

  for (int i = 0; i < new_image.rows; ++i)
  {
    for (int j = 0; j < new_image.cols; ++j)
    {
      uint16_t p = new_image.at<uint16_t>(i, j);
      max = std::max(max, p);
      min = std::min(min, p);
    }
  }

  float alpha = 0.95;
  for (int i = 0; i < new_image.rows; ++i)
  {
    for (int j = 0; j < new_image.cols; ++j)
    {
      float beta = (float(new_image.at<uint16_t>(i, j)) - min) / (max * alpha - min);

      beta = beta > 1 ? 1.0 : beta;

      new_image.at<uint16_t>(i, j) = beta * 65535.0;
    }
  }
  src = new_image;
}

void DK_Camera::calibration()
{
  // auto intrin = calibration_data_.color_camera_calibration.intrinsics.parameters;
  auto intrin = calibration_data_.depth_camera_calibration.intrinsics.parameters;
  auto color_infra_extrin = calibration_data_.extrinsics[0][1];
  std::cout << "color to depth extrin---" << camera_no_ << std::endl;
  std::cout << "rotation---" << std::endl;
  for (int i = 0; i < 9; i++)
  {
    std::cout << color_infra_extrin.rotation[i] << std::endl;
  }
  std::cout << "translation---" << std::endl;
  for (int i = 0; i < 3; i++)
  {
    std::cout << color_infra_extrin.translation[i] << std::endl;
  }
  std::cout << "intrinsics---" << camera_no_ << std::endl;
  for (int i = 0; i < 15; i++)
  {
    std::cout << intrin.v[i] << std::endl;
  }
  std::cout << "\n"
            << std::endl;
  k4a::capture cap;
  int count = 0;
  dev_->start_cameras(&configuration_);
  dev_->start_imu();

  int key = 0;
  while (key != 27)
  {
    auto result = dev_->get_capture(&cap, std::chrono::milliseconds(K4A_WAIT_INFINITE));
    if (!result)
    {
      return;
    }
    k4a::image color_image = cap.get_color_image();
    k4a::image depth_image = cap.get_depth_image();
    k4a::image infra_image = cap.get_ir_image();

    auto color_width = color_image.get_width_pixels();
    auto color_height = color_image.get_height_pixels();
    auto depth_width = depth_image.get_width_pixels();
    auto depth_height = depth_image.get_height_pixels();
    k4a::transformation trans(dev_->get_calibration(depth_mode_, color_resolution_));

    k4a::image xyz_imgae = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM, color_width, color_height, color_width * 3 * (int)sizeof(uint16_t));
    k4a::image transformed_depth_image = k4a::image::create(depth_image.get_format(), color_width, color_height, color_width * (int)sizeof(uint16_t));

    trans.depth_image_to_color_camera(depth_image, &transformed_depth_image);

    cv::Mat cv_depth_trans = get_mat(transformed_depth_image);

    cv::Mat color_mat = get_mat(color_image);
    cv::Mat infra_mat = get_mat(infra_image);

    auto infra_mat_8bit = infra_mat.clone();

    ColorizeGreyscale(infra_mat_8bit);

    infra_mat_8bit.convertTo(infra_mat_8bit, CV_8UC1, 1 / 256.0);
    cv::equalizeHist(infra_mat_8bit, infra_mat_8bit);
    // cv::Mat color_mat_3;

    //        //cv::namedWindow("color_calibration-"+std::to_string(camera_no_),CV_WINDOW_NORMAL);
    //        //cv::namedWindow("infra_calibration-"+std::to_string(camera_no_),CV_WINDOW_NORMAL);
    //        cv::imshow("color_calibration-"+std::to_string(camera_no_),color_mat);
    cv::imshow("infra_calibration-" + std::to_string(camera_no_), infra_mat_8bit);
    //        publishData(color_mat,cv_depth_trans,"calibration-"+std::to_string(camera_no_));

    //        k4a_imu_sample_t imu_t;
    //        dev_->get_imu_sample(&imu_t,std::chrono::milliseconds(3000));
    //        sensor_msgs::ImuPtr ros_imu_data(new sensor_msgs::Imu);
    //        getImuFrame(imu_t, ros_imu_data, std::__cxx11::string());
    //        dk_imu_pub_.publish(ros_imu_data);
    cv::namedWindow("color_calibration-" + std::to_string(camera_no_), 0);
    cv::resizeWindow("color_calibration-" + std::to_string(camera_no_), 640, 480);

    cv::imshow("color_calibration-" + std::to_string(camera_no_), color_mat);

    key = cv::waitKey(33);
    if (key == 32)
    {
      cv::imwrite("/home/slam/calibration/" + std::to_string(camera_no_) + "_color_" + std::to_string(count) + ".jpg", color_mat);
      // cv::imwrite("/home/slam/calibration/"+std::to_string(camera_no_)+"/infra_"+std::to_string(count)+".png",infra_mat_8bit);
      cv::imwrite("/home/slam/calibration/" + std::to_string(camera_no_) + "_depth_" + std::to_string(count) + ".tiff", cv_depth_trans);
      //            save_point_cloud_infra(depth_image, infra_image, "/home/slam/calibration/" +
      //                                                             std::to_string(camera_no_) + "_" + std::to_string(count) +
      //                                                             ".ply", infra_mat_8bit);
      save_point_cloud(depth_image, color_image, "/home/slam/calibration/" + std::to_string(camera_no_) + "_color_" + std::to_string(count) + ".ply");
      // save_point_cloud(xyz_imgae,color_image,"/home/slam/slam_data/"+std::to_string(camera_no_)+".ply");
      count++;
    }
  }
  // cv::cvtColor(color_mat_3,color_mat_3,cv::COLOR_BGRA2BGR);
  dev_->stop_cameras();
  dev_->stop_imu();
  dev_->close();
  cv::destroyAllWindows();
}

void DK_Camera::save_point_cloud_infra(k4a::image &depth_img, k4a::image &infra_img, std::string file_name,
                                       cv::Mat cv_infra)
{
  // set tansformation
  k4a::transformation trans(dev_->get_calibration(depth_mode_, color_resolution_));

  auto color_width = infra_img.get_width_pixels();
  auto color_height = infra_img.get_height_pixels();

  // create transformed depth image
  // k4a::image transformed_depth_image = k4a::image::create(depth_img.get_format(),color_width,color_height,color_width * (int)sizeof(uint16_t));
  // Transforms the depth map into the geometry of the color camera.
  // trans.depth_image_to_color_camera(depth_img,&transformed_depth_image);

  // transforms the depth into xyz imgae
  k4a::image xyz_img = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM, color_width, color_height, color_width * 3 * (int)sizeof(uint16_t));
  trans.depth_image_to_point_cloud(depth_img, K4A_CALIBRATION_TYPE_DEPTH, &xyz_img);

  // get points data
  std::vector<color_point_t> points;
  int16_t *point_cloud_image_data = (int16_t *)(void *)xyz_img.get_buffer();
  uint8_t *color_image_data = infra_img.get_buffer();

  for (int row = 0; row < color_height; ++row)
  {
    for (int col = 0; col < color_width; ++col)
    {
      int num = row * color_width + col;

      uint8_t c_val = cv_infra.at<uint8_t>(row, col);
      color_point_t point;
      point.xyz[0] = point_cloud_image_data[3 * num + 0];
      point.xyz[1] = point_cloud_image_data[3 * num + 1];
      point.xyz[2] = point_cloud_image_data[3 * num + 2];

      point.rgb[0] = c_val;
      point.rgb[1] = c_val;
      point.rgb[2] = c_val;

      if (point.rgb[0] == 0 && point.rgb[1] == 0 && point.rgb[2] == 0)
      {
        continue;
      }
      points.push_back(point);
    }
  }

#define PLY_START_HEADER "ply"
#define PLY_END_HEADER "end_header"
#define PLY_ASCII "format ascii 1.0"
#define PLY_ELEMENT_VERTEX "element vertex"

  // save to the ply file
  std::ofstream ofs(file_name); // text mode first
  ofs << PLY_START_HEADER << std::endl;
  ofs << PLY_ASCII << std::endl;
  ofs << PLY_ELEMENT_VERTEX << " " << points.size() << std::endl;
  ofs << "property float x" << std::endl;
  ofs << "property float y" << std::endl;
  ofs << "property float z" << std::endl;
  ofs << "property uchar red" << std::endl;
  ofs << "property uchar green" << std::endl;
  ofs << "property uchar blue" << std::endl;
  ofs << PLY_END_HEADER << std::endl;
  ofs.close();

  float scale = 1000.0;

  std::stringstream ss;
  for (size_t i = 0; i < points.size(); ++i)
  {
    // image data is BGR
    ss << (float)points[i].xyz[0] / scale << " " << (float)points[i].xyz[1] / scale << " " << (float)points[i].xyz[2] / scale;
    ss << " " << (float)points[i].rgb[2] << " " << (float)points[i].rgb[1] << " " << (float)points[i].rgb[0];
    ss << std::endl;
  }
  std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
  ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}

void DK_Camera::save_point_cloud(k4a::image &depth_img, k4a::image &color_img, std::string file_name)
{
  // set tansformation
  k4a::transformation trans(dev_->get_calibration(depth_mode_, color_resolution_));

  auto color_width = color_img.get_width_pixels();
  auto color_height = color_img.get_height_pixels();

  // create transformed depth image
  k4a::image transformed_depth_image = k4a::image::create(depth_img.get_format(), color_width, color_height, color_width * (int)sizeof(uint16_t));
  // Transforms the depth map into the geometry of the color camera.
  trans.depth_image_to_color_camera(depth_img, &transformed_depth_image);

  // transforms the depth into xyz imgae
  k4a::image xyz_img = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM, color_width, color_height, color_width * 3 * (int)sizeof(uint16_t));
  trans.depth_image_to_point_cloud(transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &xyz_img);

  // get points data
  std::vector<color_point_t> points;
  int16_t *point_cloud_image_data = (int16_t *)(void *)xyz_img.get_buffer();
  uint8_t *color_image_data = color_img.get_buffer();

  for (int i = 0; i < color_width * color_height; i++)
  {
    color_point_t point;
    point.xyz[0] = point_cloud_image_data[3 * i + 0];
    point.xyz[1] = point_cloud_image_data[3 * i + 1];
    point.xyz[2] = point_cloud_image_data[3 * i + 2];
    if (point.xyz[2] == 0)
    {
      continue;
    }
    point.rgb[0] = color_image_data[4 * i + 0];
    point.rgb[1] = color_image_data[4 * i + 1];
    point.rgb[2] = color_image_data[4 * i + 2];
    uint8_t alpha = color_image_data[4 * i + 3];

    if (point.rgb[0] == 0 && point.rgb[1] == 0 && point.rgb[2] == 0 && alpha == 0)
    {
      continue;
    }
    points.push_back(point);
  }

#define PLY_START_HEADER "ply"
#define PLY_END_HEADER "end_header"
#define PLY_ASCII "format ascii 1.0"
#define PLY_ELEMENT_VERTEX "element vertex"

  // save to the ply file
  std::ofstream ofs(file_name); // text mode first
  ofs << PLY_START_HEADER << std::endl;
  ofs << PLY_ASCII << std::endl;
  ofs << PLY_ELEMENT_VERTEX << " " << points.size() << std::endl;
  ofs << "property float x" << std::endl;
  ofs << "property float y" << std::endl;
  ofs << "property float z" << std::endl;
  ofs << "property uchar red" << std::endl;
  ofs << "property uchar green" << std::endl;
  ofs << "property uchar blue" << std::endl;
  ofs << PLY_END_HEADER << std::endl;
  ofs.close();

  float scale = 1000.0;

  std::stringstream ss;
  for (size_t i = 0; i < points.size(); ++i)
  {
    // image data is BGR
    ss << (float)points[i].xyz[0] / scale << " " << (float)points[i].xyz[1] / scale << " " << (float)points[i].xyz[2] / scale;
    ss << " " << (float)points[i].rgb[2] << " " << (float)points[i].rgb[1] << " " << (float)points[i].rgb[0];
    ss << std::endl;
  }
  std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
  ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}

void DK_Camera::SetDepthMode(k4a_depth_mode_t depth_mode)
{
  depth_mode_ = depth_mode;
  configuration_.depth_mode = depth_mode_;

  calibration_data_ = dev_->get_calibration(depth_mode_, color_resolution_);
}

void DK_Camera::SetColorResolution(k4a_color_resolution_t color_resolution)
{
  color_resolution_ = color_resolution;
  configuration_.color_resolution = color_resolution_;

  calibration_data_ = dev_->get_calibration(depth_mode_, color_resolution_);
}

void DK_Camera::WaitAutoExposure()
{
  auto t1 = std::chrono::system_clock::now();
  auto t2 = std::chrono::system_clock::now();

  while (std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() < 3)
  {
    k4a::capture cap;
    auto result = dev_->get_capture(&cap, std::chrono::milliseconds(K4A_WAIT_INFINITE));
    t2 = std::chrono::system_clock::now();
  }
}

void DK_Camera::setIndex(int index)
{
  index_ = index;
}

void DK_Camera::setDataPubFlag(bool value)
{
  data_pub_flag_ = value;
}

void DK_Camera::publishData(cv::Mat &color_img, cv::Mat &depth_img, std::string frame_id)
{
  sensor_msgs::ImagePtr ros_color_img = cv_bridge::CvImage(std_msgs::Header(), "bgra8", color_img).toImageMsg();
  ros_color_img->header.frame_id = frame_id;
  ros_color_img->header.stamp = ros::Time::now();
  sensor_msgs::ImagePtr ros_depth_img = cv_bridge::CvImage(std_msgs::Header(), "mono16", depth_img).toImageMsg();
  ros_depth_img->header.frame_id = frame_id;
  ros_depth_img->header.stamp = ros::Time::now();
  dk_color_image_pub_.publish(ros_color_img);
  dk_depth_image_pub_.publish(ros_depth_img);
  // ros::spinOnce();
}

void DK_Camera::publishColorImg(cv::Mat &color_img, std::string frame_id)
{
  sensor_msgs::ImagePtr ros_color_img = cv_bridge::CvImage(std_msgs::Header(), "rgb8", color_img).toImageMsg();
  ros_color_img->header.frame_id = frame_id;
  ros_color_img->header.stamp = ros::Time::now();
  dk_color_image_pub_.publish(ros_color_img);
  // ros::spinOnce();
}

void DK_Camera::publishDepthImg(cv::Mat &depth_img, std::string frame_id)
{
  sensor_msgs::ImagePtr ros_depth_img = cv_bridge::CvImage(std_msgs::Header(), "mono16", depth_img).toImageMsg();
  ros_depth_img->header.frame_id = frame_id;
  ros_depth_img->header.stamp = ros::Time::now();
  dk_depth_image_pub_.publish(ros_depth_img);
  // ros::spinOnce();
}

void DK_Camera::getImuFrame(const k4a_imu_sample_t &sample, sensor_msgs::ImuPtr &imu_msg, std::string frame_id)
{
  imu_msg->header.frame_id = frame_id;
  imu_msg->header.stamp = ros::Time::now(); // timestampToROS(std::chrono::microseconds(sample.acc_timestamp_usec));

  // The correct convention in ROS is to publish the raw sensor data, in the
  // sensor coordinate frame. Do that here.
  imu_msg->angular_velocity.x = sample.gyro_sample.xyz.x;
  imu_msg->angular_velocity.y = sample.gyro_sample.xyz.y;
  imu_msg->angular_velocity.z = sample.gyro_sample.xyz.z;

  imu_msg->linear_acceleration.x = sample.acc_sample.xyz.x;
  imu_msg->linear_acceleration.y = sample.acc_sample.xyz.y;
  imu_msg->linear_acceleration.z = sample.acc_sample.xyz.z;

  // Disable the orientation component of the IMU message since it's invalid
  imu_msg->orientation_covariance[0] = -1.0;
}

void DK_Camera::blockExposureAndWhiteBalance()
{
  dev_->set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_MANUAL, LEVEL_9);
  dev_->set_color_control(K4A_COLOR_CONTROL_WHITEBALANCE, K4A_COLOR_CONTROL_MODE_AUTO, 0);
  dev_->start_cameras(&configuration_);
  for (int i = 0; i < 60; i++)
  {
    k4a::capture cap;
    dev_->get_capture(&cap);
  }
  // Get a capture and check
  k4a::capture cap;
  auto result = dev_->get_capture(&cap, std::chrono::milliseconds(K4A_WAIT_INFINITE));

  int32_t powerline_freq, GET_WB2, GET_EX2;
  auto mode = K4A_COLOR_CONTROL_MODE_MANUAL;
  auto mode2 = K4A_COLOR_CONTROL_MODE_AUTO;
  dev_->get_color_control(K4A_COLOR_CONTROL_POWERLINE_FREQUENCY, &mode, &powerline_freq);
  dev_->get_color_control(K4A_COLOR_CONTROL_WHITEBALANCE, &mode, &GET_WB2);
  dev_->get_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, &mode, &GET_EX2);
  k4a::image k4a_color_img = cap.get_color_image();
  auto e_time = k4a_color_img.get_exposure();

  auto val = getLevelTime(e_time);
  std::cout << "Val: " << val << std::endl;
  auto wb = k4a_color_img.get_white_balance();
  dev_->stop_cameras();

  std::cout << "before: " << camera_no_ << "get power freq: " << powerline_freq << " : exposure time: " << e_time.count() << " " << GET_EX2
            << " white balance: " << wb << " " << GET_WB2 << std::endl;

  dev_->set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_MANUAL, LEVEL_7);
  dev_->set_color_control(K4A_COLOR_CONTROL_WHITEBALANCE, K4A_COLOR_CONTROL_MODE_MANUAL, wb);
  dev_->set_color_control(K4A_COLOR_CONTROL_POWERLINE_FREQUENCY, K4A_COLOR_CONTROL_MODE_MANUAL, 2);
  sleep(2);

  dev_->start_cameras(&configuration_);
  sleep(0.2);
  // Get a capture and check

  dev_->get_capture(&cap, std::chrono::milliseconds(K4A_WAIT_INFINITE));
  dev_->stop_cameras();
  k4a_color_img = cap.get_color_image();
  e_time = k4a_color_img.get_exposure();
  wb = k4a_color_img.get_white_balance();

  std::cout << "after: " << camera_no_ << "get power freq: " << powerline_freq << " : exposure time: " << e_time.count() << " white balance: " << wb << std::endl;

  // cv::imwrite("/home/slam/data_cap/hdr/"+std::to_string(exposure_time)+".jpg",cv_color_img);
}

std::tuple<std::chrono::microseconds, unsigned int> DK_Camera::getExposureAndWhiteBalance()
{
  //	dev_->stop_cameras();
  //	dev_->stop_imu();
  //    dev_->set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_AUTO, LEVEL_AUTO);
  //    dev_->set_color_control(K4A_COLOR_CONTROL_WHITEBALANCE, K4A_COLOR_CONTROL_MODE_AUTO, 0);

  //    for(int i=0;i<60;i++)
  //    {
  //        k4a::capture cap;
  //        dev_->get_capture(&cap);
  //    }
  k4a::capture cap;
  dev_->get_capture(&cap);
  k4a::image k4a_color_img = cap.get_color_image();
  auto exposure_time = k4a_color_img.get_exposure();
  auto white_balance_value = k4a_color_img.get_white_balance();
  // dev_->stop_cameras();

  return {exposure_time, white_balance_value};
}

int DK_Camera::getLevelTime(std::chrono::microseconds ms)
{
  int32_t time = ms.count();

  std::vector<int32_t> exposure_vec = {500,
                                       1250,
                                       2500,
                                       8330,
                                       16670,
                                       33330,
                                       41670,
                                       50000,
                                       66670,
                                       83330,
                                       100000,
                                       116670};

  std::vector<int32_t> exposure_level_vec = {488,
                                             977,
                                             1953,
                                             3906,
                                             7813,
                                             15625,
                                             31250,
                                             62500,
                                             125000,
                                             250000,
                                             500000,
                                             1000000};

  int index = -1;
  for (auto it : exposure_vec)
  {
    index++;
    if (time - it <= 0)
    {
      time = it;
      break;
    }
  }

  return exposure_level_vec[index];
}

void DK_Camera::setExposureAndWhiteBalance(std::chrono::microseconds exposure_time, unsigned int white_balance_value)
{
  dev_->stop_cameras();
  if (camera_no_ == 1)
    dev_->stop_imu();
  auto ex_time = getLevelTime(exposure_time);
  dev_->set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_MANUAL, ex_time);
  dev_->set_color_control(K4A_COLOR_CONTROL_WHITEBALANCE, K4A_COLOR_CONTROL_MODE_MANUAL, white_balance_value);
  dev_->start_cameras(&configuration_);
  if (camera_no_ == 1)
    dev_->start_imu();
}

int DK_Camera::getCameraNo()
{
  return camera_no_;
}

int DK_Camera::getTakeCount()
{
  return (take_count_ - 1) * 30;
}

int DK_Camera::getStationNumber()
{
  return index_;
}

bool DK_Camera::stop()
{
  stop_flag_ = true;
  dev_->stop_cameras();
  dev_->stop_imu();
  std_msgs::String str;
  // flag:savePath
  str.data = ns_runtime_flags::INFO_MESSAGE_TEXTVIEW + _saveFolder.data;
  dk_runtime_pub_.publish(str);
  ros::spinOnce();
  return true;
}

#define INVALID INT32_MIN

typedef struct _pinhole_t
{
  float px;
  float py;
  float fx;
  float fy;

  int width;
  int height;
} pinhole_t;

typedef struct _coordinate_t
{
  int x;
  int y;
  float weight[4];
} coordinate_t;

typedef enum
{
  INTERPOLATION_NEARESTNEIGHBOR, /**< Nearest neighbor interpolation */
  INTERPOLATION_BILINEAR,        /**< Bilinear interpolation */
  INTERPOLATION_BILINEAR_DEPTH   /**< Bilinear interpolation with invalidation when neighbor contain invalid
                                               data with value 0 */
} interpolation_t;

// Compute a conservative bounding box on the unit plane in which all the points have valid projections
static void compute_xy_range(const k4a_calibration_t *calibration,
                             const k4a_calibration_type_t camera,
                             const int width, const int height,
                             float &x_min, float &x_max,
                             float &y_min, float &y_max)
{
  // Step outward from the centre point until we find the bounds of valid projection
  const float step_u = 0.25f;
  const float step_v = 0.25f;
  const float min_u = 0;
  const float min_v = 0;
  const float max_u = (float)width - 1;
  const float max_v = (float)height - 1;
  const float center_u = 0.5f * width;
  const float center_v = 0.5f * height;

  int valid;
  k4a_float2_t p;
  k4a_float3_t ray;

  // search x_min
  for (float uv[2] = {center_u, center_v}; uv[0] >= min_u; uv[0] -= step_u)
  {
    p.xy.x = uv[0];
    p.xy.y = uv[1];
    k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

    if (!valid)
    {
      break;
    }
    x_min = ray.xyz.x;
  }

  // search x_max
  for (float uv[2] = {center_u, center_v}; uv[0] <= max_u; uv[0] += step_u)
  {
    p.xy.x = uv[0];
    p.xy.y = uv[1];
    k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

    if (!valid)
    {
      break;
    }
    x_max = ray.xyz.x;
  }

  // search y_min
  for (float uv[2] = {center_u, center_v}; uv[1] >= min_v; uv[1] -= step_v)
  {
    p.xy.x = uv[0];
    p.xy.y = uv[1];
    k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

    if (!valid)
    {
      break;
    }
    y_min = ray.xyz.y;
  }

  // search y_max
  for (float uv[2] = {center_u, center_v}; uv[1] <= max_v; uv[1] += step_v)
  {
    p.xy.x = uv[0];
    p.xy.y = uv[1];
    k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

    if (!valid)
    {
      break;
    }
    y_max = ray.xyz.y;
  }
}

static pinhole_t create_pinhole_from_xy_range(const k4a_calibration_t *calibration, const k4a_calibration_type_t camera)
{
  int width = calibration->depth_camera_calibration.resolution_width;
  int height = calibration->depth_camera_calibration.resolution_height;
  if (camera == K4A_CALIBRATION_TYPE_COLOR)
  {
    width = calibration->color_camera_calibration.resolution_width;
    height = calibration->color_camera_calibration.resolution_height;
  }

  float x_min = 0, x_max = 0, y_min = 0, y_max = 0;
  compute_xy_range(calibration, camera, width, height, x_min, x_max, y_min, y_max);

  pinhole_t pinhole;

  float fx = 1.f / (x_max - x_min);
  float fy = 1.f / (y_max - y_min);
  float px = -x_min * fx;
  float py = -y_min * fy;

  pinhole.fx = fx * width;
  pinhole.fy = fy * height;
  pinhole.px = px * width;
  pinhole.py = py * height;
  pinhole.width = width;
  pinhole.height = height;

  return pinhole;
}

static void create_undistortion_lut(const k4a_calibration_t *calibration,
                                    const k4a_calibration_type_t camera,
                                    const pinhole_t *pinhole, k4a_image_t lut,
                                    interpolation_t type)
{
  coordinate_t *lut_data = (coordinate_t *)(void *)k4a_image_get_buffer(lut);

  k4a_float3_t ray;
  ray.xyz.z = 1.f;

  int src_width = calibration->depth_camera_calibration.resolution_width;
  int src_height = calibration->depth_camera_calibration.resolution_height;
  if (camera == K4A_CALIBRATION_TYPE_COLOR)
  {
    src_width = calibration->color_camera_calibration.resolution_width;
    src_height = calibration->color_camera_calibration.resolution_height;
  }

  for (int y = 0, idx = 0; y < pinhole->height; y++)
  {
    ray.xyz.y = ((float)y - pinhole->py) / pinhole->fy;

    for (int x = 0; x < pinhole->width; x++, idx++)
    {
      ray.xyz.x = ((float)x - pinhole->px) / pinhole->fx;

      k4a_float2_t distorted;
      int valid;
      k4a_calibration_3d_to_2d(calibration, &ray, camera, camera, &distorted, &valid);

      coordinate_t src;
      if (type == INTERPOLATION_NEARESTNEIGHBOR)
      {
        // Remapping via nearest neighbor interpolation
        src.x = (int)floorf(distorted.xy.x + 0.5f);
        src.y = (int)floorf(distorted.xy.y + 0.5f);
      }
      else if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
      {
        // Remapping via bilinear interpolation
        src.x = (int)floorf(distorted.xy.x);
        src.y = (int)floorf(distorted.xy.y);
      }
      else
      {
        printf("Unexpected interpolation type!\n");
        exit(-1);
      }

      if (valid && src.x >= 0 && src.x < src_width && src.y >= 0 && src.y < src_height)
      {
        lut_data[idx] = src;

        if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
        {
          // Compute the floating point weights, using the distance from projected point src to the
          // image coordinate of the upper left neighbor
          float w_x = distorted.xy.x - src.x;
          float w_y = distorted.xy.y - src.y;
          float w0 = (1.f - w_x) * (1.f - w_y);
          float w1 = w_x * (1.f - w_y);
          float w2 = (1.f - w_x) * w_y;
          float w3 = w_x * w_y;

          // Fill into lut
          lut_data[idx].weight[0] = w0;
          lut_data[idx].weight[1] = w1;
          lut_data[idx].weight[2] = w2;
          lut_data[idx].weight[3] = w3;
        }
      }
      else
      {
        lut_data[idx].x = INVALID;
        lut_data[idx].y = INVALID;
      }
    }
  }
}

bool DK_Camera::start()
{
  // set save path [abstruct path + DataTime]
  stop_flag_ = false;
  std::string save_path = "/home/slam/slam_data/";
  save_path += getTimeNow();
  _saveFolder.data = save_path;

  // color, depth, imu, info[calibration]
  auto color_save_path = save_path + "/color/";
  auto depth_save_path = save_path + "/depth/";
  auto imu_save_path = save_path + "/imu/";
  auto info_path = save_path + "/info/";

  mkdir(save_path.data(), 0755);
  mkdir(color_save_path.data(), 0755);
  mkdir(depth_save_path.data(), 0755);
  mkdir(imu_save_path.data(), 0755);
  mkdir(info_path.data(), 0755);

  dev_->start_cameras(&configuration_);
  dev_->start_imu();

  // get calibration params
  k4a_calibration_t _calibration = dev_->get_calibration(configuration_.depth_mode, configuration_.color_resolution);
  k4a::transformation trans(_calibration);

  // write calibration params to file
  std::ofstream calFile(info_path + "calibration.txt");
  auto &param = _calibration.color_camera_calibration.intrinsics.parameters.param;
  calFile << std::fixed << std::setprecision(6);
  calFile << "cx: " << param.cx << std::endl;
  calFile << "cy: " << param.cy << std::endl;
  calFile << "fx: " << param.fx << std::endl;
  calFile << "fy: " << param.fy << std::endl;
  calFile << "k1: " << param.k1 << std::endl;
  calFile << "k2: " << param.k2 << std::endl;
  calFile << "k3: " << param.k3 << std::endl;
  calFile << "p1: " << param.p1 << std::endl;
  calFile << "p2: " << param.p2 << std::endl;

  k4a_image_t lut = NULL;
  pinhole_t pinhole = create_pinhole_from_xy_range(&_calibration, K4A_CALIBRATION_TYPE_COLOR);

  // Generate a pinhole model for depth camera
  int writebytes = _calibration.color_camera_calibration.resolution_width *
                   _calibration.color_camera_calibration.resolution_height *
                   (int)sizeof(coordinate_t);

  // set image creator
  k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                   pinhole.width,
                   pinhole.height,
                   pinhole.width * (int)sizeof(coordinate_t),
                   &lut);

  // set undistortion
  create_undistortion_lut(&_calibration, K4A_CALIBRATION_TYPE_COLOR,
                          &pinhole, lut, INTERPOLATION_BILINEAR_DEPTH);

  // write lut
  std::ofstream ofile(info_path + "lut.bin", std::ios::out | std::ios::binary);
  ofile << "#width height cx cy fx fy\n";
  ofile << _calibration.color_camera_calibration.resolution_width << " "
        << _calibration.color_camera_calibration.resolution_height << " "
        << pinhole.px << " " << pinhole.py << " "
        << pinhole.fx << " " << pinhole.fy << " " << std::endl;
  ofile.write((const char *)(void *)lut, writebytes);
  std::cout << ofile.badbit << std::endl;
  ofile.close();

  // create rgb and depth data variable
  k4a::capture cap_t;
  // create imu data variable
  k4a_imu_sample_t imu_t;

  int index = 0;

  // the frequence
  ros::Rate loop(30);
  switch (configuration_.camera_fps)
  {
  case k4a_fps_t::K4A_FRAMES_PER_SECOND_5:
    loop = ros::Rate(5);
    break;
  case k4a_fps_t::K4A_FRAMES_PER_SECOND_15:
    loop = ros::Rate(15);
    break;
  case k4a_fps_t::K4A_FRAMES_PER_SECOND_30:
    loop = ros::Rate(30);
    break;
  default:
    loop = ros::Rate(30);
    break;
  }

  // ofs for imu data
  std::fstream imu_txt(imu_save_path + "/" + std::to_string(index) + ".txt", std::ios::app);
  // ofs for associate
  std::ofstream associateFile(info_path + "associate.txt");
  associateFile << std::fixed << std::setprecision(6);

  while (ros::ok())
  {
    ros::spinOnce();
    if (stop_flag_)
      return false;

    // get data captures
    dev_->get_capture(&cap_t, std::chrono::milliseconds(1000));
    dev_->get_imu_sample(&imu_t, std::chrono::milliseconds(1000));

    // get data from captures
    k4a::image color_img = cap_t.get_color_image();
    k4a::image depth_img = cap_t.get_depth_image();

    // record imu data
    std::stringstream imu_ss;
    imu_ss << imu_t.acc_sample.xyz.x << ' ' << imu_t.acc_sample.xyz.y << ' ' << imu_t.acc_sample.xyz.z << ' '
           << imu_t.gyro_sample.xyz.x << ' ' << imu_t.gyro_sample.xyz.y << ' ' << imu_t.gyro_sample.xyz.z << '\n';

    if (!color_img || !depth_img)
    {
      continue;
    }

    // get rgb image width and height
    auto color_width = color_img.get_width_pixels();
    auto color_height = color_img.get_height_pixels();

    if (index == 0)
    {
      // record
      calFile << "width: " << color_width << std::endl;
      calFile << "height: " << color_height << std::endl;
    }

    // create a depth image similar to the rgb image
    k4a::image transformed_depth_image =
        k4a::image::create(depth_img.get_format(),
                           color_width, color_height,
                           color_width * (int)sizeof(uint16_t));

    // Transforms the depth map into the geometry of the color camera
    trans.depth_image_to_color_camera(depth_img, &transformed_depth_image);

    // do with transformed_depth_image and color_img
    cv::Mat cv_color_img = get_mat(color_img);
    cv::Mat cv_depth_img = get_mat(transformed_depth_image);

    // cast
    cv::cvtColor(cv_color_img, cv_color_img, cv::COLOR_BGRA2BGR);

    // save data

    // imu
    imu_txt << imu_ss.str();
    // rgb
    cv::imwrite(color_save_path + getImageName(index) + ".jpg", cv_color_img);
    // depth
    cv::imwrite(depth_save_path + getImageName(index) + ".png", cv_depth_img);
    // associate
    auto stamp = ros::Time::now().toSec();
    associateFile << stamp << ' ' << "color/" << getImageName(index) + ".jpg" << ' '
                  << stamp << ' ' << "depth/" << getImageName(index) + ".png" << std::endl;

    // publish rgb image
    if (index % 3 == 0)
    {
      cv::resize(cv_color_img, cv_color_img, cv::Size(cv_color_img.cols * 0.3, cv_color_img.rows * 0.3));
      this->publishColorImg(cv_color_img, getImageName(index));
    }

    // publish imu data
    std_msgs::String imuStr;
    imuStr.data = ns_runtime_flags::IMU_MESSAGE + imu_ss.str();
    this->dk_runtime_pub_.publish(imuStr);
    // for send runtime message -----------------------
    std_msgs::String curColorImgName,
        curDepthImgName, curTimeStamp, runTimeMsg;
    curColorImgName.data = "color/" + getImageName(index) + ".jpg";
    curDepthImgName.data = "depth/" + getImageName(index) + ".jpg";
    std::stringstream stream;
    stream << std::setiosflags(std::ios::fixed) << std::setprecision(3) << stamp;
    curTimeStamp.data = stream.str();
    // flag:color:depth:time
    runTimeMsg.data = ns_runtime_flags::COLOR_DEPTH_TIME +
                      curColorImgName.data + ":" +
                      curDepthImgName.data + ":" +
                      curTimeStamp.data;
    ROS_INFO("'dk_camera': '%s'", runTimeMsg.data.c_str());
    this->dk_runtime_pub_.publish(runTimeMsg);
    // ------------------------------------------------

    loop.sleep();
    index++;
  }

  imu_txt.close();
  calFile.close();
  associateFile.close();

  return false;
}

std::string DK_Camera::getTimeNow()
{
  time_t tt = time(NULL);
  tm *t = localtime(&tt);
  std::stringstream ss;
  ss << t->tm_year + 1900 << '-' << t->tm_mon + 1
     << '-' << t->tm_mday << '-' << t->tm_hour << '-' << t->tm_min << '-' << t->tm_sec;
  return ss.str();
}
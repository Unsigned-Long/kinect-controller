//
// Created by slam on 2020/6/29.
//

#ifndef DK_CAMERA_DK_CAMERA_H
#define DK_CAMERA_DK_CAMERA_H

#include "camera_config.h"
#include <fstream>
#include <iostream>
#include <k4a/k4a.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#include <dirent.h>
#include <sys/types.h>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>

struct color_point_t
{
  int16_t xyz[3];
  uint8_t rgb[3];
};

namespace ns_runtime_flags
{
  // format: flag[message...]
  static const std::string COLOR_DEPTH_TIME = "#0";
  static const std::string INFO_MESSAGE_TEXTVIEW = "#1";
  static const std::string INFO_MESSAGE_TOAST = "#2";
  static const std::string IMU_MESSAGE = "#3";
} // namespace ns_runtime_flags

class DK_Camera
{
public:
  DK_Camera(std::shared_ptr<k4a::device> device);
  // DK_Camera(uint32_t camera_index);
  ~DK_Camera();

  // for SLAM 0929
  bool start();
  bool stop();
  std::string getTimeNow();

  void doOneStart(int station_num);
  void doOneEnd();
  // DO: 实现相机启动，成功返回0，失败返回相机编号
  void openCamera();

  void publishData(cv::Mat &color_img, cv::Mat &depth_img, std::string frame_id);
  void publishColorImg(cv::Mat &color_img, std::string frame_id);
  void publishDepthImg(cv::Mat &depth_img, std::string frame_id);

  void getImage();
  void calibration();
  std::shared_ptr<k4a::device> GetDevice();

  void WaitAutoExposure();
  void blockExposureAndWhiteBalance();
  void setExposureAndWhiteBalance(std::chrono::microseconds exposure_time, unsigned int white_balance_value);
  std::tuple<std::chrono::microseconds, unsigned int> getExposureAndWhiteBalance();

  void closeDevice();

  // Do: Do once get color imgae and depth image ,if save_img_flag_ is true ,save image
  void doOne();
  // Do: Get a cv image with specify exposure time
  cv::Mat getExposureImage(EXPOSURE_LEVEL exposure_time);

  // Test
  void GetOneForTest();

  /**************set*************/
  void setHdrFlag(bool value);
  void setBgra2bgrFlag(bool value);
  void setSaveImgFlag(bool value);
  void setImgSavePath(std::string &path);
  void setIndex(int index);
  void setDataPubFlag(bool value);

  void SetDepthMode(k4a_depth_mode_t depth_mode);
  void SetColorResolution(k4a_color_resolution_t color_resolution);
  int getCameraNo();
  int getTakeCount();
  int getStationNumber();

  bool stop_flag_;

private:
  // for SLAM 2020 0929

  ros::NodeHandle privateNh_;
  // Do: publish image data
  ros::Publisher dk_color_image_pub_;
  ros::Publisher dk_depth_image_pub_;
  // Do: publish imu data
  ros::Publisher dk_imu_pub_;

  // Do: publish this node runtime message
  ros::Publisher dk_runtime_pub_;

  std::shared_ptr<k4a::device> dev_;

  // What: path for image to save
  std::string image_save_path_;
  k4a_device_configuration_t configuration_;
  k4a::calibration calibration_data_;
  CameraId camera_id_;
  // What: camera_no for create direct
  int camera_no_;
  std::string camera_name_;

  // 当前Scan索引号
  int index_;
  //当前获取照片的编号
  int take_count_;

  DKCamera dk_camera_;

  k4a_depth_mode_t depth_mode_;
  k4a_color_resolution_t color_resolution_;

  /**************flag***************/
  // 相机正在工作指示
  bool camera_on_flag_;
  // 保存HDR 数据 ，耗时慎启用
  bool hdr_flag_;

  bool bgra2bgr_flag_;
  // 保存数据到磁盘
  bool save_img_flag_;
  // 保存点云
  bool save_pointcloud_flag_;
  // 发布数据到topic
  bool data_pub_flag_;

  std_msgs::String _saveFolder;

  // Do: turn k4a::image to cv::Mat
  cv::Mat get_mat(k4a::image &src, bool deep_copy = true);
  // Do: sleep seconds. reload sleep() from <unistd.h>
  void sleep(float seconds);
  // Do: Get HDR image
  void mertens(std::vector<cv::Mat> &exposure_images, const std::string &image_save);
  // Do: Get image name like 001.jpg,002.jpg,010.jpg
  std::string getImageName(const size_t image_index);
  // Do: create direct for project like scan000/dk0/color
  void createDir(const int &index, const std::string &dir_filepath, DKCamera &dk_camera);

  void save_point_cloud(k4a::image &depth_img, k4a::image &color_img, std::string file_name);
  void save_point_cloud_infra(k4a::image &depth_img, k4a::image &infra_img, std::string file_name,
                              cv::Mat cv_infra);

  void getImuFrame(const k4a_imu_sample_t &sample, sensor_msgs::ImuPtr &imu_msg, std::string frame_id);

  int getLevelTime(std::chrono::microseconds ms);
};

#endif // DK_CAMERA_DK_CAMERA_H


#include "include/DK_Camera.h"
#include <iostream>
#include <k4a/k4a.hpp>
#include <memory>
#include <string>
#include <thread>
#include <time.h>

std::shared_ptr<DK_Camera> camera = nullptr;
ros::Publisher runtime_pub;

#include <unistd.h>

bool log(std::string user, std::string level, std::string msgs)
{
  if (vfork() == 0)
  {
    if (execl("/home/slam/slam_log/RecordLog.sh", "RecordLog.sh", user.data(), level.data(), msgs.data(), nullptr) == -1)
      return false;
    return true;
  }
  return false;
}

void camera_control_callback(const std_msgs::String::ConstPtr &msg);

int main(int argc, char **argv)
{

  ros::init(argc, argv, "DK_Camera");
  ros::NodeHandle nh;
  ros::Subscriber start_sub = nh.subscribe("camera_control", 10, &camera_control_callback);
  runtime_pub = nh.advertise<std_msgs::String>("runtime_state", 10);

  // Check whether the equipment is connected
  ros::Rate rate(1);
  while (ros::ok())
  {
    ros::spinOnce();
    if (k4a::device::get_installed_count() == 0)
    {
      // no vaild device
      ROS_WARN("'%s'", "no vaild device, please cennect to vaild device");
      std_msgs::String str;
      str.data = ns_runtime_flags::INFO_MESSAGE_TEXTVIEW + "No Vaild Data Gatherer";
      runtime_pub.publish(str);
      rate.sleep();
    }
    else
    {
      break;
    }
  }

  ros::Rate loop(30);
  while (ros::ok())
  {
    if (camera == nullptr)
    {

      // connect to vaild device
      ROS_INFO("'dk_camera': '%s'", "vaild device connected");
      std_msgs::String str;
      str.data = ns_runtime_flags::INFO_MESSAGE_TEXTVIEW + "Vaild Data Gatherer Connected";
      runtime_pub.publish(str);

      auto dev = std::make_shared<k4a::device>(k4a::device::open(0));
      while (ros::ok())
      {
        try
        {
          dev->get_serialnum();
          break;
        }
        catch (k4a::error &err)
        {
          std_msgs::String str;
          str.data = ns_runtime_flags::INFO_MESSAGE_TEXTVIEW + err.what();
          runtime_pub.publish(str);
          ROS_ERROR("'%s'", err.what());
          ros::spinOnce();
          loop.sleep();
        }
      }

      // create the camera
      camera = std::make_shared<DK_Camera>(dev);

      ROS_INFO("'dk_camera': '%s'", "dk camera created!");
    }

    ros::spinOnce();
    loop.sleep();
  }

  return 0;
}

void camera_control_callback(const std_msgs::String::ConstPtr &msg)
{
  ROS_INFO("'dk_camera' gets message from 'server': '%s'", msg->data.c_str());

  auto data = msg->data;

  std_msgs::String str;

  if (camera == nullptr)
  {
    if (data == "start")
    {
      str.data = ns_runtime_flags::INFO_MESSAGE_TOAST + "please connect to data gatherer";
    }
    else if (data == "stop")
    {
      str.data = ns_runtime_flags::INFO_MESSAGE_TOAST + "data gatherer is not working";
    }
    runtime_pub.publish(str);
    return;
  }

  // if device is stopped and message is 'start'
  if (data == "start")
  {
    if (camera->stop_flag_ == false)
    {
      str.data = ns_runtime_flags::INFO_MESSAGE_TOAST + "data gatherer is working";
      runtime_pub.publish(str);
    }
    else
    {
      str.data = ns_runtime_flags::INFO_MESSAGE_TOAST + "data gatherer is started";
      runtime_pub.publish(str);
      ROS_INFO("camera start");
      camera->start();
    }
  }
  // if device is started and message is 'stop'
  else if (data == "stop")
  {
    if (camera->stop_flag_ == true)
    {
      str.data =ns_runtime_flags::INFO_MESSAGE_TOAST + "data gatherer is not working";
      runtime_pub.publish(str);
    }
    else
    {
      str.data = ns_runtime_flags::INFO_MESSAGE_TOAST + "data gatherer is stopped";
      runtime_pub.publish(str);
      ROS_INFO("camera stop");
      camera->stop();
    }
  }
}
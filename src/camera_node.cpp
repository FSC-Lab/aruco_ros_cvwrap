#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <iostream>
#include <sstream>

using namespace std::string_literals;

int main(int argc, char* argv[]) {
  using namespace std::string_literals;

  ros::init(argc, argv, "camera_node");
  auto nh = ros::NodeHandle("~");
  auto it = image_transport::ImageTransport(nh);

  auto camera_name = ""s;
  if (!nh.getParam("camera_name", camera_name)) {
    ROS_ERROR("error: the following param is required: camera_name");
    return 1;
  }

  auto camera_info_url = ""s;
  if (!nh.getParam("camera_info_url", camera_info_url)) {
    ROS_ERROR("error: the following param is required: camera_info_url");
    return 1;
  }

  camera_info_manager::CameraInfoManager cim(nh, camera_name, camera_info_url);

  if (!cim.validateURL(camera_info_url)) {
    ROS_ERROR("Invalid camera info url. Got %s", camera_info_url.c_str());
    return 1;
  }

  if (!cim.loadCameraInfo(camera_info_url)) {
    ROS_INFO("Failed to load info for camera [%s] from url: %s", camera_name.c_str(), camera_info_url.c_str());
    return 1;
  }

  ROS_INFO("Loaded info for camera [%s] from url: %s", camera_name.c_str(), camera_info_url.c_str());
  auto camera_info = cim.getCameraInfo();

  auto filename = ""s;
  auto index = -1;
  auto cap = cv::VideoCapture();

  // If "index" is provided, then the VideoCapture object will be opened by camera index, and "filename" will NOT be
  // considered
  if (!nh.getParam("index", index)) {
    if (!nh.getParam("filename", filename)) {
      ROS_ERROR("error: either device index or video filename is required for video capture");
      return 1;
    }

    auto is_open = false;
    if (filename.find("appsink"s) != std::string::npos) {
      // A valid gstreamer pipeline must end in "appsink", so if this substring is found, open in GStreamer mode
      is_open = cap.open(filename, cv::CAP_GSTREAMER);
    } else {
      // Else, open a generic video object e.g. files
      is_open = cap.open(filename);
    }

    if (!is_open) {
      ROS_ERROR("Failed to open video file or capturing device or IP video stream: %s", filename.c_str());
      return 1;
    }

    ROS_INFO("Opened videocapture with: %s", filename.c_str());

  } else {
    if (!cap.open(index)) {
      ROS_ERROR("Failed to open camera: %d", index);
      return 1;
    }

    ROS_INFO("Opened videocapture at camera %d", index);
  }

  // The framerate can also be set by a parameter
  auto framerate = nh.param("framerate", 30);
  auto image_topic = nh.param("image_topic", "image"s);
  auto camera_topic = "/"s + camera_name + "/" + image_topic;
  auto camera_info_topic = "/"s + camera_name + "/camera_info";

  // Setup publishers for the image and camera info pair
  auto img_pub = it.advertise(camera_topic, 10);
  auto info_pub = nh.advertise<sensor_msgs::CameraInfo>(camera_info_topic, 10);

  auto rate = ros::Rate(framerate);
  cv::Mat frame;
  while (ros::ok()) {
    cap >> frame;
    if (frame.empty()) {
      ROS_WARN("Got empty frame!");
      continue;
    }

    auto msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    camera_info.header.stamp = msg->header.stamp = ros::Time::now();
    img_pub.publish(msg);
    info_pub.publish(camera_info);

    ros::spinOnce();
  }
  return 1;
}
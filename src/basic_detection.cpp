#include <aruco_ros_cvwrap/ArucoTag.h>
#include <aruco_ros_cvwrap/ArucoTagDetections.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <sophus/so3.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <iostream>
#include <unordered_map>

#include "TargetTrackerKF.hpp"
#include "utils.hpp"

using namespace std::string_literals;

int main(int argc, char* argv[]) {
  using namespace std::string_literals;

  ros::init(argc, argv, "basic_detection");
  auto nh = ros::NodeHandle("~");
  auto it = image_transport::ImageTransport(nh);

  auto params = utils::SetDetectorParamsFromRosParams(nh);
  auto aruco_dict = utils::SetArucoDictFromRosParams(nh);
  auto tag_length = nh.param("len", 0.1);

  auto camera_name = ""s;
  if (!nh.getParam("camera_name", camera_name)) {
    ROS_ERROR("error: the following param is required: camera_name");
    return 1;
  }

  // Setup advertisers for both full Pose measurement and Euler Angle objects
  // auto pose_pub = nh.advertise<geometry_msgs::PoseArray>("/aruco/tag_detections", 10);
  auto detections_pub = nh.advertise<aruco_ros_cvwrap::ArucoTagDetections>("/aruco/tag_detections", 10);

  // This is the Pose cache to be repeatedly overwritten for each new tag pose estimate in the inner loop
  auto detection = aruco_ros_cvwrap::ArucoTag();
  // auto tag_rpy = aruco_ros_cvwrap::EulerAngles();
  // Exploit contiguous layout of geometry_msgs::Vector3 and geometry_msgs::Quaternion's [x, y, z, (w)] data members
  // Setup Eigen::Map objects that span over data members in the Pose object, such that these data can be manipulated as
  // if they are Eigen::Vector3d and Eigen::Quaterniond respectively
  auto tag_translation = Eigen::Map<Eigen::Vector3d>(&detection.pose.pose.position.x);
  auto tag_rotation = Eigen::Map<Sophus::SO3d>(&detection.pose.pose.orientation.x);
  auto detection_covariance = Eigen::Map<Eigen::Matrix<double, 6, 6>>(detection.pose.covariance.data());
  ROS_INFO("Attempting to subscribe to /%s/image and /%s/camera_info", camera_name.c_str(), camera_name.c_str());

  auto ekf = std::unordered_map<int, TargetTrackerKF>();

  auto q_var =
      nh.param("process_variance", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

  if (q_var.size() != 12) {
    ROS_ERROR("Expects 12 elements in process variance vector. Got %zu elements", q_var.size());
    return 1;
  }

  auto Q_var = Eigen::Map<Eigen::Matrix<double, 12, 1>>(q_var.data()).asDiagonal();

  auto r_var = nh.param("observation_variance", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

  if (r_var.size() != 6) {
    ROS_ERROR("Expects 6 elements in observation variance vector. Got %zu elements", r_var.size());
    return 1;
  }
  auto R_var = Eigen::Map<Eigen::Matrix<double, 6, 1>>(r_var.data()).asDiagonal();
  auto last_measurement = ros::Time::now();
  auto image_sub = it.subscribeCamera(
      "/"s + camera_name + "/image"s, 10,
      [&](const sensor_msgs::ImageConstPtr& img_ptr, const sensor_msgs::CameraInfoConstPtr& info_ptr) {
        // Move the data that the info_ptr points to, also makes the data nonconst
        auto info = std::move(*info_ptr);

        // Construct Mat object to span over the buffer in the camera_info.K
        // camera_info.K is boost::array<double, 9>, so MUST use CV_64F. Using CV_32F will trash the data
        auto camera_matrix = cv::Mat(3, 3, CV_64F, info.K.data());

        // Construct Mat object to span over the buffer in the camera_info.D
        // Not strictly necessary since camera_info.D is std::vector<double> that meets the cv::InputArray requirement,
        // but buffer span construction is cheap and cv::Mat can be printed out easily
        auto distortion_coeffs = cv::Mat(info.D.size(), 1, CV_64F, info.D.data());

        auto cv_img_ptr = cv_bridge::toCvShare(img_ptr, "bgr8");

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(cv_img_ptr->image, aruco_dict, corners, ids, params);

        auto tag_poses = aruco_ros_cvwrap::ArucoTagDetections();
        // auto tag_angles = aruco_ros_cvwrap::EulerAnglesArray();

        if (ids.size() > 0) {
          std::vector<cv::Vec3d> rvecs, tvecs;

          cv::aruco::estimatePoseSingleMarkers(corners, tag_length, camera_matrix, distortion_coeffs, rvecs, tvecs);
          last_measurement = tag_poses.header.stamp = ros::Time::now();
          // aruco_ros_cvwrap::EulerAngles ang;
          for (auto i = 0U; i < ids.size(); ++i) {
            // Construct an Eigen::Map spanning over elements of the cv::Vec for translation
            auto translation = Eigen::Map<Eigen::Vector3d>(tvecs[i].val);

            // Construct an Eigen::Map spanning over elements of the cv::Vec for rotation
            auto rotation = Eigen::Map<Eigen::Vector3d>(rvecs[i].val);

            auto C = Sophus::SO3d::exp(-rotation);
            // Construct the SE3 object representing the pose. The rationale is:
            // Measurements:
            // r^{pc}_c: position of target w.r.t. camera in frame of camera
            // C_{pc}: translation of target w.r.t. camera
            // SE3 object:
            // T_p_c = [C_{pc}, r^{cp}_p]
            //         [  0   , 1       ]
            // Thus:
            // r^{cp}_p = -C_{pc} * r^{pc}_c
            auto T = Sophus::SE3d(C, -(C * translation));
            if (ekf.count(ids[i]) == 0) {
              // The pose estimation for this tag hasn't been seen before. Construct the pose estimator
              ekf.insert({ids[i], TargetTrackerKF(T, Q_var, R_var)});
            } else {
              detection.id = ids[i];
              auto& filt = ekf[ids[i]];

              // Run the observation model
              filt.observationModel(T);

              tag_rotation = filt.pose().so3();
              tag_translation = filt.pose().translation();
              tag_translation = -(tag_rotation.inverse() * tag_translation);
              detection_covariance = filt.covariance().topLeftCorner<6, 6>();
              // tag_angle = tag_rotation.unit_quaternion().toRotationMatrix().eulerAngles(2, 1, 0).reverse();
              // tag_angles.euler_angles.push_back(tag_rpy);
              tag_poses.detections.push_back(detection);
            }
          }
          // pose_pub.publish(tag_poses);
          detections_pub.publish(tag_poses);
        }
      });

  auto rate = ros::Rate(10);
  auto last_loop = ros::Time::now();

  while (ros::ok()) {
    auto now = ros::Time::now();
    if ((now - last_measurement) < ros::Duration(5)) {
      auto dT = now - last_loop;

      for (auto& tup : ekf) {
        tup.second.processModel(dT.toSec());
      }
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 1;
}
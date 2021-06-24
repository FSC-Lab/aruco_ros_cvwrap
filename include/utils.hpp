#include <ros/ros.h>
#include <opencv2/aruco.hpp>

using namespace std::string_literals;

namespace utils {
/**
 * @brief Set the Aruco Dict by retrieving the "aruco_dictionary" object from the parameter server
 *
 * @param n the NodeHandle object
 * @return cv::Ptr<cv::aruco::Dictionary> the aruco Dictionary
 */
cv::Ptr<cv::aruco::Dictionary> SetArucoDictFromRosParams(const ros::NodeHandle& n);

/**
 * @brief Set the Detector Params From Ros Params object
 *
 * @param n the NodeHandle object
 * @return cv::Ptr<cv::aruco::DetectorParameters> the aruco detector parameters object
 */
cv::Ptr<cv::aruco::DetectorParameters> SetDetectorParamsFromRosParams(const ros::NodeHandle& n);
}  // namespace utils
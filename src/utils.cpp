#include <utils.hpp>

cv::Ptr<cv::aruco::Dictionary> utils::SetArucoDictFromRosParams(const ros::NodeHandle& n) {
  auto name = n.param("aruco_dictionary", "DICT_6X6_250"s);
  if (name == "DICT_4X4_100") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
  } else if (name == "DICT_4X4_1000") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
  } else if (name == "DICT_4X4_250") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
  } else if (name == "DICT_4X4_50") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  } else if (name == "DICT_5X5_100") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
  } else if (name == "DICT_5X5_1000") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
  } else if (name == "DICT_5X5_250") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
  } else if (name == "DICT_5X5_50") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
  } else if (name == "DICT_6X6_100") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
  } else if (name == "DICT_6X6_1000") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
  } else if (name == "DICT_6X6_250") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  } else if (name == "DICT_6X6_50") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
  } else if (name == "DICT_7X7_100") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_100);
  } else if (name == "DICT_7X7_1000") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_1000);
  } else if (name == "DICT_7X7_250") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_250);
  } else if (name == "DICT_7X7_50") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_50);
  } else if (name == "DICT_ARUCO_ORIGINAL") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
  } else {
    n.deleteParam("aruco_dictionary");
    ROS_ERROR("%s is not a valid aruco dictionary name", name.c_str());
    return cv::Ptr<cv::aruco::Dictionary>();
  }
}

cv::Ptr<cv::aruco::DetectorParameters> utils::SetDetectorParamsFromRosParams(const ros::NodeHandle& n) {
  auto p = cv::aruco::DetectorParameters::create();

  p->adaptiveThreshConstant = n.param("adaptiveThreshConstant", p->adaptiveThreshConstant);
  p->adaptiveThreshWinSizeMax = n.param("adaptiveThreshWinSizeMax", p->adaptiveThreshWinSizeMax);
  p->adaptiveThreshWinSizeMin = n.param("adaptiveThreshWinSizeMin", p->adaptiveThreshWinSizeMin);
  p->adaptiveThreshWinSizeStep = n.param("adaptiveThreshWinSizeStep", p->adaptiveThreshWinSizeStep);
  p->cornerRefinementMaxIterations = n.param("cornerRefinementMaxIterations", p->cornerRefinementMaxIterations);
  p->cornerRefinementMinAccuracy = n.param("cornerRefinementMinAccuracy", p->cornerRefinementMinAccuracy);
  p->cornerRefinementWinSize = n.param("cornerRefinementWinSize", p->cornerRefinementWinSize);
  p->errorCorrectionRate = n.param("errorCorrectionRate", p->errorCorrectionRate);
  p->markerBorderBits = n.param("markerBorderBits", p->markerBorderBits);
  p->maxErroneousBitsInBorderRate = n.param("maxErroneousBitsInBorderRate", p->maxErroneousBitsInBorderRate);
  p->maxMarkerPerimeterRate = n.param("maxMarkerPerimeterRate", p->maxMarkerPerimeterRate);
  p->minCornerDistanceRate = n.param("minCornerDistanceRate", p->minCornerDistanceRate);
  p->minDistanceToBorder = n.param("minDistanceToBorder", p->minDistanceToBorder);
  p->minMarkerDistanceRate = n.param("minMarkerDistanceRate", p->minMarkerDistanceRate);
  p->minMarkerPerimeterRate = n.param("minMarkerPerimeterRate", p->minMarkerPerimeterRate);
  p->minOtsuStdDev = n.param("minOtsuStdDev", p->minOtsuStdDev);
  p->perspectiveRemoveIgnoredMarginPerCell =
      n.param("perspectiveRemoveIgnoredMarginPerCell", p->perspectiveRemoveIgnoredMarginPerCell);
  p->perspectiveRemovePixelPerCell = n.param("perspectiveRemovePixelPerCell", p->perspectiveRemovePixelPerCell);
  p->polygonalApproxAccuracyRate = n.param("polygonalApproxAccuracyRate", p->polygonalApproxAccuracyRate);
  return p;
}
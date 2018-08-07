//
// Created by andreas on 8/7/18.
//

#ifndef VISUAL_ODOM_CAMERA_PARAMETERS_H
#define VISUAL_ODOM_CAMERA_PARAMETERS_H

#include <vector>
#include "math.h"

#include <cv_bridge/cv_bridge.h>
#include "opencv2/calib3d/calib3d.hpp"

class CameraParameters {
public:
    CameraParameters(ros::NodeHandle nh, std::string calibration_file = "");

    CameraParameters(ros::NodeHandle nh);

    float radiansToDegrees(float radians);

    float degreesToRadians(float degrees);

    float normalizeDegrees(float degrees);

    void setTiltDegrees(float degrees);

    void setTiltRadians(float degrees);

    void setDistanceFromGround(float height);

    void setDistanceFromCenter(float distance);

    void setHeight(float height);

    void setWidth(float width);

    void setCalibrationFile(std::string calibration_file);

    void setImageTopic(std::string image_topic);

    void setTwistTopic(std::string twist_topic);

    void setOdomTopic(std::string odom_topic);

    void setSubtensionHorizontal(float meters);

    void setSubtensionVertical(float meters);

    float getDistanceFromGround();

    float getDistanceFromCenter();

    float getTilt();

    float getHeight();

    float getWidth();

    float getSubtensionHorizontal();

    float getSubtensionVertical();

    std::string getCalibrationFile();

    std::string getImageTopic();

    std::string getTwistTopic();

    std::string getOdomTopic();

    cv::Matx33d &getCameraCalibrationMatrix();

    cv::Matx33d &getWMatrix();

    cv::Mat &getDistortionCoefficients();

    cv::Mat &getRectificationMatrix();

    cv::Mat &getProjectionMatrix();


protected:
private:
    ros::NodeHandle nodeHandle_;
    std::string calibration_file_;
    std::string image_topic_;
    std::string odom_topic_;
    std::string twist_topic_;
    float distance_from_center_;
    float distance_from_ground_;
    float tilt_degrees_;
    float height_;
    float width_;
    float subtension_vertical_m_;
    float subtension_horizontal_m_;
    cv::Matx33d camera_calibration_matrix_;
    cv::Matx33d W_;
    cv::Mat distortion_coefficients_;
    cv::Mat rectification_matrix_;
    cv::Mat projection_matrix_;
};

#endif //VISUAL_ODOM_CAMERA_PARAMETERS_H

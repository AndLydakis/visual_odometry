//
// Created by andreas on 8/7/18.
//

#include "../include/camera_parameters.h"
#include <math.h>
#include <vector>


float CameraParameters::degreesToRadians(float degrees) {
    return (degrees * M_PI) / 180;
}

float CameraParameters::radiansToDegrees(float radians) {
    return radians * (180 / M_PI);
}

void CameraParameters::setDistanceFromGround(float height) {
    distance_from_ground_ = height >= 0 ? height : 0;
}

void CameraParameters::setDistanceFromCenter(float distance) {
    distance_from_center_ = distance >= 0 ? distance : 0;
}

float CameraParameters::normalizeDegrees(float degrees) {
    degrees = fmod(degrees + 180, 360.0);
    if (degrees < 0) degrees += 360.0;
    return degrees - 180;
}


void CameraParameters::setTiltDegrees(float degrees) {
    tilt_degrees_ = normalizeDegrees(degrees);
}

void CameraParameters::setTiltRadians(float radians) {
    tilt_degrees_ = normalizeDegrees(radiansToDegrees(radians));
}

float CameraParameters::getDistanceFromCenter() {
    return distance_from_center_;
}

float CameraParameters::getDistanceFromGround() {
    return distance_from_ground_;
}

float CameraParameters::getTilt() {
    return tilt_degrees_;
}

int CameraParameters::getHeight() {
    return height_;
}

int CameraParameters::getWidth() {
    return width_;
}

float CameraParameters::getSubtensionHorizontal() {
    return subtension_horizontal_m_;
}

float CameraParameters::getSubtensionVertical() {
    return subtension_vertical_m_;
}

void CameraParameters::setHeight(float height) {
    height_ = height >= 0 ? height : 0;
}

void CameraParameters::setWidth(float width) {
    width_ = width >= 0 ? width : 0;
}

void CameraParameters::setCalibrationFile(std::string calibration_file) {
    calibration_file_ = calibration_file;
}

std::string CameraParameters::getCalibrationFile() {
    return calibration_file_;
}

void CameraParameters::setImageTopic(std::string image_topic) {
    image_topic_ = image_topic;
}

void CameraParameters::setTwistTopic(std::string twist_topic) {
    twist_topic_ = twist_topic;
}

void CameraParameters::setOdomTopic(std::string odom_topic) {
    odom_topic_ = odom_topic;
}

void CameraParameters::setSubtensionHorizontal(float meters) {
    subtension_horizontal_m_ = meters >= 0 ? meters : 0;
}

void CameraParameters::setSubtensionVertical(float meters) {
    subtension_vertical_m_ = meters >= 0 ? meters : 0;
}

CameraParameters::CameraParameters(ros::NodeHandle nh, std::string calibration_file) : nodeHandle_(nh),
                                                                                       calibration_file_(
                                                                                               calibration_file) {

}

cv::Mat &CameraParameters::getDistortionCoefficients() {
    return distortion_coefficients_;
}


cv::Mat &CameraParameters::getRectificationMatrix() {
    return rectification_matrix_;
}

cv::Mat &CameraParameters::getProjectionMatrix() {
    return projection_matrix_;
}

cv::Matx33d &CameraParameters::getCameraCalibrationMatrix() {
    return camera_calibration_matrix_;
}

cv::Matx33d &CameraParameters::getWMatrix() {
    return W_;
}

CameraParameters::CameraParameters(ros::NodeHandle nh) : nodeHandle_(nh) {
    if (!nodeHandle_.getParam("/image_height", height_)) throw std::runtime_error("Could not read image height");
    if (!nodeHandle_.getParam("/image_width", width_)) throw std::runtime_error("Could not read image width");
    int rows = 0;
    int cols = 0;
    std::vector<double> data;
    if (!nodeHandle_.getParam("/camera_matrix/rows", rows) ||
        !nodeHandle_.getParam("/camera_matrix/cols", cols) ||
        !nodeHandle_.getParam("/camera_matrix/data", data))
        throw std::runtime_error("Could not initialize camera calibration matrix");
    std::cout << rows << " " << cols << std::endl;
    data.resize(rows * cols);
//    for (auto i : data) {
//        std::cout << data[i] << std::endl;
//    }
    camera_calibration_matrix_ = cv::Mat(rows, cols, CV_64F, &data[0]);

    std::vector<double> data2;
    if (!nodeHandle_.getParam("/distortion_coefficients/rows", rows) ||
        !nodeHandle_.getParam("/distortion_coefficients/cols", cols) ||
        !nodeHandle_.getParam("/distortion_coefficients/data", data2))
        throw std::runtime_error("Could not initialize camera distortion coefficients matrix");
    std::cout << rows << " " << cols << std::endl;
    data2.resize(rows * cols);
    distortion_coefficients_ = cv::Mat(rows, cols, CV_64F, &data2[0]);
    data2.clear();

    std::vector<double> data3;
    if (!nodeHandle_.getParam("/rectification_matrix/rows", rows) ||
        !nodeHandle_.getParam("/rectification_matrix/cols", cols) ||
        !nodeHandle_.getParam("/rectification_matrix/data", data3))
        throw std::runtime_error("Could not initialize rectification matrix");
    std::cout << rows << " " << cols << std::endl;
    std::cout <<data3.
    data3.resize(rows * cols);
    rectification_matrix_ = cv::Mat(rows, cols, CV_64F, &data3[0]);
    data3.clear();

    std::vector<double> data4;
    if (!nodeHandle_.getParam("/projection_matrix/rows", rows) ||
        !nodeHandle_.getParam("/projection_matrix/cols", cols) ||
        !nodeHandle_.getParam("/projection_matrix/data", data4))
        throw std::runtime_error("Could not initialize projection matrix");
    std::cout << rows << " " << cols << std::endl;
    data4.resize(rows * cols);
    projection_matrix_ = cv::Mat(rows, cols, CV_64F, &data4[0]);
    data4.clear();
}

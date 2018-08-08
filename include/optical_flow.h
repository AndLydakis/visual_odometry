//
// Created by andreas on 8/7/18.
//

#ifndef VISUAL_ODOM_OPTICAL_FLOW_H
#define VISUAL_ODOM_OPTICAL_FLOW_H

#define FILTER_SIZE 5
#define DROPPED_THRESHOLD 0.75
#define MAX_POINTS 300
#define DEADBAND_M 0.0005
#define ROBOT_RADIUS 0.15
#define ROTATION_CONSTANT 1.57

#include <iostream>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <cmath>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "compressed_image_transport/compressed_subscriber.h"
#include "compressed_image_transport/compression_common.h"

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/video/tracking.hpp>
#include <optical_flow.h>

#include "camera_parameters.h"

class OpticalFlow {
public:
    explicit OpticalFlow(ros::NodeHandle nh, std::string image_topic = "/usb_cam/image_raw",
                         std::string twist_topic = "/twist/visual",
                         int filter_size = FILTER_SIZE, double dropped_threshold = DROPPED_THRESHOLD,
                         int max_points = MAX_POINTS, double deadband = DEADBAND_M, double robot_radius = ROBOT_RADIUS,
                         double rotation_constant = ROTATION_CONSTANT);

    ~OpticalFlow();

    void imageCallback(const sensor_msgs::ImageConstPtr &image);

    cv::Point3f feature_motion_estimate(std::vector<cv::Point2f> &prev_coords, std::vector<cv::Point2f> &curr_coords);

    void setFilterSize(int fs);

    void setMaxTrackedPoints(int pts);

    void setDroppedThreshold(double thresh);

    void setDeadbandMeters(double deadband);

    void setRobotRadius(double radius);

    void setRotationConstant(double constant);

    void setImageTopic(std::string image_topic);

    void setTwistTopic(std::string twist_topic);

    int getFilterSize() const;

    int getMaxTrackedPoints() const;

    double getDroppedThreshold() const;

    double getDeadBandMeters() const;

    double getRobotRadius() const;

    double getRotationConstant() const;

    std::string getImageTopic() const;

    std::string getTwistTopic() const;

    void print();

protected:
private:
    ros::NodeHandle nh_;
    ros::Publisher twist_publisher_;
    image_transport::ImageTransport image_transport_;
    image_transport::Subscriber image_subscriber_;
    std::string image_topic_;
    std::string twist_topic_;
    std::string windowName_ = "Tracked Features";

    std::vector<geometry_msgs::Twist> filter_;

    CameraParameters cameraParameters_;

    //orthogonal matrix
    cv::Matx33d W_;
    //Homography matrix
    cv::Matx33d H_;
    //Fundamental matrix
    cv::Matx33d F_;
    //Essential matrix
    cv::Matx33d E_;

    cv::Mat curr_;
    cv::Mat prev_;
    std::vector<float> flow_errors_;
    std::vector<unsigned char> flow_status_;
    std::vector<cv::Point2f> curr_track_indices_;
    std::vector<cv::Point2f> prev_track_indices_;

    cv::Point3f accumulated_motion_ = cv::Point3f(0.0, 0.0, 0.0);

    int filter_size_;
    int max_points_;
    double dropped_threshold_;
    double deadband_;
    double robot_radius_;
    double rotation_constant_;
    double current_timestamp_;
    double previous_timestamp_;
    int filter_counter_ = 0;
};

#endif //VISUAL_ODOM_OPTICAL_FLOW_H

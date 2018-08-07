//
// Created by andreas on 8/7/18.
//

#ifndef VISUAL_ODOM_OPTICAL_FLOW_H
#define VISUAL_ODOM_OPTICAL_FLOW_H

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

#include "camera_parameters.h"

class OpticalFlow {
public:
    OpticalFlow(ros::NodeHandle nh, std::string image_topic = "/usb_cam/image_raw", std::string odom_topic,
                std::string twist_topic);

    ~OpticalFlow() {}

protected:
private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    ros::Publisher odom_publisher_;
    ros::Publisher twist_publisher_;
    std::string image_topic_;
    std::string odom_topic_;
    std::string twist_topic_;
    CameraParameters cameraParameters_;
};

#endif //VISUAL_ODOM_OPTICAL_FLOW_H

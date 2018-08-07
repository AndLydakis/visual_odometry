#ifndef VISUAL_ODOM_OPTICAL_FLOW_CPP
#define VISUAL_ODOM_OPTICAL_FLOW_CPP

#include "optical_flow.h"
#include "../include/optical_flow.h"


#endif


OpticalFlow::OpticalFlow(ros::NodeHandle nh, std::string image_topic, std::string odom_topic, std::string twist_topic)
        : nh_(nh), cameraParameters_(nh_), image_topic_(image_topic), odom_topic_(odom_topic),
          twist_topic_(twist_topic) {

}

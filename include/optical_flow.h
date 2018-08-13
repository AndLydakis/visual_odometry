//
// Created by Andreas Lydakis on 8/7/18.
//

#ifndef VISUAL_ODOM_OPTICAL_FLOW_H
#define VISUAL_ODOM_OPTICAL_FLOW_H

/** @defgroup defines_
 *  @name Default Constructor Parameters
 *  Safe default values for constructor parameters.
 */
/**@{*/
///Number of motion measurements to average over
#define FILTER_SIZE 5
///Create a new set of tracked features if more than this have been dropped between 2 iterations
#define DROPPED_THRESHOLD 0.75
///Number of points to keep track of
#define MAX_POINTS 300
///Ignore movement smaller than this
#define DEADBAND_M 0.0005
///Radius of robot base
#define ROBOT_RADIUS 0.44
///Constant for motion estimate, experiment with the robot and find the best value for your case
#define ROTATION_CONSTANT 1.57
///Default topic to listen for images
#define DEFAULT_IMAGE_TOPIC "/usb_cam/image_raw"
///Default topic to publish motion estimation
#define DEFAULT_TWIST_TOPIC "/twist/visual"
/**@}*/

#include <iostream>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <cmath>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/Twist.h"
#include "compressed_image_transport/compressed_subscriber.h"
#include "compressed_image_transport/compression_common.h"

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/video/tracking.hpp>
#include <optical_flow.h>

#include "camera_parameters.h"

/**
 * Class that provides a motion estimate between consecutive images. Uses images streamed in a ros topic
 * and publishes a geometry_msgs/Twist message. To be used with other odometry sources such as IMUs.
 * Heavily based on  <a href="https://github.com/apollack11/advanced-computer-vision">https://github.com/apollack11/advanced-computer-vision</a>
 */
class OpticalFlow {
public:
    /**
     * Constructor: The only required argument is a ros::NodeHandle, the rest can assume default values
     * but should be set depending on your implementation
     *
     * @param nh Handles all the ROS communications
     * @param image_topic ROS topic where the camera images are being published. See OpticalFlow::image_subscriber_
     * @param twist_topic ROS topic where the calculated motion is published. See OpticalFlow::twist_publisher_
     * @param dropped_threshold Calculate new features if more than this have been lost between frames
     * @param filter_size The number of motion estimates that is used to calculate the average motion
     * @param max_points The number of features to track
     * @param deadband Ignore motion smaller than this (meters)
     * @param robot_radius The radius of the robot base (meters)
     * @param rotation_constant Constant that is used for the motion calculation, it should be experimentally determined for
     * each platform
     */
    explicit OpticalFlow(ros::NodeHandle nh,
                         std::string image_topic = DEFAULT_IMAGE_TOPIC,
                         std::string twist_topic = DEFAULT_TWIST_TOPIC,
                         int filter_size = FILTER_SIZE,
                         double dropped_threshold = DROPPED_THRESHOLD,
                         int max_points = MAX_POINTS,
                         double deadband = DEADBAND_M,
                         double robot_radius = ROBOT_RADIUS,
                         double rotation_constant = ROTATION_CONSTANT);

    /**
     * Destructor
     */
    ~OpticalFlow();

    /**
     * ROS callback that handles the image processing
     * @param image A sensor_msgs/ImageConstPtr message containing a compressed image from the RGB camera
     */
    void imageCallback(const sensor_msgs::ImageConstPtr &image);

    /**
     * Function that estimates the average robot motion based on the motion of the different features in consecutive frames
     * @param previous_coordinates The coordinates of the tracked features in the previous frame
     * @param current_coordinates The coordinates of the tracked features in the current frame
     * @return A point containing the averaged motion in the x,y,z axes (z is always zero for our case)
     */
    cv::Point3f feature_motion_estimate(std::vector<cv::Point2f> &previous_coordinates,
                                        std::vector<cv::Point2f> &current_coordinates);

    /**
     * Set the filter size
     * @param fs The new size of the filter (>0)
     */
    void setFilterSize(int fs);

    /**
     * Set the number of tracked points
     * @param pts The new number of tracked points (>0)
     */
    void setMaxTrackedPoints(int pts);

    /**
     * Set the threshold to reset tracked features
     * @param thresh The new threshold (in (0-1])
     */
    void setDroppedThreshold(double thresh);

    /**
     * Set the deadband for ignoring small motions
     * @param deadband The new deadband (in meters >= 0)
     */
    void setDeadbandMeters(double deadband);

    /**
     * Set the robot radius
     * @param radius The new robot radius (in meters >0)
     */
    void setRobotRadius(double radius);

    /**
     * Set the rotation constant for motion calculation
     * @param constant The new rotation constant
     */
    void setRotationConstant(double constant);

    /**
     * Set the topic that OpticalFlow::image_subscriber_ uses to listen for images
     * @param image_topic The new topic
     */
    void setImageTopic(std::string image_topic);

    /**
     * Set the topic that OpticalFlow::twist_publisher_ uses to publish the estimated motion
     * @param twist_topic The new topic
     */
    void setTwistTopic(std::string twist_topic);

    /**
     * Return the filter size
     * @return The filter size (OpticalFlow::filter_size_)
     */
    int getFilterSize() const;

    /**
     * Return the number of tracked features
     * @return The number of tracked features (OpticalFlow::max_points_)
     */
    int getMaxTrackedPoints() const;

    /**
     * Return the dropped point threshold for generating a new feature set
     * @return The threshold (OpticalFlow::dropped_threshold_)
     */
    double getDroppedThreshold() const;

    /**
     * Return the deadband for motion to ignore in meters
     * @return The deadband (OpticalFlow::deadband_)
     */
    double getDeadBandMeters() const;

    /**
     * Return the robot radius (meters)
     * @return The robot radius (OpticalFlow::robot_radius_)
     */
    double getRobotRadius() const;

    /**
     * Return the rotation constant
     * @return The rotation constant (OpticalFlow::rotation_constant_)
     */
    double getRotationConstant() const;

    /**
     * Return the topic OpticalFlow::image_subscriber_ listens to for images
     * @return The topic that OpticalFlow::image_subscriber_ listens to (OpticalFlow::image_topic_)
     */
    std::string getImageTopic() const;

    /**
     * Return the topic OpticalFlow::twist_publisher_ publishes the estimated motion to
     * @return The topic that OpticalFlow::twist_publisher_ publishes to (OpticalFlow::twist_topic_)
     */
    std::string getTwistTopic() const;

    /**
     * Outputs the basic parameters of the current object
     */
    void print();

protected:
private:
    ///Needed for ROS Interface
    ros::NodeHandle nh_;
    ///Publishes the estimated motion
    ros::Publisher twist_publisher_;
    ///Needed for ROS image processing
    image_transport::ImageTransport image_transport_;
    ///Listens for the incoming images
    image_transport::Subscriber image_subscriber_;
    ///The ROS topic used by OpticalFlow::image_subsciber_
    std::string image_topic_;
    ///The ROS topic used by OpticalFlow::twist_publisher_
    std::string twist_topic_;
    ///The title of the visualization window
    std::string windowName_ = "Tracked Features";
    ///A vector that stores recent motion estimates
    std::vector<geometry_msgs::Twist> filter_;
    ///A CameraParameters instance that stores the used camera attributes
    CameraParameters cameraParameters_;

    ///The homography mapping matrx for points between frames described in Chapter 9.2 in Multiple View Geometry in Computer Vision (Hartley-Zisserman)
    cv::Matx33d H_;
    ///The fundamental matrix F described in Chapter 9.2 in Multiple View Geometry in Computer Vision (Hartley-Zisserman)
    cv::Matx33d F_;
    ///The essential E matrix described in Chapter 9.6 in Multiple View Geometry in Computer Vision (Hartley-Zisserman)
    cv::Matx33d E_;
    ///The W matrix used in Eq. 9.13 in Multiple View Geometry in Computer Vision (Hartley-Zisserman)
    cv::Matx33d W_;

    ///cv::Mat representation of the current frame
    cv::Mat current_image_;
    ///cv::Mat representation of the previous frame
    cv::Mat previous_image_;
    ///The error in the flow calculation for each feature
    std::vector<float> flow_errors_;
    ///1 if the corresponding feature has been found during the flow calculation, 0 otherwise
    std::vector<unsigned char> flow_status_;
    ///Coordinates of tracked features on the current frame
    std::vector<cv::Point2f> curr_track_indices_;
    ///Coordinates of tracked features on the previous frame
    std::vector<cv::Point2f> prev_track_indices_;
    ///The total accumulated motion so far (in X, Y, Z coordinates). Initialized to zero
    cv::Point3f accumulated_motion_ = cv::Point3f(0.0, 0.0, 0.0);
    ///Size of stored frames to average motion
    int filter_size_;
    ///Number of features to track
    int max_points_;
    ///Determines the position of the new frame in the frame vector
    int filter_counter_ = 0;
    ///Percentage of features that are allowed to be dropped between frames before calculating a new set
    double dropped_threshold_;
    ///Motion less than this will be ignored to avoid accumulating errors (meters)
    double deadband_;
    ///The radius of the robotic platform (meters)
    double robot_radius_;
    ///Experimentally defined rotation constant for the motion calculation (see OpticalFlow::feature_motion_estimate)
    double rotation_constant_;
    ///Current frame timestamp in seconds
    double current_timestamp_;
    ///Previous frame timestamp in seconds
    double previous_timestamp_;
};

#endif //VISUAL_ODOM_OPTICAL_FLOW_H

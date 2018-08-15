#include <utility>

#ifndef VISUAL_ODOM_OPTICAL_FLOW_CPP
#define VISUAL_ODOM_OPTICAL_FLOW_CPP

#include <optical_flow.h>

#endif


/**
 * Initializes the instance, and its CameraParameters object, initializes subscribers
 * and publishes as well as the H and W matrices
 */
OpticalFlow::OpticalFlow(ros::NodeHandle nh)
        : nh_(nh), cameraParameters_(nh), image_transport_(nh) {
    try {
        nh_.param<std::string>("twist_topic", twist_topic_, DEFAULT_TWIST_TOPIC);
        nh_.param<std::string>("image_topic", image_topic_, DEFAULT_IMAGE_TOPIC);
        nh_.param<int>("filter_size", filter_size_, FILTER_SIZE);
        nh_.param<int>("max_points", max_points_, MAX_POINTS);
        nh_.param<double>("deadband", deadband_, DEADBAND_M);
        nh_.param<double>("robot_radius", robot_radius_, ROBOT_RADIUS);
        nh_.param<double>("rotation_constant", rotation_constant_, ROTATION_CONSTANT);
        nh_.param<double>("dropped_threshold", dropped_threshold_, DROPPED_THRESHOLD);

        image_subscriber_ = image_transport_.subscribe(image_topic_, 1, &OpticalFlow::imageCallback, this,
                                                       image_transport::TransportHints("compressed"));
        twist_publisher_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>(twist_topic_, 1);
        cv::namedWindow(windowName_, CV_WINDOW_AUTOSIZE);
        std::vector<double> wData{0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
        W_ = cv::Mat(3, 3, CV_64F, &wData[0]);
        std::vector<double> HData{0.0002347417933653588, -9.613823951336309e-20, -0.07500000298023225,
                                  -7.422126200315807e-19, -0.0002818370786240783, 0.5159999728202818,
                                  1.683477982667922e-19, 5.30242624981192e-18, 1};
        H_ = cv::Mat(3, 3, CV_64F, &HData[0]);
        previous_timestamp_ = ros::Time::now().toSec();
        filter_.resize(filter_size_);
        geometry_msgs::TwistWithCovarianceStamped msg_ = geometry_msgs::TwistWithCovarianceStamped();

        msg_.twist.twist.linear.x = 0.0;
        msg_.twist.twist.linear.y = 0.0;
        msg_.twist.twist.linear.z = 0.0;
        msg_.twist.twist.angular.x = 0.0;
        msg_.twist.twist.angular.y = 0.0;
        msg_.twist.twist.angular.z = 0.0;

        for (int i = 0; i < filter_size_; ++i) {
            filter_.emplace_back(msg_);
        }
        filter_counter_ = 0;

    } catch (const ros::Exception &re) {
        std::cerr << "ROS error: " << re.what() << std::endl;
        throw (re);
    } catch (const cv::Exception &cve) {
        std::cerr << "CV error: " << cve.what() << std::endl;
        throw (cve);
    } catch (...) {
        ROS_ERROR("Something went wrong");
        throw (std::runtime_error("Something went wrong"));
    }
    print();
}

/**
 * Destroys the visualization window
 */
OpticalFlow::~OpticalFlow() {
    cv::destroyAllWindows();
}

/**
 * Callback that gets activated when a new sensor_msgs::ImageConstPtr is sent by the camera
 *  - Creates a cv::Mat representation of the RGB image
 *  - Creates a cv::Mat representation of the greyscale image
 *  - If there are not enough features from the previous image it calculates a new set (cv::goodFeaturesToTrack())
 *  - If there are no availalbe features the function returns
 *  - Uses the iterative Lucas-Kanade method to calculates an optical flow for a sparse feature set using with pyramids. (cv::calcOpticalFlowPyrLK())
 *  - Uses OpticalFlow::feature_motion_estimate() to calculate the estimated motion
 *  - Averages the motion for the stored measurements
 *  - Publishes the estimated motion as a geometry_msgs::TwistWithCovarianceStamped message
 *  - Calculates the epipolar lines (correspondance between the two frames) (cv::computeCorrespondEpilines())
 *  - Draws the tracked features on the RGB image
 *  - Updates the timestamps and tracked feature indices
 */
void OpticalFlow::imageCallback(const sensor_msgs::ImageConstPtr &image) {
//    std::cout << "Enter Image Callback\n";
    try {
//        std::cout << "Create grayscale image\n";
        cv::Mat color_image_ = (cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8))->image;
        cv::cvtColor(color_image_, current_image_, CV_BGR2GRAY);
        if (!previous_image_.data) {
            current_image_.copyTo(previous_image_);
            return;
        }
//        std::cout << "Calculate new set of points if there are not enough from previous image\n";
        if (prev_track_indices_.size() < dropped_threshold_ * max_points_) {
            cv::goodFeaturesToTrack(previous_image_, prev_track_indices_, max_points_, 0.1, 5.0);
        }
        if (prev_track_indices_.empty()) {
            ROS_WARN("No tracked features");
            current_image_.copyTo(previous_image_);
            return;
        };

//        std::cout << "Calculate optical flow\n";
        cv::calcOpticalFlowPyrLK(previous_image_, current_image_, prev_track_indices_, curr_track_indices_,
                                 flow_status_, flow_errors_,
                                 cv::Size(21, 21), 4);

        if (curr_track_indices_.size() != prev_track_indices_.size()) {
            ROS_ERROR("tracking index data size different between previous and current images");
            return;
        }

//        std::cout << "Calculate motion estimate";
        cv::Point3f estimated_motion = feature_motion_estimate(prev_track_indices_, curr_track_indices_);
        std::cout << "Estimated Motion This Iteration: " << estimated_motion.x << " " << estimated_motion.y << " "
                  << estimated_motion.z << std::endl;
        accumulated_motion_ += estimated_motion;
        std::cout << "Estimated Motion so Far: " << accumulated_motion_ << ", Rotation:"
                  << accumulated_motion_.x / (2 * M_PI * robot_radius_) * 360 * rotation_constant_ << std::endl;

        F_ = findFundamentalMat(prev_track_indices_, curr_track_indices_, CV_FM_LMEDS, 1, 0.99, flow_status_);


        // calculate Essential matrix from Fundamental matrix and camera matrix
        //E_ = camera_parameters_.getCameraCalibrationMatrix().t * F_ * camera_parameters_.getCameraCalibrationMatrix();
        // helpful clarification:
        // http://stackoverflow.com/questions/16639106/camera-motion-from-corresponding-images
        // syntax inspiration found at:
        // http://www.morethantechnical.com/2012/02/07/structure-from-motion-and-3d-reconstruction-on-the-easy-in-opencv-2-3-w-code/
        //SVD svd(E_);
        //R_ = svd.u * Mat(W_) * svd.vt;
        //t_ = svd.u.col(2);


        //Create new twist msg
        geometry_msgs::TwistWithCovarianceStamped twist_msg;
        current_timestamp_ = ros::Time::now().toSec();
        double t_diff = current_timestamp_ - previous_timestamp_;
        twist_msg.twist.twist.linear.x = estimated_motion.y / t_diff;
        twist_msg.twist.twist.angular.z =
                estimated_motion.x / (2 * M_PI * robot_radius_) * 2 * M_PI * rotation_constant_ / t_diff;
        //Put it in the filter
        filter_[filter_counter_] = twist_msg;

        twist_msg.twist.twist.linear.x = 0.0;
        twist_msg.twist.twist.linear.y = 0.0;
        twist_msg.twist.twist.linear.z = 0.0;
        twist_msg.twist.twist.angular.x = 0.0;
        twist_msg.twist.twist.angular.y = 0.0;
        twist_msg.twist.twist.angular.z = 0.0;
        //Average measurements
        for (int i = 0; i < filter_size_; ++i) {
            twist_msg.twist.twist.linear.x += filter_[i].twist.twist.linear.x / filter_size_;
            twist_msg.twist.twist.linear.y += filter_[i].twist.twist.linear.y / filter_size_;
            twist_msg.twist.twist.angular.x += filter_[i].twist.twist.angular.x / filter_size_;
            twist_msg.twist.twist.angular.y += filter_[i].twist.twist.angular.y / filter_size_;
            twist_msg.twist.twist.angular.z += filter_[i].twist.twist.angular.z / filter_size_;
        }

        //Publish messages
        twist_msg.header.stamp = ros::Time::now();
        twist_publisher_.publish(twist_msg);

        //Update the next position of the filter
        ++filter_counter_ %= filter_size_;

        //Draw features
        cv::Mat draw_image = color_image_;
        std::vector<cv::Vec<float, 3>> epilines1, epilines2;
        cv::computeCorrespondEpilines(prev_track_indices_, 1, F_, epilines1);
        cv::computeCorrespondEpilines(curr_track_indices_, 2, F_, epilines2);
        CV_Assert(prev_track_indices_.size() == epilines1.size() && epilines1.size() == epilines2.size());
        cv::RNG rng(0);
        for (int i = 0; i < prev_track_indices_.size(); i++) {
            // Epipolar lines of the 1st point set are drawn in the 2nd image and vice-versa
            cv::Scalar color(rng(256), rng(256), rng(256));
            cv::circle(draw_image, curr_track_indices_[i], 3, cv::Scalar(0, 255, 0), -1, CV_AA);
        }

        imshow(windowName_, draw_image);
        cv::waitKey(30);
        //update images, indices and timestamps
        current_image_.copyTo(previous_image_);
        prev_track_indices_ = curr_track_indices_;

        auto first = prev_track_indices_.begin();
        auto new_start = first;
        auto last = prev_track_indices_.end();

        int pixelHeight = cameraParameters_.getHeight();
        int pixelWidth = cameraParameters_.getWidth();
        while (first != last) {
            if ((*first).x < 0.0 || (*first).x > pixelHeight || (*first).y < 0.0 || (*first).y > pixelWidth) {
                prev_track_indices_.erase(first);
            }
            ++first;
        }
        prev_track_indices_.begin() = new_start;
        previous_timestamp_ = current_timestamp_;

    } catch (const ros::Exception &re) {
        ROS_ERROR("ROS error: %s", re.what());
        std::cerr << "ROS error: " << re.what() << std::endl;
        return;
    } catch (const cv::Exception &cve) {
        ROS_ERROR("CV error: %s", cve.what());
        std::cerr << "CV error: " << cve.what() << std::endl;
        return;
    } catch (const cv_bridge::Exception &cvbe) {
        ROS_ERROR("cv_bridge error: %s", cvbe.what());
        std::cerr << "CV error: " << cvbe.what() << std::endl;
        return;
    } catch (...) {
        ROS_ERROR("Something went wrong");
        return;
    }
//    std::cout << "Exit Image Callback\n";

}

/**
 * Estimates the motion of features between two consecutive frames
 * by converting them from euclidean to homogeneous space and
 * averaging the difference in coordinates
 *
 * If the motion is smaller than OpticalFlow::deadband_ it returns a 0 movement
 */
cv::Point3f
OpticalFlow::feature_motion_estimate(std::vector<cv::Point2f> &previous_coordinates,
                                     std::vector<cv::Point2f> &current_coordinates) {
//    std::cout << "Estimating Motion\n";
    std::vector<cv::Point3f> previous_homogeneous_points;
    cv::convertPointsToHomogeneous(previous_coordinates, previous_homogeneous_points);
    auto prev_it = previous_homogeneous_points.begin();

    std::vector<cv::Point3f> current_homogeneous_points;
    cv::convertPointsToHomogeneous(current_coordinates, current_homogeneous_points);
    auto curr_it = current_homogeneous_points.begin();

    cv::Point3f average_motion(0.0, 0.0, 0.0);

    int N = previous_homogeneous_points.size();
//    std::cout << "Looping through points\n";
    for (; curr_it != current_homogeneous_points.end(); ++prev_it, ++curr_it) {
//        std::cout << (*curr_it).x << " " << (*curr_it).y << " " << (*curr_it).z << " | " << (*prev_it).x << " "
//                  << (*prev_it).y << " " << (*prev_it).z << std::endl;
        average_motion += cv::Point3f((H_ * cv::Mat(*curr_it) - H_ * cv::Mat(*prev_it)) / N);
    }
    //?
//    std::cout << "Adjusting averaged motion\n";
    average_motion.y *= -rotation_constant_ * 4.0 / 3.0;
//    std::cout << average_motion.x << " " << average_motion.y << " " << average_motion.z << std::endl;
    if ((fabs(average_motion.x) > deadband_) || (fabs(average_motion.y) > deadband_)) return average_motion;
    return cv::Point3f(0.0, 0.0, 0.0);
}

/**
 * Sets the filter size, if the input value is <= 0 it
 * will use the default value \ref FILTER_SIZE
 */
void OpticalFlow::setFilterSize(int fs) {
    filter_size_ = fs > 0 ? fs : FILTER_SIZE;
}

/**
 * Sets the number of tracked points, if the input value is <= 0 it
 * will use the default value \ref MAX_POINTS
 */
void OpticalFlow::setMaxTrackedPoints(int pts) {
    max_points_ = pts > 0 ? pts : MAX_POINTS;
}

/**
 * Sets the number of points that are allowed to be dropped between frames
 * if the input value is not in (0,1], it will use the default value \ref DROPPED_THRESHOLD
 */
void OpticalFlow::setDroppedThreshold(double thresh) {
    dropped_threshold_ = (thresh > 0 && thresh <= 1) ? thresh : DROPPED_THRESHOLD;
}

/**
 * Sets the minimum valid movement, if the input value is negative it
 * will use the default value \ref DEADBAND_M
 */
void OpticalFlow::setDeadbandMeters(double deadband) {
    deadband_ = deadband >= 0 ? deadband : DEADBAND_M;
}

/**
 * Sets the robot radius in meters, if the input value is negative it
 * will use the default value \ref ROBOT_RADIUS
 */
void OpticalFlow::setRobotRadius(double radius) {
    robot_radius_ = radius > 0 ? radius : ROBOT_RADIUS;
}

/**
 * Sets the rotation constant
 */
void OpticalFlow::setRotationConstant(double constant) {
    rotation_constant_ = constant;
}

/**
 * Sets the OpticalFlow::image_topic_
 */
void OpticalFlow::setImageTopic(std::string image_topic) {
    image_topic_ = std::move(image_topic);
    image_subscriber_ = image_transport_.subscribe(image_topic_, 1, &OpticalFlow::imageCallback, this,
                                                   image_transport::TransportHints("compressed"));
}

/**
 * Sets the OpticalFlow::twist_topic_
 */
void OpticalFlow::setTwistTopic(std::string twist_topic) {
    twist_topic_ = std::move(twist_topic);
    twist_publisher_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>(twist_topic_, 1);
}

int OpticalFlow::getFilterSize() const {
    return filter_size_;
}

int OpticalFlow::getMaxTrackedPoints() const {
    return max_points_;
}

double OpticalFlow::getDroppedThreshold() const {
    return dropped_threshold_;
}

double OpticalFlow::getDeadBandMeters() const {
    return deadband_;
}

double OpticalFlow::getRobotRadius() const {
    return robot_radius_;
}

double OpticalFlow::getRotationConstant() const {
    return rotation_constant_;
}

std::string OpticalFlow::getImageTopic() const {
    return image_topic_;
}

std::string OpticalFlow::getTwistTopic() const {
    return twist_topic_;
}

void OpticalFlow::print() {
    std::cout << "#######################" << std::endl;
    std::cout << "Optical Flow Node" << std::endl;
    std::cout << "Image Height (Pixels): " << cameraParameters_.getHeight() << std::endl;
    std::cout << "Image Width (Pixels): " << cameraParameters_.getWidth() << std::endl;
    std::cout << "Image Topic: " << image_topic_ << std::endl;
    std::cout << "TwistWithCovarianceStamped Topic: " << twist_topic_ << std::endl;
    std::cout << "Robot Radius: " << robot_radius_ << std::endl;
    std::cout << "Deadband: " << deadband_ << std::endl;
    std::cout << "Number of Tracked Points: " << max_points_ << std::endl;
    std::cout << "Filter Size: " << filter_size_ << std::endl;
    std::cout << "Dropped threshold: " << dropped_threshold_ << std::endl;
    std::cout << "-----------------------" << std::endl;
    std::cout << "Camera Calibration Matrix: \n";
    std::cout << cameraParameters_.getCameraCalibrationMatrix() << std::endl;
    std::cout << "-----------------------" << std::endl;
    std::cout << "Distortion Coefficients: \n";
    std::cout << cameraParameters_.getDistortionCoefficients() << std::endl;
    std::cout << "-----------------------" << std::endl;
    std::cout << "Rectification Matrix: \n";
    std::cout << cameraParameters_.getRectificationMatrix() << std::endl;
    std::cout << "-----------------------" << std::endl;
    std::cout << "Projection Matrix: \n";
    std::cout << cameraParameters_.getProjectionMatrix() << std::endl;
    std::cout << "-----------------------" << std::endl;
    std::cout << "W Matrix: \n";
    std::cout << W_ << std::endl;
    std::cout << "-----------------------" << std::endl;
    std::cout << "#######################" << std::endl;
}

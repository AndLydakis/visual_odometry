#include <utility>

#ifndef VISUAL_ODOM_OPTICAL_FLOW_CPP
#define VISUAL_ODOM_OPTICAL_FLOW_CPP

#include <optical_flow.h>

#include "../include/optical_flow.h"

#endif


OpticalFlow::OpticalFlow(ros::NodeHandle nh, std::string image_topic, std::string twist_topic, int filter_size,
                         double dropped_threshold, int max_points, double deadband, double robot_radius,
                         double rotation_constant)
        : nh_(nh), image_transport_(nh), cameraParameters_(nh_), image_topic_(std::move(image_topic)),
          twist_topic_(std::move(twist_topic)), filter_size_(filter_size), dropped_threshold_(dropped_threshold),
          max_points_(max_points), deadband_(deadband), robot_radius_(robot_radius),
          rotation_constant_(rotation_constant) {
    try {
        image_subscriber_ = image_transport_.subscribe(image_topic_, 1, &OpticalFlow::imageCallback, this,
                                                       image_transport::TransportHints("compressed"));
        twist_publisher_ = nh_.advertise<geometry_msgs::Twist>(twist_topic_, 1);
        cv::namedWindow(windowName_, CV_WINDOW_AUTOSIZE);
        std::vector<double> wData{0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
        W_ = cv::Mat(3, 3, CV_64F, &wData[0]);
        std::vector<double> HData{0.0002347417933653588, -9.613823951336309e-20, -0.07500000298023225,
                                  -7.422126200315807e-19, -0.0002818370786240783, 0.5159999728202818,
                                  1.683477982667922e-19, 5.30242624981192e-18, 1};
        H_ = cv::Mat(3, 3, CV_64F, &HData[0]);
        filter_.reserve(filter_size_);
        for (int i = 0; i < filter_size_; ++i) {
            filter_[i] = geometry_msgs::Twist();
            filter_[i].linear.x = 0.0;
            filter_[i].linear.y = 0.0;
            filter_[i].linear.z = 0.0;
            filter_[i].angular.x = 0.0;
            filter_[i].angular.y = 0.0;
            filter_[i].angular.z = 0.0;
        }

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

OpticalFlow::~OpticalFlow() {
    cv::destroyAllWindows();
}

void OpticalFlow::imageCallback(const sensor_msgs::ImageConstPtr &image) {
    std::cout << "Enter Image Callback\n";
    try {
        std::cout << "Create grayscale image\n";
        cv::Mat greyscale_image;
        cv::Mat color_image_ = (cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8))->image;
        cv::cvtColor(color_image_, greyscale_image, CV_BGR2GRAY);
        if (!prev_.data) {
            greyscale_image.copyTo(prev_);
            return;
        }
        std::cout << "Calculate new set of points if there are not enough from previous image\n";
        if (prev_track_indices_.size() < dropped_threshold_ * max_points_) {
            cv::goodFeaturesToTrack(prev_, prev_track_indices_, max_points_, 0.1, 5.0);
        }
        if (prev_track_indices_.empty()) {
            curr_.copyTo(prev_);
            return;
        };
        previous_timestamp_ = ros::Time::now().toSec();
        std::cout << "Calculate optical flow\n";
        cv::calcOpticalFlowPyrLK(prev_, curr_, prev_track_indices_, curr_track_indices_, flow_status_, flow_errors_,
                                 cv::Size(21, 21), 4);
        cv::Point3f estimated_motion = feature_motion_estimate(prev_track_indices_, curr_track_indices_);
        accumulated_motion_ += estimated_motion;
        std::cout << "Estimated Motion so Far: " << accumulated_motion_.x << ", Rotation:"
                  << accumulated_motion_.x / (2 * M_PI * robot_radius_) * 360 * rotation_constant_ << std::endl;

//        //Calculate Fundamental Matrix
//        F_ = cv::findFundamentalMat(prev_track_indices_, curr_track_indices_, CV_FM_LMEDS, 1, 0.99, flow_status_);
//        //Calculate Essential Matrix
//        E_ = cameraParameters_.getCameraCalibrationMatrix().t() * F_ * cameraParameters_.getCameraCalibrationMatrix();

//        cv::SVD svd(E_);
//        R_ = svd.u * cv::Mat(W_) * svd.vt;
//        t_ = svd.u.col(2);

        geometry_msgs::Twist twist_msg;
        current_timestamp_ = ros::Time::now().toSec();
        double t_diff = current_timestamp_ - previous_timestamp_;
        twist_msg.linear.x = estimated_motion.y / t_diff;
        twist_msg.angular.z = estimated_motion.x / (2 * M_PI * robot_radius_) * 2 * M_PI * rotation_constant_ / t_diff;

        filter_[filter_counter_] = twist_msg;

        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = 0.0;
        for (int i = 0; i < filter_size_; ++i) {
            twist_msg.linear.x += filter_[i].linear.x / filter_size_;
            twist_msg.linear.y += filter_[i].linear.y / filter_size_;
            twist_msg.angular.x += filter_[i].angular.x / filter_size_;
            twist_msg.angular.y += filter_[i].angular.y / filter_size_;
            twist_msg.angular.z += filter_[i].angular.z / filter_size_;
        }

        twist_publisher_.publish(twist_msg);
        ++filter_counter_ %= filter_size_;

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
        curr_.copyTo(prev_);
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
        ROS_ERROR("ROS error: &s", re.what());
        std::cerr << "ROS error: " << re.what() << std::endl;
        return;
    } catch (const cv::Exception &cve) {
        ROS_ERROR("CV error: &s", cve.what());
        std::cerr << "CV error: " << cve.what() << std::endl;
        return;
    } catch (const cv_bridge::Exception &cvbe) {
        ROS_ERROR("cv_bridge error: &s", cvbe.what());
        std::cerr << "CV error: " << cvbe.what() << std::endl;
        return;
    } catch (...) {
        ROS_ERROR("Something went wrong");
        return;
    }
    std::cout << "Exit Image Callback\n";

}

cv::Point3f
OpticalFlow::feature_motion_estimate(std::vector<cv::Point2f> &previous_coordinates,
                                     std::vector<cv::Point2f> &current_coordinates) {
    std::cout << "Estimating Motion\n";
    std::vector<cv::Point3f> previous_homogeneous_points;
    cv::convertPointsToHomogeneous(previous_coordinates, previous_homogeneous_points);
    auto prev_it = previous_homogeneous_points.begin();
    std::vector<cv::Point3f> current_homogeneous_points;
    cv::convertPointsToHomogeneous(current_coordinates, current_homogeneous_points);
    auto curr_it = current_homogeneous_points.begin();

    cv::Point3f average_motion(0.0, 0.0, 0.0);

    std::cout << "Looping through points\n";
    for (; curr_it != current_homogeneous_points.end(); ++prev_it, ++curr_it) {
        average_motion += cv::Point3f((H_ * cv::Mat(*curr_it) - H_ * cv::Mat(*prev_it)));
        average_motion.x /= previous_homogeneous_points.size();
        average_motion.y /= previous_homogeneous_points.size();
        average_motion.z /= previous_homogeneous_points.size();
    }
    //?
    std::cout << "Adjusting averaged motion\n";
    average_motion.y *= -1.57 * 4.0 / 3.0;
    if ((fabs(average_motion.x) > deadband_) || (fabs(average_motion.y) > deadband_)) return average_motion;
    return cv::Point3f(0.0, 0.0, 0.0);
}

void OpticalFlow::setFilterSize(int fs) {
    filter_size_ = fs > 0 ? fs : FILTER_SIZE;
}

void OpticalFlow::setMaxTrackedPoints(int pts) {
    max_points_ = pts > 0 ? pts : MAX_POINTS;
}

void OpticalFlow::setDroppedThreshold(double thresh) {
    dropped_threshold_ = (thresh > 0 && thresh <= 1) ? thresh : DROPPED_THRESHOLD;
}

void OpticalFlow::setDeadbandMeters(double deadband) {
    deadband_ = deadband >= 0 ? deadband : DEADBAND_M;
}

void OpticalFlow::setRobotRadius(double radius) {
    robot_radius_ = radius > 0 ? radius : ROBOT_RADIUS;
}

void OpticalFlow::setRotationConstant(double constant) {
    rotation_constant_ = constant;
}

void OpticalFlow::setImageTopic(std::string image_topic) {
    image_topic_ = std::move(image_topic);
    image_subscriber_ = image_transport_.subscribe(image_topic_, 1, &OpticalFlow::imageCallback, this,
                                                   image_transport::TransportHints("compressed"));
}

void OpticalFlow::setTwistTopic(std::string twist_topic) {
    twist_topic_ = std::move(twist_topic);
    twist_publisher_ = nh_.advertise<geometry_msgs::Twist>(twist_topic_, 1);
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
    std::cout << "Twist Topic: " << twist_topic_ << std::endl;
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
    std::cout << "#######################" << std::endl;
}

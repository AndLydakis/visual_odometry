//
// Created by Andreas Lydakis on 8/7/18.
//

#ifndef VISUAL_ODOM_CAMERA_PARAMETERS_H
#define VISUAL_ODOM_CAMERA_PARAMETERS_H

/** @name Default Camera Parameters
*  Safe default values for camera parameters.
*/
/**@{*/
///Default Camera Width
#define WIDTH 640
///Default Camera Height
#define HEIGHT 480
/**@}*/

#include <vector>
#include "math.h"

#include <cv_bridge/cv_bridge.h>
#include "opencv2/calib3d/calib3d.hpp"

/**
 * A container class for camera parameters, read from a ROS camera calibration file, loaded
 * with the ROS parameter server
 */
class CameraParameters {
public:
    /**
     * Constructor
     * @param nh the ros::Nodehandle that handles reading from the ROS parameter server
     */
    CameraParameters(ros::NodeHandle nh);

    /**
     * Convert from radians to degrees
     * @param radians An angle in radians
     * @return The value in degrees
     */
    float radiansToDegrees(float radians);

    /**
     * Convert from degrees to radians
     * @param degrees An angle in degrees
     * @return the value in radians
     */
    float degreesToRadians(float degrees);

    /**
     * Normalize an angle in degrees to [- \ref M_PI, \ref M_PI]
     * @param degrees An angle in degrees
     * @return The normalized angle
     */
    float normalizeDegrees(float degrees);

    /**
     * Set the tild of the camera in degrees
     * @param degrees The tilt of the camera in degrees (CameraParameters::tilt_degrees_)
     */
    void setTiltDegrees(float degrees);

    /**
     * Set the tilt of the camera in radians
     * @param radians The tilt of the camera in radians (CameraParameters::tilt_degrees_)
     */
    void setTiltRadians(float radians);

    /**
     * Set the distance of the camera from the ground
     * @param height The height of the camera (meters) (CameraParameters::distance_from_ground_)
     */
    void setDistanceFromGround(float height);

    /**
     * Set the distance of the camera from the center of the platform
     * @param distance Distance from center (meters) (CameraParameters::distance_from_center_)
     */
    void setDistanceFromCenter(float distance);

    /**
     * Set image height
     * @param height The new image height(pixels) (CameraParameters::height_)
     */
    void setHeight(float height);

    /**
     * Set image width
     * @param width The new image width(pixels) (CameraParameters::width_)
     */
    void setWidth(float width);

    /**
     * Set the path of the camera calibration file
     * @todo Not yet implemented (the parameters are read directly from the ROS parameter server)
     * @param calibration_file the path to the camera calibration file
     */
    void setCalibrationFile(std::string calibration_file);

    /**
     * Set the horizontal subtended meters for the camera
     * @todo Not yet implemented or used
     * @param meters the horizontal subtended meters for the camera
     */
    void setSubtensionHorizontal(float meters);

    /**
     * Set the vertical subtended meters for the camera
     * @todo Not yet implemented or used
     * @param meters the vertical subtended meters for the camera
     */
    void setSubtensionVertical(float meters);

    /**
     * Return the distance from the ground for the camer
     * @return The distance from ground (CameraParameters::distance_from_ground_)
     */
    float getDistanceFromGround();

    /**
     * Return the distance from the ground for the camer
     * @return The distance from ground (CameraParameters::distance_from_center_)
     */
    float getDistanceFromCenter();

    /**
     * Return the tilt of the camera
     * @return The tilt of the camera (CameraParameters::tilt_degrees_)
     */
    float getTilt();

    /**
     * Return the image height
     * @return The image height (CameraParameters::height_)
     */
    int getHeight();

    /**
     * Return the image width
     * @return The image width (CameraParameters::width_)
     */
    int getWidth();

    /**
     * Return the horizontal subtended meters
     * @todo not used currently
     * @return The horizontal subtended meters (CameraParameters::subtension_horizontal_m_)
     */
    float getSubtensionHorizontal();

    /**
     * Return the vertical subtended meters
     * @todo not used currently
     * @return The vertical subtended meters (CameraParameters::subtension_vertical_m_)
     */
    float getSubtensionVertical();

    /**
     * Return the path to the camera calibration file
     * @return The path to the camera calibration file (CameraParameters::calibration_file_)
     */
    std::string getCalibrationFile();

    /**
     * Return the camera calibration matrix
     * @return The camera calibration matrix (CameraParameters::calibration_matrix_)
     */
    cv::Matx33d &getCameraCalibrationMatrix();

    /**
     * Return the camera distortion coefficients
     * @return The camera distortion coefficients matrix (CameraParameters::distortion_coefficients_)
     */
    cv::Mat &getDistortionCoefficients();

    /**
     * Return the camera rectification matrix
     * @return The camera rectification matrix (CameraParameters::rectification_matrix_)
     */
    cv::Mat &getRectificationMatrix();

    /**
     * Return the camera projection matrix
     * @return The camera projection matrix (CameraParameters::projection_matrix_)
     */
    cv::Mat &getProjectionMatrix();


protected:
private:
    ///Handles the reading from the parameter server
    ros::NodeHandle nodeHandle_;
    ///Path to file that contains the camera calibration results
    std::string calibration_file_;
    ///Image height(pixels)
    int height_;
    ///Image width(pixels)
    int width_;
    ///Camera distance from the center of the robot (meters)
    float distance_from_center_;
    ///Camera distance from the ground (meters)
    float distance_from_ground_;
    ///Camera tilt(degrees)
    float tilt_degrees_;
    ///Camera vertical subtendension (meters)
    float subtension_vertical_m_;
    ///Camera horizontal subtendension (meters)
    float subtension_horizontal_m_;
    ///3x3 camera calibration matrix
    cv::Matx33d camera_calibration_matrix_;
    ///Camera distortion coefficients
    cv::Mat distortion_coefficients_;
    ///Camera rectification matrix
    cv::Mat rectification_matrix_;
    ///Camera projection matrix
    cv::Mat projection_matrix_;
};

#endif //VISUAL_ODOM_CAMERA_PARAMETERS_H

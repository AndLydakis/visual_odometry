#include <cstdlib>
#include "optical_flow.cpp"
#include "camera_parameters.cpp"


int main(int argc, char **argv) {
    ros::init(argc, argv, "optical_flow_node");
    ros::NodeHandle nh_;
    ros::Rate rate(100);
    OpticalFlow of(nh_);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return EXIT_SUCCESS;
}
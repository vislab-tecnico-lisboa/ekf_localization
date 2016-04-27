#include "kalman.hpp"

#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ekf_localization_node");
    ros::NodeHandle node;
    ros::NodeHandle priv_node("~");
    if(argc != 2)
        std::cout << "No Input Image!" << std::endl;

    double spin_rate;

    priv_node.param("spin",spin_rate, 20.0);

    EKFnode k(node,spin_rate);

    ros::spin();

    return 0;
}

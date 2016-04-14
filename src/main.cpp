#include <iostream>
#include <opencv2/opencv.hpp>

#include "kalman.hpp"

#include "ros/ros.h"

const std::string name = "Kalman";

int main(int argc, char *argv[])
{
    ros::init(argc, argv, name);
    ros::NodeHandle node;
    ros::NodeHandle priv_node("~");
    if(argc != 2)
        std::cout << "No Input Image!" << std::endl;


    //const cv::Mat map = cv::imread(argv[1], CV_LOAD_IMAGE_ANYCOLOR);
    const cv::Mat map = cv::imread(argv[1]);
    std::cout << argv[1]<< std::endl;
    const int spin_rate = 20;

    cv::namedWindow(name);
    double init_x, init_y, init_theta;
    priv_node.param("init_x", init_x, 0.0);
    priv_node.param("init_y", init_y, 0.0);
    priv_node.param("init_theta", init_theta, 0.0);

    kalman k(node,map,init_x,init_y,init_theta,spin_rate);

    ros::Rate rate(spin_rate);
    while(ros::ok())
    {
        ros::spinOnce();
        k.spin();
        rate.sleep();
    }
    ros::shutdown();

    return 0;
}

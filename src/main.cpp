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


    const cv::Mat map = cv::imread(argv[1], CV_LOAD_IMAGE_ANYCOLOR);
    std::cout << argv[1]<< std::endl;
    const int spin_rate = 10;

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
        cv::Vec3d X= k.getX();
        cv::Matx<double,3,3> Q=k.getP();
        Eigen::Vector2f mean;
        mean[0]=X[0];
        mean[1]=X[1];

        Eigen::Matrix2f covMatrix;
        covMatrix(0,0)=Q(0,0);
        covMatrix(0,1)=Q(0,1);

        covMatrix(1,0)=Q(1,0);
        covMatrix(1,1)=Q(1,1);
        k.drawCovariance(mean,covMatrix);
        //k.correct();
        //k.show_map(name,true);
        k.broadcast();
        rate.sleep();
    }
    ros::shutdown();

    return 0;
}

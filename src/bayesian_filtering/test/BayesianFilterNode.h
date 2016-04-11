#ifndef BAYESIANFILTERNODE_H
#define BAYESIANFILTERNODE_H
#include <filter/extendedkalmanfilter.h>
#include "nonlinearanalyticconditionalgaussianmobile.h"
#include <model/analyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>

#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <pose_cov_ops/pose_cov_ops.h>
#include <sensor_msgs/LaserScan.h>

// pcl stuff
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include "geometry_helpers.h"
#include <visualization_msgs/Marker.h>

#include <opencv2/core/core.hpp>

// Sizes
#define STATE_SIZE 3 //state: x,y,theta
#define MEAS_SIZE 2  // measurment: 3D points

class BayesianFilterNode
{
    // ROS STUFF
    typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;
    typedef sensor_msgs::LaserScanConstPtr LaserConstPtr;
    ros::Time last_update_time;
    ros::NodeHandle nh_, private_node_handle_;
    ros::Subscriber laser_sub;
    ros::Publisher laser_features_pub;
    ros::Publisher location_undertainty;
    boost::shared_ptr<tf::TransformListener> listener;

    // KALMAN VARS
    boost::shared_ptr<BFL::ExtendedKalmanFilter> filter;
    boost::shared_ptr<BFL::LinearAnalyticConditionalGaussian> sys_pdf;
    boost::shared_ptr<BFL::LinearAnalyticSystemModelGaussianUncertainty>  sys_model;
    boost::shared_ptr<BFL::LinearAnalyticConditionalGaussian> meas_pdf;
    boost::shared_ptr<BFL::LinearAnalyticMeasurementModelGaussianUncertainty> meas_model;
    boost::shared_ptr<BFL::Gaussian> prior_cov;

    double dt;

    bool odom_initialized;

    std::string base_link;
    std::string odom_link;

    cv::Mat getFeatures(std::vector<cv::Point2f> & points)
    {};


    // Global features
    std::vector<cv::Point2f> global_features;
public:
    BayesianFilterNode(const ros::NodeHandle &nh_);

    ~BayesianFilterNode();
    void createFilter(const double & prior_mean,
                      const double & prior_std_dev);

    void laserCallback(const LaserConstPtr & msg);

    void drawCovariance(const Eigen::Vector2f& mean, const Eigen::Matrix2f& covMatrix);
};

#endif // BAYESIANFILTERNODE_H

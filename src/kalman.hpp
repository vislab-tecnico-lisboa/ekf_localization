// BY Rui Pimentel de Figueiredo

#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/random.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/nondet_random.hpp>
#include <boost/random/normal_distribution.hpp>

#include <Eigen/Core>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/Marker.h>
#include "featuresextractor.h"

// Point cloud library used for scan matching
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/publisher.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

// Bayesian filtering library
#include <filter/extendedkalmanfilter.h>
#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>

#include "bayesian_filtering/nonlinearanalyticconditionalgaussianmobile.h"
#include <laser_geometry/laser_geometry.h>


class EKFnode
{
    typedef pcl::PointXYZ point_type;
    typedef pcl::PointCloud<point_type> map_t;
    typedef map_t::Ptr map_t_ptr;

    // ROS stuff
    ros::NodeHandle nh_priv;
    ros::NodeHandle nh_;

    tf::Transformer transformer_;
    ros::Time odom_last_stamp_, laser_last_stamp_, odom_init_stamp_,laser_init_stamp_, filter_stamp_;

    boost::shared_ptr<tf::TransformListener> listener;
    std::string base_link;
    std::string odom_link;
    std::string map_link;
    std::string laser_link;
    bool use_map_topic_, first_map_only_,first_map_received_;
    ros::Subscriber laser_sub;
    ros::Subscriber map_sub_;
    ros::Publisher location_undertainty;
    ros::Publisher map_pub_;
    ros::Publisher local_features_pub;
    ros::Timer timer_;


    tf::TransformBroadcaster tf_broadcaster;
    pcl::PointCloud<point_type>::Ptr laser;
    laser_geometry::LaserProjection projector_;
    boost::shared_ptr<BFL::ExtendedKalmanFilter> filter;
    boost::shared_ptr<BFL::NonLinearAnalyticConditionalGaussianMobile> sys_pdf;
    boost::shared_ptr<BFL::AnalyticSystemModelGaussianUncertainty> sys_model;
    boost::shared_ptr<BFL::LinearAnalyticConditionalGaussian> meas_pdf;
    boost::shared_ptr<BFL::LinearAnalyticMeasurementModelGaussianUncertainty> meas_model;

    pcl::PointCloud<point_type>::Ptr map_;

    double alpha_1, alpha_2, alpha_3, alpha_4;
    double d_thresh_, a_thresh_;
    double voxel_grid_size;

    //paramater to store latest odom pose
    tf::Stamped<tf::Pose> latest_odom_pose_;
    tf::Stamped<tf::Pose> filter_odom_pose_;
    // ICP and vision stuff
    FeaturesExtractor features_extractor;
    double max_correspondence_distance;
    int max_iterations;
    int ransac_iterations;
    double ransac_outlier_threshold;
    double icp_optimization_epsilon;
    double icp_score_scale;
    double covariance_marker_scale_;
    // aux vars
    bool odom_active_, laser_active_, odom_initialized_,laser_initialized_;

    BFL::ColumnVector last_laser_pose_;
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void sendTransform(const tf::Transform & transform_, const ros::Time & time_stamp_, const std::string & target_frame_, const std::string & origin_frame_);
    void drawCovariance(const Eigen::Matrix2f& covMatrix);
    void drawFeatures();
    void requestMap();
    void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);
    void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
    void convertMap( const nav_msgs::OccupancyGrid& map_msg );

    void broadcast(const ros::Time & broad_cast_time);
    bool predict();

    void angleOverflowCorrect(double& a)
    {
        while ((a) >  M_PI) a -= 2*M_PI;
        while ((a) < -M_PI) a += 2*M_PI;
    }

    void publishFeatures()
    {
        map_pub_.publish(map_);
    }
    enum {OCCUPIED = 0, FREE = 255};
    const static int CV_TYPE = CV_64F;

public:

    EKFnode(const ros::NodeHandle& nh, const double & spin_rate, const double & voxel_grid_size_=0.005);

    void spin(const ros::TimerEvent& e);
};


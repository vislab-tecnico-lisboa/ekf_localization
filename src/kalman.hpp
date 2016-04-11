#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/random.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/nondet_random.hpp>
#include <boost/random/normal_distribution.hpp>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/tf.h>
#include "tf/transform_broadcaster.h"
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <visualization_msgs/Marker.h>
//#include <tf/LinearMath/btMatrix3x3.h>
//#include <tf/LinearMath/btQuaternion.h>

struct rangle
{
	double range, angle;
	rangle(double r, double a) : range(r), angle(a) {}
};

class kalman
{
    std::string base_link;
    std::string odom_link;
    std::string map_link;
	const cv::Mat map;
	const double dt;
	double gt_x, gt_y, gt_theta;
	double linear, angular;
    ros::Subscriber cmd_sub;
    ros::Subscriber laser_sub;
    ros::Subscriber bpgt_sub;
    ros::Publisher location_undertainty;
    tf::TransformBroadcaster tf_broadcaster;
	std::vector<rangle> laser;

    cv::Vec3d X;           //!< predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)
    cv::Matx<double,3,3> F;   				//!< state transition matrix (F)
    cv::Matx<double,3,3> I;   				//!< state transition matrix (F)
    cv::Matx<double,3,3> H;  				//!< measurement matrix (H)
    cv::Matx<double,3,3> Q;					//!< process noise covariance matrix (Q)
    //cv::Mat measurementNoiseCov;//!< measurement noise covariance matrix (R)
    cv::Matx<double,3,3> K;               //!< Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
	cv::Matx<double,3,3> P;

public:
    cv::Vec3d getX()
    {
        return X;
    }

    cv::Matx<double,3,3> getP()
    {
        return P;
    }

    void drawCovariance(const Eigen::Vector2f& mean, const Eigen::Matrix2f& covMatrix);
	kalman(ros::NodeHandle& nh, const cv::Mat& pmap, double x_init, double y_init, double theta_init, int spin_rate);
    void broadcast();

	void pose_callback(const nav_msgs::Odometry msg);
	void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

	double ray_trace(const double x, const double y, const double theta, const double angle) const;

	void predict(const nav_msgs::Odometry msg);
	void correct();

    cv::Point2d toStage(cv::Point2i p) const;
    cv::Point2i toImage(cv::Point2d p) const;

	cv::Mat show_map(const std::string& win_name, bool draw) const;

    enum {OCCUPIED = 0, FREE = 255};
	const static int CV_TYPE = CV_64F;
};


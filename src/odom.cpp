#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <pose_cov_ops/pose_cov_ops.h>
#include <sensor_msgs/LaserScan.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "BayesianFilterNode");
    ros::NodeHandle nh;
    double rate=50.0;
    ros::Rate r(rate);
    std::string odom_link="odom";
    std::string base_link="base_link";

    ros::Time last_update_time=ros::Time::now();
    tf::TransformListener listener;

    ros::Publisher odom_publisher=nh.advertise<nav_msgs::Odometry>("/odom",2);

    while(ros::ok())
    {
        tf::StampedTransform baseDeltaTf;
        tf::StampedTransform odomTf;

        // Get delta motion in cartesian coordinates with TF
        ros::Time current_time;

        try
        {
            current_time = ros::Time::now();
            listener.waitForTransform(base_link, last_update_time, base_link, current_time, odom_link, ros::Duration(0.1) );
            listener.lookupTransform(base_link, last_update_time, base_link, current_time, odom_link, baseDeltaTf); // Velocity

            listener.waitForTransform(base_link, odom_link, ros::Time(0), ros::Duration(0.1) );
            listener.lookupTransform(base_link, odom_link, ros::Time(0), odomTf); // Position
        }
        catch (tf::TransformException &ex)
        {
            //ROS_WARN("%s",ex.what());
            continue;
        }

        double dt=current_time.toSec()-last_update_time.toSec();
        last_update_time=current_time;

        double delta_yaw=baseDeltaTf.getRotation().getAngle();
        double delta_x=baseDeltaTf.getOrigin().getX();

        double v_yaw=delta_yaw/dt;
        double v_x=delta_x/dt;

        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp=ros::Time::now();
        odom_msg.pose.pose.position.x=odomTf.getOrigin().getX();
        odom_msg.pose.pose.position.y=odomTf.getOrigin().getY();
        odom_msg.pose.pose.position.z=odomTf.getOrigin().getZ();
        odom_msg.pose.pose.orientation.w=odomTf.getRotation().getW();
        odom_msg.pose.pose.orientation.x=odomTf.getRotation().getX();
        odom_msg.pose.pose.orientation.y=odomTf.getRotation().getY();
        odom_msg.pose.pose.orientation.z=odomTf.getRotation().getZ();

        odom_msg.twist.twist.angular.z=v_yaw;
        odom_msg.twist.twist.linear.x=v_x;
        odom_publisher.publish(odom_msg);
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}

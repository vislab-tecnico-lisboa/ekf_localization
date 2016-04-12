#include "kalman.hpp"

kalman::kalman(ros::NodeHandle& nh, const cv::Mat& pmap, double x_init, double y_init, double theta_init, int spin_rate)
    : map(pmap.clone()), dt(1.0/(double)spin_rate), linear(0), angular(0),
      listener(new tf::TransformListener(ros::Duration(10.0)))

{

    this->cmd_sub = nh.subscribe("odom", 1, &kalman::predict, this);
    this->laser_sub = nh.subscribe("scan", 1, &kalman::laser_callback, this);
    this->bpgt_sub = nh.subscribe("base_pose_ground_truth", 1, &kalman::pose_callback, this);
    this->location_undertainty = nh.advertise<visualization_msgs::Marker>("/location_undertainty",1);
    this->map_features_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_features",1);
    this->local_features_pub = nh.advertise<sensor_msgs::PointCloud2>("/local_features",1);

    base_link="/base_link";
    odom_link="/odom";
    map_link="/map";
    laser_link="/laser_frame";

    this->X[0] = x_init;
    this->X[1] = y_init;
    this->X[2] = theta_init;

    this->F(0,0) = 1;
    this->F(1,1) = 1;
    this->F(2,2) = 1;

    this->I(0,0) = 1;
    this->I(1,1) = 1;
    this->I(2,2) = 1;

    this->P(0,0) = 1;
    this->P(1,1) = 1;
    this->P(2,2) = 1;

    this->Q(0,0) = 1/10000.0;
    this->Q(1,1) = 1/10000.0;
    this->Q(2,2) = 1/10000.0;

    //Bound image by occupied cells.
    std::cout << this->map.size() << std::endl;
    this->map.row(0) = cv::Scalar(0);

    this->map.row(map.size().width - 1) = cv::Scalar(0);
    this->map.col(0) = cv::Scalar(0);
    this->map.col(map.size().height - 1)  = cv::Scalar(0);

    std::vector<cv::Point2f> map_features_cv=features_extractor.mapFeatures(this->map);
    map_features.resize(map_features_cv.size());
    map_features.header.frame_id="map";
    pcl::PointXYZRGB point=pcl::PointXYZRGB(255, 0, 0);

    double map_scale=0.05;
    for(unsigned int i=0; i<map_features_cv.size();++i)
    {
        point.x=map_scale*map_features_cv[i].x;
        point.y=-map_scale*map_features_cv[i].y;
        point.z=0.0;
        map_features[i]=point;
    }
}


void kalman::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    //const int size = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    laser.resize(msg->ranges.size());
    laser.header.frame_id=laser_link;
    pcl_conversions::toPCL(ros::Time::now(), laser.header.stamp);

    pcl::PointXYZRGB point=pcl::PointXYZRGB(0, 0, 255);
    double step=msg->angle_increment;

    for(int i=0; i< msg->ranges.size();++i)
    {
        double angle_=msg->angle_min+(double)step*i;
        point.x = msg->ranges[i]*cos(angle_);
        point.y = msg->ranges[i]*sin(angle_);
        point.z = 0.0;
        laser[i]=point;
    }

    Eigen::Matrix4f laserToBaseEigen;
    tf::StampedTransform laserToBaseTf;
    ros::Time current_time = ros::Time(0);

    try
    {

        listener->waitForTransform(map_link, laser_link, current_time, ros::Duration(1.0) );
        listener->lookupTransform(map_link, laser_link, current_time, laserToBaseTf); // ABSOLUTE EGO TO WORLD
    }
    catch(tf::TransformException& ex)
    {
        //ROS_ERROR_STREAM( "Transform error: " << ex.what() << ", quitting callback");
        return;
    }

    pcl_ros::transformAsMatrix(laserToBaseTf, laserToBaseEigen);

    pcl::PointCloud<pcl::PointXYZRGB> laser_out;
    pcl::transformPointCloud (laser, laser_out, laserToBaseEigen);
    laser_out.is_dense=false;
    laser_out.header.frame_id=map_link;
    pcl_conversions::toPCL(ros::Time::now(), laser_out.header.stamp);

    local_features_pub.publish(laser_out);
    return;
}

void kalman::predict(const nav_msgs::Odometry msg)
{
    const double scale = 0.0;

    static boost::mt19937 rng;
    static boost::normal_distribution<double> x_noise;
    static boost::normal_distribution<double> y_noise;
    static boost::normal_distribution<double> theta_noise;

    static boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > dx(rng,x_noise);
    static boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > dy(rng,y_noise);
    static boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > dtheta(rng,theta_noise);

    this->X[0] += linear * dt * cos( X[2] ) + dx() * scale;
    this->X[1] += linear * dt * sin( X[2] ) + dy() * scale;
    this->X[2] += angular * dt + dtheta() * scale;

    std::cout << "X" << cv::Point3d(X) << std::endl;

    this->F(0,2) = -linear * dt * sin( X[2] ); //t+1 ?
    this->F(1,2) =  linear * dt * cos( X[2] ); //t+1 ?

    std::cout << "F" << std::endl << cv::Mat(F) << std::endl;
    //	std::cout << "P" << std::endl << cv::Mat(P) << std::endl;
    P = F * P * F.t() + Q;
    P = (P + P.t()) * 0.5;

    //	std::cout << "P" << std::endl << cv::Mat(P) << std::endl;

    this->linear = msg.twist.twist.linear.x;
    this->angular = msg.twist.twist.angular.z;

    return;
}

void kalman::correct()
{
    const double x = this->X[0];
    const double y = this->X[1];
    const double theta = this->X[2];
    const double dx = 1.5;
    const double dy = 1.5;
    const double dtheta = 1.5;

    cv::Vec3d res;

    /*for(size_t i = 0; i < laser.size(); i++)
    {
        const double angle = laser[i].angle;
        res[i] = laser[i].range - ray_trace(x,y,theta, angle);
        this->H(i,0) = (ray_trace(x+dx,y,theta,angle) - ray_trace(x-dx,y,theta,angle) ) / 2*dx;
        this->H(i,1) = (ray_trace(x,y+dy,theta,angle) - ray_trace(x,y-dy,theta,angle) ) / 2*dy;
        this->H(i,2) = (ray_trace(x,y,theta+dtheta,angle) - ray_trace(x,y,theta-dtheta,angle) ) / 2*dtheta;
    }*/
    std::cout << std::endl << "res" << cv::Point3d(res) << std::endl;
    //	std::cout << std::endl << "H" << cv::Mat(H) << std::endl;

    cv::Matx<double,3,3> S = H * P * H.t();  // + R
    cv::Matx<double,3,3> K = P * H.t() * S.inv();

    std::cout << std::endl << "K" << cv::Mat(K) << std::endl;

    X += K*res;

    std::cout << "X" << cv::Point3d(X) << std::endl;

    P = (I - K * H) * P;
    P = (P + P.t()) * 0.5;
    std::cout << "P" << std::endl << cv::Mat(P) << std::endl;

}

void kalman::broadcast()
{
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(this->X[0], this->X[1], 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, this->X[2]);
    transform.setRotation(q);
    tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "map"));
}

double kalman::ray_trace(const double x, const double y, const double theta, const double angle) const
{

    //const double y, const double theta, const double range, const double angle){
    //std::cout << "Range: " << range << " Angle: " << angle << std::endl;
    const double range = 4.0;

    const cv::Point2i robot = this->toImage( cv::Point2d(x, y) ); //Get Robot's Esitimated position.

    const double laser_x = range * cos(theta + angle);
    const double laser_y = range * sin(theta + angle);

    const cv::Point2d stage_exp(x + laser_x, y + laser_y);
    const cv::Point2i map_expected = this->toImage(stage_exp);

    cv::LineIterator lit(this->map, robot, map_expected);
    if(lit.count == -1 || lit.step == 0)
    {
        std::cout << "LINE ERROR";
        return range;
    }

    cv::Point2d actual;
    while(true)
    { //Follow line until hit occupied cell. Bounded the image in ctor so can't loop forever.
        if(*(*lit) == OCCUPIED)
        {
            actual = this->toStage( lit.pos() ); // Difference between estimated and expected.
            break;
        }
        lit++;
    }
    const double dx = (x - actual.x);
    const double dy = (y - actual.y);
    return sqrt(dx*dx + dy*dy);
}


cv::Point2d kalman::toStage(cv::Point2i p) const
{
    const double x_ratio = 50.0/map.size().width;
    const double y_ratio = 50.0/map.size().height;
    double x = p.x*x_ratio -25;
    double y = -(p.y - map.size().height)*y_ratio -25;
    return cv::Point2d(x,y);
}

cv::Point2i kalman::toImage(cv::Point2d p) const
{
    const double x_ratio = map.size().width/50.0;
    const double y_ratio = map.size().height/50.0;
    double x = (p.x + 25)*x_ratio;
    double y = map.size().height - (p.y + 25 )*y_ratio;
    return cv::Point2i (x,y);
}

/*cv::Mat kalman::show_map(const std::string& win_name, bool draw) const {
    cv::Mat clone = this->map.clone();

    const cv::Point2d p(X[0], X[1]);
    cv::circle(clone, this->toImage(cv::Point2d(gt_x,gt_y)), 5, 155, -1);
    cv::circle(clone, this->toImage(p), 5, 10, -1);

    if(draw)
    {
        cv::imshow(win_name, clone);
        cv::waitKey(10);
    }
    return clone;
}*/

void kalman::drawCovariance(const Eigen::Vector2f& mean, const Eigen::Matrix2f& covMatrix)
{
    visualization_msgs::Marker tempMarker;
    tempMarker.pose.position.x = 0;
    tempMarker.pose.position.y = 0;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(covMatrix);

    const Eigen::Vector2f& eigValues (eig.eigenvalues());
    const Eigen::Matrix2f& eigVectors (eig.eigenvectors());

    float angle = (atan2(eigVectors(1, 0), eigVectors(0, 0)));


    tempMarker.type = visualization_msgs::Marker::SPHERE;

    double lengthMajor = sqrt(eigValues[0]);
    double lengthMinor = sqrt(eigValues[1]);

    tempMarker.scale.x = 3*lengthMajor;
    tempMarker.scale.y = 3*lengthMinor;
    tempMarker.scale.z = 0.001;

    tempMarker.color.a = 1.0;
    tempMarker.color.r = 1.0;

    tempMarker.pose.orientation.w = cos(angle*0.5);
    tempMarker.pose.orientation.z = sin(angle*0.5);

    tempMarker.header.frame_id=base_link;
    tempMarker.id = 0;

    location_undertainty.publish(tempMarker);
}


void kalman::pose_callback(const nav_msgs::Odometry msg)
{
    this->gt_x = -msg.pose.pose.position.y;
    this->gt_y = msg.pose.pose.position.x;

    double roll, pitch, heading;

    tf::Quaternion q (msg.pose.pose.orientation.x,
                      msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z,
                      msg.pose.pose.orientation.w);

    tf::Matrix3x3(q).getRPY(roll, pitch, heading);

    if(heading < -M_PI/2)
        heading += 5*M_PI/2;
    else
        heading += M_PI/2;

    this->gt_theta = heading;
}

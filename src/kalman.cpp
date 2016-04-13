#include "kalman.hpp"

kalman::kalman(ros::NodeHandle& nh, const cv::Mat& pmap, double x_init, double y_init, double theta_init, int spin_rate)
    : map(pmap.clone()), dt(1.0/(double)spin_rate), linear(0), angular(0),
      listener(new tf::TransformListener(ros::Duration(10.0))),
      map_features(new pcl::PointCloud<point_type>()),
      laser(new pcl::PointCloud<point_type>())


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

    // odometry cov
    this->Q(0,0) = 0.1;
    this->Q(1,1) = 0.1;
    this->Q(2,2) = 0.1;

    // obs model
    this->H(0,0) = 1;
    this->H(1,1) = 1;
    this->H(2,2) = 1;
    // laser cov
    this->R(0,0) = 0.01;
    this->R(1,1) = 0.01;
    this->R(2,2) = 0.01;
    //Bound image by occupied cells.
    //std::cout << this->map.size() << std::endl;
    this->map.row(0) = cv::Scalar(0);

    this->map.row(map.size().width - 1) = cv::Scalar(0);
    this->map.col(0) = cv::Scalar(0);
    this->map.col(map.size().height - 1)  = cv::Scalar(0);

    std::vector<cv::Point2f> map_features_cv=features_extractor.mapFeatures(this->map);
    map_features->resize(map_features_cv.size());
    map_features->header.frame_id="map";
    point_type point;//=point_type(255, 0, 0);

    double map_scale=0.05;
    for(unsigned int i=0; i<map_features_cv.size();++i)
    {
        point.x=map_scale*map_features_cv[i].x;
        point.y=-map_scale*map_features_cv[i].y;
        point.z=0.0;
        (*map_features)[i]=point;
    }


    tf::StampedTransform laserToBaseTf;
    // Transform laser to base frame
    while(ros::ok())
    {
        try
        {
            listener->waitForTransform(base_link, laser_link, ros::Time(0), ros::Duration(1.0) );
            listener->lookupTransform(base_link, laser_link, ros::Time(0), laserToBaseTf); // ABSOLUTE EGO TO WORLD
        }
        catch(tf::TransformException& ex)
        {
            //ROS_ERROR_STREAM( "Transform error: " << ex.what() << ", quitting callback");
            continue;
        }
        break;
    }
    pcl_ros::transformAsMatrix(laserToBaseTf, laserToBaseEigen);

}


void kalman::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_time=msg->header.stamp;
    // subtracting base to odom from map to base and send map to odom instead

    //const int size = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    laser->resize(msg->ranges.size());
    laser->header.frame_id=laser_link;
    laser->is_dense=false;
    pcl_conversions::toPCL(ros::Time::now(), laser->header.stamp);

    point_type point;//=point_type(0, 0, 255);
    double step=msg->angle_increment;

    for(int i=0; i< msg->ranges.size();++i)
    {
        double angle_=msg->angle_min+(double)step*i;
        point.x = msg->ranges[i]*cos(angle_);
        point.y = msg->ranges[i]*sin(angle_);
        point.z = 0.0;
        point.intensity=1.0;
        (*laser)[i]=point;
    }
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    pcl::removeNaNFromPointCloud((*laser), *laser,inliers->indices);
    tf::StampedTransform laserToMapTf;

    // Transform to laser scan to map frame
    try
    {

        listener->waitForTransform(map_link, laser_link, msg->header.stamp, ros::Duration(1.0) );
        listener->lookupTransform(map_link, laser_link, msg->header.stamp, laserToMapTf); // ABSOLUTE EGO TO WORLD
    }
    catch(tf::TransformException& ex)
    {
        return;
    }

    // Downsample
    double voxel_grid_size=0.60;
    pcl::VoxelGrid<point_type> voxel_grid;
    voxel_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
    voxel_grid.setInputCloud (laser);
    voxel_grid.filter (*laser);
    voxel_grid.setInputCloud (map_features);
    voxel_grid.filter (*map_features);

    pcl_ros::transformAsMatrix(laserToMapTf, laserToMapEigen);
    pcl::PointCloud<point_type>::Ptr laser_map (new pcl::PointCloud<point_type>());
    pcl::transformPointCloud (*laser, *laser_map, laserToMapEigen);
    laser_map->is_dense=false;
    laser_map->header.frame_id=map_link;
    pcl_conversions::toPCL(ros::Time::now(), laser_map->header.stamp);

    // Compute ICP
    pcl::IterativeClosestPointNonLinear<point_type, point_type> icp;
    icp.setInputSource(laser_map);
    icp.setInputTarget(map_features);
    //icp.setTransformationEpsilon (1e-6);
    icp.setMaxCorrespondenceDistance (10.0);
    //icp.setMaximumIterations(50);
    icp.setRANSACIterations (50);

    pcl::PointCloud<point_type> Final;
    icp.align(Final);

    Final.header.frame_id=map_link;

    //R(0,0)=icp.getFitnessScore();
    //R(1,1)=icp.getFitnessScore();
    //R(2,2)=icp.getFitnessScore();
    Eigen::Matrix4f correction_transform_map_frame=icp.getFinalTransformation();
    Eigen::Matrix4f correction_transform_=correction_transform_map_frame.inverse();

    std::cout << correction_transform_ << std::endl;
    cv::Vec3d obs;
    obs[0]=correction_transform_(0,3);
    obs[1]=correction_transform_(1,3);
    obs[2]=correction_transform_.block<3,3>(0,0).eulerAngles (0,1,2)(2);
    angleOverflowCorrect(obs[2]);

    correct(obs);
    std::cout << "obs:"<< obs << std::endl;
    pcl::transformPointCloud (*laser_map, *laser_map, correction_transform_);

    local_features_pub.publish(laser_map);
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

    //std::cout << "X" << cv::Point3d(X) << std::endl;

    this->F(0,2) = -linear * dt * sin( X[2] ); //t+1 ?
    this->F(1,2) =  linear * dt * cos( X[2] ); //t+1 ?

    //std::cout << "F" << std::endl << cv::Mat(F) << std::endl;
    //	std::cout << "P" << std::endl << cv::Mat(P) << std::endl;
    P = F * P * F.t() + Q;
    P = (P + P.t()) * 0.5;

    //	std::cout << "P" << std::endl << cv::Mat(P) << std::endl;

    this->linear = msg.twist.twist.linear.x;
    this->angular = msg.twist.twist.angular.z;

    return;
}

void kalman::correct(const cv::Vec3d & obs)
{

    //std::cout << "obs:"<< obs<< std::endl;
    //std::cout << "state:"<< X<< std::endl;

    cv::Vec3d res;
    res = obs;
    std::cout << "res:"<< res<< std::endl;

    cv::Matx<double,3,3> S = H * P * H.t() + R;
    cv::Matx<double,3,3> K = P * H.t() * S.inv();

    //std::cout << std::endl << "K" << cv::Mat(K) << std::endl;

    X += K*res;

    //std::cout << "X" << cv::Point3d(X) << std::endl;

    P = (I - K * H) * P;
    P = (P + P.t()) * 0.5;
    //std::cout << "P" << std::endl << cv::Mat(P) << std::endl;

}

void kalman::broadcast()
{
    tf::Stamped<tf::Pose> odom_to_map;
    try
    {
        tf::Transform tmp_tf(tf::createQuaternionFromYaw(this->X[2]),
                tf::Vector3(this->X[0],
                this->X[1],
                0.0));
        tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(),
                                              laser_time,
                                              base_link);
        this->listener->transformPose(odom_link,
                                      tmp_tf_stamped,
                                      odom_to_map);
    }
    catch(tf::TransformException)
    {
        ROS_DEBUG("Failed to subtract base to odom transform");
        return;
    }

    latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                               tf::Point(odom_to_map.getOrigin()));

    // We want to send a transform that is good up until a
    // tolerance time so that odom can be used
    ros::Time transform_expiration = (laser_time);
    tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                        ros::Time::now(),
                                        map_link, odom_link);
    tf_broadcaster.sendTransform(tmp_tf_stamped);

    std::cout<< "broadcasting"<< std::endl;
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

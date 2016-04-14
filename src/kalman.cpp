#include "kalman.hpp"

kalman::kalman(ros::NodeHandle& nh, const cv::Mat& pmap, double x_init, double y_init, double theta_init, int spin_rate, double voxel_grid_size_)
    : map(pmap.clone()),
      linear(0),
      angular(0),
      listener(new tf::TransformListener(ros::Duration(10.0))),
      map_features(new pcl::PointCloud<point_type>()),
      map_features_aux(new pcl::PointCloud<point_type>()),
      laser(new pcl::PointCloud<point_type>()),
      voxel_grid_size(voxel_grid_size_),
      odom_active_(false),
      laser_active_(false),
      odom_initializing_(false),
      laser_initializing_(false)
{

    //this->cmd_sub = nh.subscribe("odom", 1, &kalman::odometry_callback, this);
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


    this->X_acum[0] = x_init;
    this->X_acum[1] = y_init;
    this->X_acum[2] = theta_init;

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

    //this->Q(0,1) = 0.00001;
    //this->Q(1,0) = 0.00001;

    this->Q(0,2) = 0.05;
    this->Q(1,2) = 0.05;

    this->Q(2,0) = 0.05;
    this->Q(2,1) = 0.05;

    this->Q(2,2) = 10.0;

    // obs model
    this->H(0,0) = 1;
    this->H(1,1) = 1;
    this->H(2,2) = 1;
    // laser cov
    this->R(0,0) = 0.1;
    this->R(1,1) = 0.1;
    this->R(2,2) = 0.1;

    //Bound image by occupied cells.
    this->map.row(0) = cv::Scalar(0);
    this->map.row(map.size().width - 1) = cv::Scalar(0);
    this->map.col(0) = cv::Scalar(0);
    this->map.col(map.size().height - 1)  = cv::Scalar(0);

    std::vector<cv::Point2f> map_features_cv=features_extractor.mapFeatures(this->map);
    map_features_aux->resize(map_features_cv.size());
    map_features->header.frame_id="map";
    point_type point;//=point_type(255, 0, 0);

    double map_scale=0.05;
    /*for(unsigned int i=0; i<map_features_cv.size();++i)
    {
        point.x=map_scale*map_features_cv[i].x;
        point.y=-map_scale*map_features_cv[i].y;
        point.z=0.0;
        point.intensity=1.0;
        (*map_features_aux)[i]=point;
    }*/
    //sleep(5.0); //GIVE TIME TO TF
    for(unsigned int i=0; i<map_features_cv.size();++i)
    {
        point.x=map_scale*map_features_cv[i].x;
        point.y=-map_scale*map_features_cv[i].y;
        point.z=0.0;
        point.intensity=1.0;
        map_features->push_back(point);
    }

    //    pcl::VoxelGrid<point_type> voxel_grid;
    //    voxel_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
    //    voxel_grid.setInputCloud (map_features);
    //    voxel_grid.filter (*map_features);

    // Get Transform from laser to base frame

    tf::StampedTransform laserToBaseTf;
    while(ros::ok())
    {
        try
        {
            listener->waitForTransform(base_link, laser_link, ros::Time(0), ros::Duration(0.05) );
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

    // initialize
    filter_stamp_ = ros::Time::now();
    filter_stamp_old_ = ros::Time::now();
}


void kalman::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

    laser_stamp_=msg->header.stamp;
    filter_stamp_=laser_stamp_;
    if (!laser_active_)
    {
        if (!laser_initializing_)
        {
            laser_initializing_ = true;
            odom_init_stamp_ = odom_stamp_;
            ROS_INFO("Initializing Odom sensor");

        }
        if ( filter_stamp_ >= odom_init_stamp_)
        {
            laser_active_ = true;
            laser_initializing_ = false;
            filter_stamp_old_=filter_stamp_;
            ROS_INFO("Odom sensor activated");
            return;
        }
        else
        {
            ROS_ERROR("Waiting to activate Laser, because Laser measurements are still %f sec in the future.",
                      (odom_init_stamp_ - filter_stamp_).toSec());
        }
    }

    tf::StampedTransform laserToMapTf;
    try
    {
        listener->waitForTransform(map_link, laser_link, filter_stamp_old_, ros::Duration(0.05) );
        listener->lookupTransform(map_link, laser_link, filter_stamp_old_, laserToMapTf);
    }
    catch(tf::TransformException& ex)
    {
        ROS_WARN("%s",ex.what());
    }

    //std::cout << "before predicting:"<< laserToMapTf.getOrigin().getX() << " "  <<laserToMapTf.getOrigin().getY() << " " << laserToMapTf.getRotation().getAngle() << std::endl;

    if(!predict())
    {
        std::cout << "ups"<< std::endl;
        return;
    }

    //return;
    // subtracting base to odom from map to base and send map to odom instead

    //const int size = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    laser->resize(msg->ranges.size());
    laser->header.frame_id=laser_link;
    laser->is_dense=false;
    pcl_conversions::toPCL(filter_stamp_, laser->header.stamp);

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

    // Remove NAN
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::removeNaNFromPointCloud((*laser), *laser,inliers->indices);

    // Downsample
    pcl::VoxelGrid<point_type> voxel_grid;
    voxel_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
    voxel_grid.setInputCloud (laser);
    voxel_grid.filter (*laser);
    //std::cerr << "Cloud before filtering: " << std::endl;
    //std::cerr << *laser << std::endl;

    // Create the filtering object
    //    pcl::StatisticalOutlierRemoval<point_type> sor;
    //    sor.setInputCloud (laser);
    //    sor.setMeanK (30);
    //    sor.setStddevMulThresh (1.0);
    //    sor.filter (*laser);

    //std::cerr << "Cloud after filtering: " << std::endl;
    //std::cerr << *laser << std::endl;

    // Transform to laser scan to map frame
    try
    {
        listener->waitForTransform(map_link, laser_link, filter_stamp_, ros::Duration(0.05) );
        listener->lookupTransform(map_link, laser_link, filter_stamp_, laserToMapTf);
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR("HERE2");

        ROS_WARN("%s",ex.what());

        return;
    }
    //std::cout << "after predicting:"<< laserToMapTf.getOrigin().getX() << " "  <<laserToMapTf.getOrigin().getY() << " " << laserToMapTf.getRotation().getAngle() << std::endl;

    pcl_ros::transformAsMatrix(laserToMapTf, laserToMapEigen);
    pcl::PointCloud<point_type>::Ptr laser_map (new pcl::PointCloud<point_type>());
    pcl::transformPointCloud (*laser, *laser_map, laserToMapEigen);
    laser_map->is_dense=false;
    laser_map->header.frame_id=map_link;
    pcl_conversions::toPCL(filter_stamp_, laser_map->header.stamp);

    // Compute ICP
    pcl::IterativeClosestPoint<point_type, point_type> icp;

    icp.setInputSource(laser_map);
    icp.setInputTarget(map_features);
    icp.setTransformationEpsilon (1e-8);
    icp.setMaxCorrespondenceDistance(10); //10
    icp.setMaximumIterations(200); //200
    icp.setRANSACIterations (200);

    pcl::PointCloud<point_type> Final;
    icp.align(Final);

    Final.header.frame_id=map_link;
    double scale=1.0;
    //std::cout << "conv:"<< icp.getFitnessScore()<< std::endl;
    R(0,0)=scale*icp.getFitnessScore();
    R(1,1)=scale*icp.getFitnessScore();
    R(2,2)=scale*icp.getFitnessScore();
    Eigen::Matrix4f correction_transform_map_frame=icp.getFinalTransformation();
    Eigen::Matrix4f correction_transform_=correction_transform_map_frame;

    //std::cout << correction_transform_ << std::endl;
    cv::Vec3d res;

    res[0]=correction_transform_(0,3);
    res[1]=correction_transform_(1,3);
    res[2]=correction_transform_.block<3,3>(0,0).eulerAngles (0,1,2)(2);
    angleOverflowCorrect(res[2]);

    correct(res);


    if(icp.hasConverged())
    {
        broadcast();
    }
    else
    {
        ROS_ERROR("DIDN_TE CONVERFGE");
    }

    pcl::transformPointCloud (*laser_map, *laser_map, correction_transform_);

    local_features_pub.publish(laser_map);
    return;
}


bool kalman::predict()
{

    tf::StampedTransform baseDeltaTf;

    tf::StampedTransform odomTf;

    // Get delta motion in cartesian coordinates with TF
    try
    {
        listener->waitForTransform(base_link, filter_stamp_old_, base_link, filter_stamp_ , odom_link, ros::Duration(0.05) );
        listener->lookupTransform(base_link, filter_stamp_old_, base_link, filter_stamp_, odom_link, baseDeltaTf); // Velocity

        listener->waitForTransform(base_link, odom_link, filter_stamp_ , ros::Duration(0.05) );
        listener->lookupTransform(base_link, odom_link, filter_stamp_, odomTf); // Velocity
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        return false;
    }

    double dt=filter_stamp_.toSec()-filter_stamp_old_.toSec();
    if(dt<0.02) // Problems at initialization
    {
    }


    //    double delta_yaw=baseDeltaTf.getRotation().getAngle();
    //    double delta_x=sqrt(baseDeltaTf.getOrigin().getX()*baseDeltaTf.getOrigin().getX() +baseDeltaTf.getOrigin().getY()*baseDeltaTf.getOrigin().getY());

    //    double v_yaw=-delta_yaw/dt;
    //    double v_x=delta_x/dt;
    //    //std::cout << "delta_x:"<< delta_x << std::endl;
    //    //std::cout << "delta_yaw:"<< delta_yaw << std::endl;
    //    //std::cout << "v_x:"<< v_x << std::endl;
    //    //std::cout << "v_yaw:"<< v_yaw << std::endl;
    //    nav_msgs::Odometry odom_msg;
    //    odom_msg.twist.twist.angular.z=v_yaw;
    //    odom_msg.twist.twist.linear.x=v_x;


    //this->linear = msg->twist.twist.linear.x;
    //this->angular = msg->twist.twist.angular.z;
    //    this->linear = odom_msg.twist.twist.linear.x;
    //    this->angular = odom_msg.twist.twist.angular.z;

    //    this->X[0] += linear * dt * cos( X[2] );
    //    this->X[1] += linear * dt * sin( X[2] );
    //    this->X[2] += angular * dt;

    double delta_x=baseDeltaTf.getOrigin().getX();
    double delta_y=baseDeltaTf.getOrigin().getY();
    double delta_theta=baseDeltaTf.getRotation().getAngle();
    angleOverflowCorrect(delta_theta);
    this->X[0] += delta_x;
    this->X[1] += delta_y;
    this->X[2] += delta_theta;
    angleOverflowCorrect(this->X_acum[2]);

//    this->X[0] = odomTf.getOrigin().getX();
//    this->X[1] = odomTf.getOrigin().getY();
//    this->X[2] = odomTf.getRotation().getAngle();
    angleOverflowCorrect(this->X[2]);

    std::cout << "X:" << X.t() << std::endl;
    std::cout << "X acum:" << X_acum.t() << std::endl << std::endl;



    //std::cout << "X" << cv::Point3d(X) << std::endl;

    //this->F(0,2) = -linear * dt * sin( X[2] ); //t+1 ?
    //this->F(1,2) =  linear * dt * cos( X[2] ); //t+1 ?

    P = F * P * F.t() + Q;
    P = (P + P.t()) * 0.5;



    // Send prediction to TF
    tf::Transform transform(tf::createQuaternionFromYaw(this->X[2]),
            tf::Vector3(this->X[0],
            this->X[1],
            0.0));

    broadcast();

    filter_stamp_old_=filter_stamp_;
    return true;


}

void kalman::correct(const cv::Vec3d & res)
{


    cv::Matx<double,3,3> S = H * P * H.t() + R;
    cv::Matx<double,3,3> K = P * H.t() * S.inv();

    //std::cout << std::endl << "K" << cv::Mat(K) << std::endl;

    X += K*res;
    angleOverflowCorrect(this->X[2]);
    //std::cout << "X" << cv::Point3d(X) << std::endl;

    P = (I - K * H) * P;
    P = (P + P.t()) * 0.5;
    //std::cout << "P" << std::endl << cv::Mat(P) << std::endl;

}

void kalman::sendTransform(const tf::Transform & transform_, const ros::Time & time_stamp_, const std::string & target_frame_, const std::string & origin_frame_)
{
    tf_broadcaster.sendTransform(tf::StampedTransform(transform_, time_stamp_, target_frame_, origin_frame_));
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
        tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf,
                                              filter_stamp_,
                                              base_link); // base to map
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
    tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                        filter_stamp_,//+ros::Duration(0.1)
                                        map_link, odom_link);
    tf_broadcaster.sendTransform(tmp_tf_stamped);

}







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

#include "kalman.hpp"

kalman::kalman(ros::NodeHandle& nh, const cv::Mat& pmap, double x_init, double y_init, double theta_init, int spin_rate, double voxel_grid_size_)
    : nh_priv("~"),
      map(pmap.clone()),
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

    nh_priv.param<std::string>("base_link",base_link, "base_link");
    nh_priv.param<std::string>("odom_link",odom_link, "odom");

    nh_priv.param("alpha_1",alpha_1, 0.05);
    nh_priv.param("alpha_2",alpha_2, 0.001);
    nh_priv.param("alpha_3",alpha_3, 5.0);
    nh_priv.param("alpha_4",alpha_4, 0.05);
    nh_priv.param("max_correspondence_distance",max_correspondence_distance, 100.0);
    nh_priv.param("max_iterations",max_iterations, 1000);
    nh_priv.param("ransac_iterations",ransac_iterations, 1000);
    nh_priv.param("ransac_outlier_threshold",ransac_outlier_threshold, 0.1);
    nh_priv.param("icp_optimization_epsilon",icp_optimization_epsilon, 0.0000001);
    nh_priv.param("icp_score_scale",icp_score_scale, 100.0);

    ROS_INFO_STREAM("alpha_1:"<<alpha_1);
    ROS_INFO_STREAM("alpha_2:"<<alpha_2);
    ROS_INFO_STREAM("alpha_3:"<<alpha_3);
    ROS_INFO_STREAM("alpha_4:"<<alpha_4);
    ROS_INFO_STREAM("max_correspondence_distance:"<<max_correspondence_distance);
    ROS_INFO_STREAM("max_iterations:"<<max_iterations);
    ROS_INFO_STREAM("ransac_iterations:"<<ransac_iterations);
    ROS_INFO_STREAM("ransac_outlier_threshold:"<<ransac_outlier_threshold);
    ROS_INFO_STREAM("icp_optimization_epsilon:"<<icp_optimization_epsilon);
    ROS_INFO_STREAM("icp_score_scale:"<<icp_score_scale);

    alpha_2=alpha_2*180/M_PI; // Convert to radian
    alpha_3=alpha_3*M_PI/180; // Convert to radians

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

    this->P(0,0) = 100000000000.1;
    this->P(1,1) = 100000000000.1;
    this->P(2,2) = 3.14;

    this->X_acum[0] = x_init;
    this->X_acum[1] = y_init;
    this->X_acum[2] = theta_init;

    this->F(0,0) = 1;
    this->F(1,1) = 1;
    this->F(2,2) = 1;

    this->I(0,0) = 1;
    this->I(1,1) = 1;
    this->I(2,2) = 1;



    // odometry cov
    this->Q(0,0) = 0.001;
    this->Q(1,1) = 0.001;

    //this->Q(0,1) = 0.00001;
    //this->Q(1,0) = 0.00001;

    this->Q(0,2) = 0.0002;
    this->Q(1,2) = 0.0002;

    this->Q(2,0) = 0.0002;
    this->Q(2,1) = 0.0002;

    this->Q(2,2) = 0.005;

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

    double map_scale=0.029999;
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
        point.x=map_scale*map_features_cv[i].x-0.13;
        point.y=-map_scale*map_features_cv[i].y-0.13;
        point.z=0.0;
        point.intensity=1.0;
        map_features->push_back(point);
    }

    pcl::VoxelGrid<point_type> voxel_grid;
    voxel_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
    voxel_grid.setInputCloud (map_features);
    voxel_grid.filter (*map_features);

    // Get Transform from laser to base frame

    tf::StampedTransform laserToBaseTf;
    while(ros::ok())
    {
        try
        {
            listener->waitForTransform(laser_link, base_link, ros::Time(0), ros::Duration(0.05) );
            listener->lookupTransform(laser_link, base_link, ros::Time(0), laserToBaseTf); // ABSOLUTE EGO TO WORLD
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

    if(!predict())
    {
        std::cout << "ups"<< std::endl;
        return;
    }

    tf::StampedTransform laserToBaseTf;
    tf::StampedTransform mapToBaseTf;
    try
    {
        listener->waitForTransform(map_link, laser_link, filter_stamp_, ros::Duration(0.01) );
        listener->lookupTransform(map_link, laser_link, filter_stamp_, laserToBaseTf);
        listener->waitForTransform(map_link, map_link, filter_stamp_, ros::Duration(0.01) );
        listener->lookupTransform(map_link, map_link, filter_stamp_, mapToBaseTf);
    }
    catch(tf::TransformException& ex)
    {
        ROS_WARN("%s",ex.what());
        return;

    }

    Eigen::Matrix4f mapToBaseEigen;
    Eigen::Matrix4f laserToBaseEigen;

    pcl_ros::transformAsMatrix(laserToBaseTf, laserToBaseEigen);
    pcl_ros::transformAsMatrix(mapToBaseTf, mapToBaseEigen);



    laser->resize(msg->ranges.size());
    laser->header.frame_id=base_link;
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
        point.intensity=1.0/sqrt(point.x*point.x+point.y*point.y);
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

    // Transform laser to base link
    pcl::PointCloud<point_type>::Ptr laser_in_base_link (new pcl::PointCloud<point_type>());
    pcl::transformPointCloud (*laser, *laser_in_base_link, laserToBaseEigen);
    laser_in_base_link->is_dense=false;
    laser_in_base_link->header.frame_id=map_link;
    pcl_conversions::toPCL(filter_stamp_, laser_in_base_link->header.stamp);

    // Transform map to base link
    pcl::PointCloud<point_type>::Ptr map_in_base_link (new pcl::PointCloud<point_type>());
    pcl::transformPointCloud (*map_features, *map_in_base_link, mapToBaseEigen);
    map_in_base_link->is_dense=false;
    map_in_base_link->header.frame_id=base_link;
    pcl_conversions::toPCL(filter_stamp_, map_in_base_link->header.stamp);

    // Compute ICP
    pcl::IterativeClosestPoint<point_type, point_type> icp;
    icp.setInputSource(laser_in_base_link);
    icp.setInputTarget(map_in_base_link);
    icp.setTransformationEpsilon (icp_optimization_epsilon);
    icp.setMaxCorrespondenceDistance(max_correspondence_distance); //10

    icp.setMaximumIterations(max_iterations); //200
    icp.setRANSACIterations (ransac_iterations);
    icp.setRANSACOutlierRejectionThreshold(ransac_outlier_threshold);
    pcl::PointCloud<point_type> Final;
    icp.align(Final);

    Final.header.frame_id=map_link;

    R(0,0)=icp_score_scale*icp.getFitnessScore(max_correspondence_distance);
    R(1,1)=icp_score_scale*icp.getFitnessScore(max_correspondence_distance);
    R(2,2)=icp_score_scale*icp.getFitnessScore(max_correspondence_distance);
    //Eigen::Matrix4f correction_transform_map_frame=icp.getFinalTransformation();
    Eigen::Matrix4f correction_transform_=icp.getFinalTransformation();

    cv::Vec3d res;

    res[0]=correction_transform_(0,3);
    res[1]=correction_transform_(1,3);
    res[2]=correction_transform_.block<3,3>(0,0).eulerAngles (0,1,2)(2);

    angleOverflowCorrect(res[2]);

    correct(res);

    //std::cout << res << std::endl;
    if(icp.hasConverged())
    {
        broadcast();
    }
    else
    {
        ROS_ERROR("DIDN_TE CONVERFGE");
    }

    //pcl::transformPointCloud (*laser_in_base_link, *laser_in_base_link, correction_transform_);
    local_features_pub.publish(Final);
    return;
}


bool kalman::predict()
{

    tf::StampedTransform baseDeltaTf;

    // Get delta motion in cartesian coordinates with TF
    try
    {
        listener->waitForTransform(base_link, filter_stamp_old_, base_link, filter_stamp_ , odom_link, ros::Duration(0.01) );
        listener->lookupTransform(base_link, filter_stamp_old_, base_link, filter_stamp_, odom_link, baseDeltaTf); // delta position

    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        return false;
    }

    double dx=baseDeltaTf.getOrigin().getX();
    double dy=baseDeltaTf.getOrigin().getY();
    double d_theta=baseDeltaTf.getRotation().getAxis()[2]*baseDeltaTf.getRotation().getAngle();

    //std::cout << "dy:"<< dy << " dx:" << dx << " dtheta:" <<d_theta<<  std::endl;

    double delta_rot1=atan2(dy,dx);
    double delta_trans=sqrt(dx*dx+dy*dy);
    double delta_rot2=d_theta-delta_rot1;

    this->X[0] += cos(this->X[2]+delta_rot1)*delta_trans;
    this->X[1] += sin(this->X[2]+delta_rot1)*delta_trans;
    this->X[2] += d_theta;


    angleOverflowCorrect(this->X[2]);
    //std::cout << delta_trans << std::endl;

    double var_rot1=alpha_1*delta_rot1*delta_rot1+alpha_2*delta_trans*delta_trans;
    double var_trans=alpha_3*delta_trans*delta_trans+alpha_4*(delta_rot1*delta_rot1+delta_rot2*delta_rot2);
    double var_rot2=alpha_1*delta_rot2*delta_rot2+alpha_2*delta_trans*delta_trans;
    cv::Matx<double,3,3> Sigma;
    Sigma(0,0)=var_rot1;
    Sigma(1,1)=var_trans;
    Sigma(2,2)=var_rot2;


    cv::Matx<double,3,3> J;
    J(0,0)=-sin(delta_rot1); J(0,1)=cos(delta_rot1); J(0,2)=0;
    J(1,0)=cos(delta_rot1);  J(1,1)=sin(delta_rot1);  J(1,2)=0;
    J(2,0)=1;  J(2,1)=0;  J(2,2)=1;


    P += J*Sigma*J.t();
    P = (P + P.t()) * 0.5;


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
        tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(),
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
                                        filter_stamp_+ros::Duration(0.1),
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

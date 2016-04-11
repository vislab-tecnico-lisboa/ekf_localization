#include "BayesianFilterNode.h"

using namespace MatrixWrapper;
using namespace BFL;
using namespace std;

BayesianFilterNode::BayesianFilterNode(const ros::NodeHandle &nh) :
    nh_(nh),
    private_node_handle_("~"),
    odom_initialized(false),
    listener(new tf::TransformListener(ros::Duration(2.0))),
    dt(0.0)
{
    double prior_mean;
    double prior_std_dev;
    int particles_number;
    // Initial estimate of position and orientation
    private_node_handle_.param("prior_mean",prior_mean, 1.0);
    private_node_handle_.param("prior_std_dev",prior_std_dev, 0.01);
    private_node_handle_.param("particles_number",particles_number, 100);
    private_node_handle_.param("prior_std_dev",prior_std_dev, 0.01);
    private_node_handle_.param<std::string>("base_link",base_link, "base_link");
    private_node_handle_.param<std::string>("odom_link",odom_link, "odom");

    ROS_INFO_STREAM("prior_mean: "<<prior_mean);

    createFilter(prior_mean,
                 prior_std_dev);

    laser_sub = nh_.subscribe("scan", 1, &BayesianFilterNode::laserCallback, this);
    laser_features_pub = nh_.advertise<sensor_msgs::PointCloud2>("/laser_features_cloud",1);
    location_undertainty = nh_.advertise<visualization_msgs::Marker>("/location_undertainty",1);
}

BayesianFilterNode::~BayesianFilterNode()
{}

void BayesianFilterNode::createFilter(const double & prior_mean,
                                      const double & prior_std_dev)
{
    ROS_INFO("CONSTRUCTING");
    /****************************
     * NonLinear system model   *
     ***************************/
    // create gaussian
    ColumnVector sys_noise_Mu(STATE_SIZE);
    sys_noise_Mu(1) = 0.0;
    sys_noise_Mu(2) = 0.0;
    sys_noise_Mu(3) = 0.0;
    ROS_INFO("CONSTRUCTING");

    SymmetricMatrix sys_noise_Cov(STATE_SIZE);
    sys_noise_Cov = 0.0;
    sys_noise_Cov(1,1) = 0.003;
    sys_noise_Cov(1,2) = 0.01;
    sys_noise_Cov(1,3) = 0.0;
    sys_noise_Cov(2,1) = 0.01;
    sys_noise_Cov(2,2) = 0.001;
    sys_noise_Cov(2,3) = 0.0;
    sys_noise_Cov(3,1) = 0.0;
    sys_noise_Cov(3,2) = 0.0;
    sys_noise_Cov(3,3) = 0.001;

    Gaussian system_Uncertainty(sys_noise_Mu, sys_noise_Cov);

    /*************************
     * Linear System model   *
     *************************/

    Matrix A(3,3);
    A(1,1) = 1.0;
    A(1,2) = 0.0;
    A(1,3) = 0.0;

    A(2,1) = 0.0;
    A(2,2) = 1.0;
    A(2,3) = 0.0;

    A(3,1) = 0.0;
    A(3,2) = 0.0;
    A(3,3) = 1.0;
    Matrix B(1,3);
    B(1,1) = 1.0;
    B(1,2) = 1.0;
    B(1,3) = 1.0;

    vector<Matrix> AB(2);
    AB[0] = A;
    AB[1] = B;


    sys_pdf = boost::shared_ptr<BFL::LinearAnalyticConditionalGaussian> (new  BFL::LinearAnalyticConditionalGaussian(AB, system_Uncertainty));
    sys_model = boost::shared_ptr<BFL::LinearAnalyticSystemModelGaussianUncertainty> (new BFL::LinearAnalyticSystemModelGaussianUncertainty (sys_pdf.get()));

    /*********************************
     * NonLinear Measurement model   *
     ********************************/

    // Construct the measurement noise (a scalar in this case)
    ColumnVector meas_noise_Mu(MEAS_SIZE);
    meas_noise_Mu(1) = 0.0;
    meas_noise_Mu(2) = 0.0;
    SymmetricMatrix meas_noise_Cov(MEAS_SIZE);
    meas_noise_Cov(1,1) = 0.001;
    meas_noise_Cov(1,2) = 0.0;
    meas_noise_Cov(2,1) = 0.0;
    meas_noise_Cov(2,2) = 0.001;


    Gaussian measurement_Uncertainty(meas_noise_Mu, meas_noise_Cov);

    Matrix H(MEAS_SIZE,STATE_SIZE);
    H = 0.0;
    H(1,1) = 1.0;
    H(2,2) = 1.0;
    //H(3,3) = 1.0;
    meas_pdf = boost::shared_ptr<BFL::LinearAnalyticConditionalGaussian>(new BFL::LinearAnalyticConditionalGaussian(H,measurement_Uncertainty));
    meas_model = boost::shared_ptr<BFL::LinearAnalyticMeasurementModelGaussianUncertainty>(new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(meas_pdf.get()));

    /****************************
     * Linear prior DENSITY     *
     ***************************/
    // Continuous Gaussian prior (for Kalman filters)
    ColumnVector prior_Mu(STATE_SIZE);
    prior_Mu(1) = 0;
    prior_Mu(2) = 0;
    prior_Mu(3) = prior_mean;
    SymmetricMatrix prior_Cov(STATE_SIZE);
    prior_Cov(1,1) = 0.168;
    prior_Cov(1,2) = -0.79;
    prior_Cov(1,3) = 0.0;
    prior_Cov(2,1) = -0.79;
    prior_Cov(2,2) = 0.038;
    prior_Cov(2,3) = 0.0;
    prior_Cov(3,1) = 0.0;
    prior_Cov(3,2) = 0.0;
    prior_Cov(3,3) = 0.0;
    Gaussian prior_cont(prior_Mu,prior_Cov);

    // Discrete prior for Particle filter (using the continuous Gaussian prior)


    /******************************
     * Construction of the Filter *
     ******************************/
    filter=boost::shared_ptr<ExtendedKalmanFilter> (new ExtendedKalmanFilter(&prior_cont));
    ROS_INFO("END CONSTRUCTING");

}

void BayesianFilterNode::laserCallback(const LaserConstPtr & msg)
{
    ROS_INFO("CALLBACK");

    tf::StampedTransform baseDeltaTf;

    // Get delta motion in cartesian coordinates with TF
    ros::Time current_time;

    try
    {
        current_time = ros::Time::now();
        listener->waitForTransform(base_link, current_time, base_link, last_update_time, odom_link, ros::Duration(0.1) );
        listener->lookupTransform(base_link, current_time, base_link, last_update_time, odom_link, baseDeltaTf); // DELTA EGO
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        return;
    }

    last_update_time=current_time;

    double delta_yaw=baseDeltaTf.getRotation().getAngle();
    double delta_x=baseDeltaTf.getOrigin().getX();
    double delta_y=baseDeltaTf.getOrigin().getY();


    ColumnVector prediction_mean(3);
    prediction_mean(1)=delta_x;
    prediction_mean(2)=delta_y;
    prediction_mean(3)=delta_yaw;

    BFL::SymmetricMatrix prediction_noise(STATE_SIZE);
    prediction_noise(1,1) = delta_x;
    prediction_noise(1,2) = 0.0;
    prediction_noise(1,3) = 0.0;
    prediction_noise(2,1) = 0.0;
    prediction_noise(2,2) = delta_y;
    prediction_noise(2,3) = 0.0;
    prediction_noise(3,1) = 0.0;
    prediction_noise(3,2) = 0.0;
    prediction_noise(3,3) = delta_yaw;
    sys_pdf->AdditiveNoiseSigmaSet(prediction_noise);

    //filter->Update(sys_model.get(),prediction_mean);


    Pdf<ColumnVector> * posterior = filter->PostGet();

    ColumnVector estimated_mean=posterior->ExpectedValueGet();

    BFL::SymmetricMatrix estimated_cov=posterior->CovarianceGet();


    // HERE WE SHOULD PREDICT

    Eigen::Vector2f mean_eigen;
    mean_eigen[0]=estimated_mean(1);
    mean_eigen[1]=estimated_mean(2);


    Eigen::Matrix2f cov_matrix;
    cov_matrix(0,0)=estimated_cov(1,1);
    cov_matrix(0,1)=estimated_cov(1,2);

    cov_matrix(1,0)=estimated_cov(2,1);
    cov_matrix(1,1)=estimated_cov(2,2);

    drawCovariance(mean_eigen,cov_matrix);
}




void BayesianFilterNode::drawCovariance(const Eigen::Vector2f& mean, const Eigen::Matrix2f& covMatrix)
{
    visualization_msgs::Marker tempMarker;
    tempMarker.pose.position.x = mean[0];
    tempMarker.pose.position.y = mean[1];

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



int main(int argc, char** argv)
{
    ros::init(argc, argv, "BayesianFilterNode");
    ros::NodeHandle nh;
    BayesianFilterNode pfNode(nh);
    ros::spin();
    return 0;
}

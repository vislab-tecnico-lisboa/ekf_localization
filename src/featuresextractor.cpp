#include "featuresextractor.h"

FeaturesExtractor::FeaturesExtractor()
{
}

std::vector<cv::Point2f> FeaturesExtractor::mapFeatures(cv::Mat & map,
                                                        const int & maxCorners,
                                                        const double & qualityLevel,
                                                        const double & minDistance,
                                                        const int & blockSize,
                                                        const bool & useHarrisDetector,
                                                        const double & k)
{
    std::vector<cv::Point2f> corners;
    //Good Features

    cv::goodFeaturesToTrack( map,corners,maxCorners,qualityLevel,minDistance,cv::Mat(),blockSize,useHarrisDetector,k );


    //Show results
    for( int i = 0; i < corners.size(); i++ )
    {
        //cv::circle( map, corners[i], 10, cv::Scalar(250,0,0), -1, 8, 0 );
        corners[i].x-=map.cols/2.0;
        corners[i].y-=map.rows/2.0;
    }


    //Show results
    //cv::imshow("TESTE",map);
    //cv::waitKey();

    return corners;
}

std::vector<cv::Point2f> FeaturesExtractor::localFeatures(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_in)
{
    pcl::HarrisKeypoint3D <pcl::PointXYZRGB, pcl::PointXYZRGB> detector;
      pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZRGB>);
      detector.setNonMaxSupression (true);
      detector.setInputCloud (point_cloud_in);
      detector.setThreshold (1e-6);
      pcl::StopWatch watch;
      detector.compute (*keypoints);
      pcl::console::print_highlight ("Detected %zd points in %lfs\n", keypoints->size (), watch.getTimeSeconds ());
      pcl::PointIndicesConstPtr keypoints_indices = detector.getKeypointsIndices ();
    //points_ = harris.getKeypointsIndices ();
}


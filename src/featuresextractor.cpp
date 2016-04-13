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

    cv::goodFeaturesToTrack( map,corners,1000000,qualityLevel,minDistance,cv::Mat(),blockSize,useHarrisDetector,k );


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

pcl::PointCloud<pcl::PointXYZI>::Ptr FeaturesExtractor::localFeatures(const pcl::PointCloud<point_type>::Ptr point_cloud_in)
{
    pcl::PointCloud<point_type>::Ptr point_cloud_out(new pcl::PointCloud<point_type>());

    pcl::HarrisKeypoint3D <point_type, point_type> detector;
    pcl::PointCloud<point_type>::Ptr keypoints (new pcl::PointCloud<point_type>);
    detector.setInputCloud (point_cloud_in);
    //detector.setNonMaxSupression (true);
    //detector.setThreshold (0.1);
    detector.setRadius(0.05);
    pcl::StopWatch watch;
    detector.compute (*keypoints);
    //pcl::console::print_highlight ("Detected %zd points in %lfs\n from %zd", keypoints->size (), watch.getTimeSeconds (), point_cloud_in->size());
    pcl::IndicesConstPtr keypoints_indices = detector.getIndices ();

    pcl::ExtractIndices<point_type> extract;

    extract.setInputCloud (point_cloud_in);
    extract.setIndices (keypoints_indices);
    extract.setNegative (false);
    extract.filter (*point_cloud_out);
    return point_cloud_out;
}


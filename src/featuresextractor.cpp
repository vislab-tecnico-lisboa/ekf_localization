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
    //Extracted Points
    std::vector<cv::Point2f> corners;

    //Extract Map Points

    for(int ii=1;ii<(map.rows)-1;ii++)
    {
        for(int jj=1;jj<(map.cols)-1;jj++)
        {
            if((cv::Point3i(map.at<cv::Vec3b>(ii,jj)).x==0) && (cv::Point3i(map.at<cv::Vec3b>(ii,jj)).y==0) && (cv::Point3i(map.at<cv::Vec3b>(ii,jj)).z==0))
            {
                corners.push_back(cv::Point(jj,ii));
            }
        }
    }

    //Scale
    for(int gp=0;gp<corners.size();gp++)
    {
        corners[gp].x-=map.cols/2.0;
        corners[gp].y-=map.rows/2.0;
    }

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


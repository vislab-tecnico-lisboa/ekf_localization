#ifndef FEATURESEXTRACTOR_H
#define FEATURESEXTRACTOR_H
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <pcl/keypoints/harris_3d.h>

class FeaturesExtractor
{


public:
    typedef pcl::PointXYZI point_type;
    FeaturesExtractor();

    std::vector<cv::Point2f> mapFeatures(cv::Mat & map,
                                         const int & maxCorners=50,
                                         const double & qualityLevel=0.001,
                                         const double & minDistance=1.0,
                                         const int & blockSize=3,
                                         const bool & useHarrisDetector=false,
                                         const double & k = 0.04);

    pcl::PointCloud<point_type>::Ptr localFeatures(const pcl::PointCloud<point_type>::Ptr point_cloud_in);
};

#endif // FEATURESEXTRACTOR_H

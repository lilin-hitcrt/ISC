#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <sys/time.h>
typedef cv::Mat ISCDescriptor;
class ISC
{
public:
    ISC(){
        ring_step = max_dis/rings;
        sector_step = 2*M_PI/sectors;
    };
    ~ISC(){};
    double cal_score(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud1, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud2);
private:
    int rings = 20;
    int sectors = 90;
    double ring_step = 0.0;
    double sector_step = 0.0;
    double max_dis = 60;
    ISCDescriptor calculate_isc(const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pointcloud);
    double calculate_geometry_dis(const ISCDescriptor& desc1, const ISCDescriptor& desc2, int& angle);
    double calculate_intensity_dis(const ISCDescriptor& desc1, const ISCDescriptor& desc2, int& angle);
    void ground_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out);
};
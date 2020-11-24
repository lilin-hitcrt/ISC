#include "isc.hpp"

ISCDescriptor ISC::calculate_isc(const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pointcloud)
{
    ISCDescriptor isc = cv::Mat::zeros(cv::Size(sectors, rings), CV_8U);

    for (int i = 0; i < (int)filtered_pointcloud->points.size(); i++)
    {
        double distance = std::sqrt(filtered_pointcloud->points[i].x * filtered_pointcloud->points[i].x + filtered_pointcloud->points[i].y * filtered_pointcloud->points[i].y);
        if (distance >= max_dis)
            continue;
        double angle = M_PI + std::atan2(filtered_pointcloud->points[i].y, filtered_pointcloud->points[i].x);
        int ring_id = std::floor(distance / ring_step);
        int sector_id = std::floor(angle / sector_step);
        if (ring_id >= rings)
            continue;
        if (sector_id >= sectors)
            continue;
#ifndef INTEGER_INTENSITY
        int intensity_temp = (int)(255 * filtered_pointcloud->points[i].intensity);
#else
        int intensity_temp = (int)(filtered_pointcloud->points[i].intensity);
#endif
        if (isc.at<unsigned char>(ring_id, sector_id) < intensity_temp)
            isc.at<unsigned char>(ring_id, sector_id) = intensity_temp;
    }

    return isc;
}

double ISC::calculate_geometry_dis(const ISCDescriptor& desc1, const ISCDescriptor& desc2, int& angle){
    double similarity = 0.0;

    for(int i=0;i<sectors;i++){
        int match_count=0;
        for(int p=0;p<sectors;p++){
            int new_col = p+i>=sectors?p+i-sectors:p+i;
            for(int q=0;q<rings;q++){
                if((desc1.at<unsigned char>(q,p)==true && desc2.at<unsigned char>(q,new_col)==true) || (desc1.at<unsigned char>(q,p)== false && desc2.at<unsigned char>(q,new_col)== false)){
                    match_count++;
                }
            }
        }
        if(match_count>similarity){
            similarity=match_count;
            angle = i;
        }

    }
    return similarity/(sectors*rings);
    
}
double ISC::calculate_intensity_dis(const ISCDescriptor& desc1, const ISCDescriptor& desc2, int& angle){
    double difference = 1.0;
    double angle_temp = angle;
    for(int i=angle_temp-10;i<angle_temp+10;i++){

        int match_count=0;
        int total_points=0;
        for(int p=0;p<sectors;p++){
            int new_col = p+i;
            if(new_col>=sectors)
                new_col = new_col-sectors;
            if(new_col<0)
                new_col = new_col+sectors;
            for(int q=0;q<rings;q++){
                    match_count += abs(desc1.at<unsigned char>(q,p)-desc2.at<unsigned char>(q,new_col));
                    total_points++;
            }
            
        }
        double diff_temp = ((double)match_count)/(sectors*rings*255);
        if(diff_temp<difference)
            difference=diff_temp;

    }
    return 1 - difference;
    
}

double ISC::cal_score(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud1, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud2){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZI>);
    struct timeval time_t;
    double time1,time2,time3,time4;
    ground_filter(cloud1,cloud_filtered1);
    ground_filter(cloud2,cloud_filtered2);
    gettimeofday(&time_t,nullptr);
    time1=time_t.tv_sec*1e3+time_t.tv_usec*1e-3;
    auto desc1=calculate_isc(cloud_filtered1);
    gettimeofday(&time_t,nullptr);
    time2=time_t.tv_sec*1e3+time_t.tv_usec*1e-3;
    auto desc2=calculate_isc(cloud_filtered2);
    int angle =0;
    gettimeofday(&time_t,nullptr);
    time3=time_t.tv_sec*1e3+time_t.tv_usec*1e-3;
    double geo_score = calculate_geometry_dis(desc1,desc2,angle);
    double inten_score = calculate_intensity_dis(desc1,desc2,angle);
    gettimeofday(&time_t,nullptr);
    time4=time_t.tv_sec*1e3+time_t.tv_usec*1e-3;
    float desc_time=time2-time1;
    float retri_time=time4-time3;
    std::cout<<"Description time:"<<desc_time<<std::endl;
    std::cout<<"Retrieval time:"<<retri_time<<std::endl;
    std::cout<<"Total time:"<<desc_time+retri_time<<std::endl;
    cv::imshow("desc1",desc1);
    cv::imshow("desc2",desc2);
    cv::waitKey(0);
    if(geo_score<0.67){
        return 0;
    }else{
        return inten_score;
    }
}

void ISC::ground_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out){
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud (pc_in);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-0.9, 30.0);
    pass.filter (*pc_out);

}
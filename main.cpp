#include<iostream>
#include <pcl/io/pcd_io.h>
#include <string>
#include "isc.hpp"
pcl::PointCloud<pcl::PointXYZI>::Ptr getCloud(std::string file){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    int32_t num = 1000000;
    float *data = (float*)malloc(num*sizeof(float));
    float *px = data+0;
    float *py = data+1;
    float *pz = data+2;
    float *pr = data+3;
    FILE *stream;
    stream = fopen (file.c_str(),"rb");
    if(stream==NULL){
        std::cerr<<"Stream is NULL!"<<std::endl;
        return NULL;
    }
    cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    num = fread(data,sizeof(float),num,stream)/4;
    for (int32_t i=0; i<num; i++) {
        pcl::PointXYZI p;
        p.x=(*px);
        p.y=(*py);
        p.z=(*pz);
        p.intensity=(*pr);
        cloud->points.push_back(p);
        px+=4; py+=4; pz+=4; pr+=4;
    }
    cloud->height=1;
    cloud->width=cloud->points.size();
    fclose(stream);
    free(data);
    return cloud;
}
 int main(int argc,char** argv){
     std::string path1,path2;
     if(argc!=3){
         std::cout<<"Usage:./demo file1.bin file2.bin"<<std::endl;
         path1="../data/000000.bin";
         path2="../data/000002.bin";
     }else{
        path1=argv[1];
        path2=argv[2];
     }
     ISC isc;
     auto cloud1=getCloud(path1);
     auto cloud2=getCloud(path2);
     auto score=isc.cal_score(cloud1,cloud2);
     std::cout<<"Score:"<<score<<std::endl;
     return 0;
 }
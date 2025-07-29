#include "processor.h"

template<typename PointType>
Processor<PointType>::Processor() {}

template<typename PointType>
Processor<PointType>::~Processor() {}

template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr Processor<PointType>::filterCloud(typename pcl::PointCloud<PointType>::Ptr cloud,
    float filterResolution, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint, bool enableCropBox){

    // Reference: https://pointclouds.org/documentation/tutorials/voxel_grid.html and https://pointclouds.org/documentation/classpcl_1_1_crop_box_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html
    // ============= VOXEL GRID DOWNSAMPLING ============= 
    typename pcl::PointCloud<PointType>::Ptr filteredcloud = std::make_shared<pcl::PointCloud<PointType>>();
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterResolution, filterResolution, filterResolution);
    sor.filter(*filteredcloud);

    // ============= CROP BOX ============= 
    if(enableCropBox){
        typename pcl::PointCloud<PointType>::Ptr croppedcloud = std::make_shared<pcl::PointCloud<PointType>>();
        pcl::CropBox<PointType> cropBoxFilter(true);
        cropBoxFilter.setInputCloud(filteredcloud);
        cropBoxFilter.setMin(minPoint);
        cropBoxFilter.setMax(maxPoint);
        cropBoxFilter.filter(*croppedcloud);
        return croppedcloud;
    }
}

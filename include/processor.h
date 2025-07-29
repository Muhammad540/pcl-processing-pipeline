/*
Author: Muhammad Ahmed
Free to use and modify

This header contains the template class and the necessary functions to process the point cloud data
*/

#ifndef PROCESSOR_H
#define PROCESSOR_H

// pcl specific headers
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>

// standard headers
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

// custom headers
#include "bounding_box.h"

template<typename PointType>
class Processor {
    public:
        Processor();
        ~Processor();

        /** Filter the point cloud using voxel grid filter */
        typename pcl::PointCloud<PointType>::Ptr filterCloud(typename pcl::PointCloud<PointType>::Ptr cloud, 
            float filterResolution, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);
        
        /** Helper function given the inliers, separate the point cloud into two parts, one with the inliers and the other without it*/
        std::pair<typename pcl::PointCloud<PointType>::Ptr, typename pcl::PointCloud<PointType>::Ptr> separateClouds(
            pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointType>::Ptr cloud);
        
        /** Separate the ground plane from the rest of the point cloud using Ransac */
        std::pair<typename pcl::PointCloud<PointType>::Ptr, typename pcl::PointCloud<PointType>::Ptr> segmentPlane(
            typename pcl::PointCloud<PointType>::Ptr cloud, int maxIterations, float distanceThreshold);
        
        /** Cluster the point cloud using KdTree and euclidian clustering algorithm */
        std::vector<typename pcl::PointCloud<PointType>::Ptr> clustering(typename pcl::PointCloud<PointType>::Ptr cloud,
            float clusterTolerance, int minSize, int maxSize);

        /** Bounding box generation */
        BoundingBox boundingBox(typename pcl::PointCloud<PointType>::Ptr cluster);

        /** Minimum oriented bounding box generation */
        BoundingBoxQ boundingBoxQ(typename pcl::PointCloud<PointType>::Ptr cluster);

        /** Load a pcd file */
        typename pcl::PointCloud<PointType>::Ptr loadPcd(std::string file);
};
















#endif
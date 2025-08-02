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
#include "processorConfig.h"

template<typename PointType>
class Processor {
    private:
        ProcessorConfig config_;

        Eigen::Vector4f parseVector4f(const std::string& str);
        std::string trim(const std::string& str);
        
    public:
        Processor();
        ~Processor();

        /** Filter the point cloud using voxel grid filter and crop it to keep only the points within min-max range */
        typename pcl::PointCloud<PointType>::Ptr filterCloud(typename pcl::PointCloud<PointType>::Ptr cloud, 
            float filterResolution, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint, bool enableCropBox = false);
        
        /** Remove a certain region of points given the min-max region */
        typename pcl::PointCloud<PointType>::Ptr removeRegion(typename pcl::PointCloud<PointType>::Ptr cloud,
            Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

        /** Helper function given the inliers, separate the point cloud into two parts, one with the inliers and the other without it*/
        std::pair<typename pcl::PointCloud<PointType>::Ptr, typename pcl::PointCloud<PointType>::Ptr> separateClouds(
            pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointType>::Ptr cloud);
        
        /** Separate the ground plane from the rest of the point cloud using Ransac */
        std::pair<typename pcl::PointCloud<PointType>::Ptr, typename pcl::PointCloud<PointType>::Ptr> segmentPlane(
            typename pcl::PointCloud<PointType>::Ptr cloud, int maxIterations, float distanceThreshold);
        
        /** Cluster the point cloud using KdTree and euclidian clustering algorithm */
        std::vector<typename pcl::PointCloud<PointType>::Ptr> clustering(typename pcl::PointCloud<PointType>::Ptr cloud,
            float clusterTolerance, int minSize, int maxSize);

        /** Bounding box generation for a given cluster */
        BoundingBox boundingBox(typename pcl::PointCloud<PointType>::Ptr cluster);

        /** Minimum oriented bounding box generation for a given cluster */
        BoundingBoxQ boundingBoxQ(typename pcl::PointCloud<PointType>::Ptr cluster);

        /** Load a pcd file */
        typename pcl::PointCloud<PointType>::Ptr loadPcd(std::string file);

        /** Save a pcd file */
        void savePcd(std::string file, typename pcl::PointCloud<PointType>::Ptr cloud);

        /** Load parameters from the config file */
        bool loadConfig(const std::string& configFile);

        /** Get current configuration file */
        inline ProcessorConfig getConfig() const { return config_; }

        /** Set configuration */
        inline void setConfig(const ProcessorConfig& config) { config_ = config; }

        /** Print current configuration */
        void printConfig() const;
};

#include "processor.inl"
















#endif
#ifndef PROCESSOR_CONFIG_H
#define PROCESSOR_CONFIG_H

#include <Eigen/Dense>

struct ProcessorConfig{
    // Voxel grid filtering parameters 
    float filterResolution = 0.2f;
    Eigen::Vector4f minPoint = Eigen::Vector4f(-10, -5, -2, 1);
    Eigen::Vector4f maxPoint = Eigen::Vector4f(10, 8, 1, 1);
    bool enableCropBox = false;

    // Region removal parameters 
    Eigen::Vector4f removeMinPoint = Eigen::Vector4f(-1.5, -1.7, -1, 1);
    Eigen::Vector4f removeMaxPoint = Eigen::Vector4f(2.6, 1.7, -0.4, 1);
    bool enableRegionRemoval = false;

    // Segmentation parameters 
    int maxIterations = 100;
    float distanceThreshold = 0.3f;
    Eigen::Vector3f obstacleCloudColor = {1.0f, 0.0f, 0.0f}; // Red
    Eigen::Vector3f groundPlaneColor = {0.0f, 1.0f, 0.0f}; // Green

    // Clustering parameters 
    float clusterTolerance = 0.5f;
    int minClusterSize = 10;
    int maxClusterSize = 500;

    // Visualization parameters
    bool renderSegementation = false;
    bool renderAxisAlignedBoxes = false;
    bool renderOrientedBoxes = false;
    bool renderClusters = false;
    bool visualizerTurnedON = false;

    ProcessorConfig() = default;
};

#endif
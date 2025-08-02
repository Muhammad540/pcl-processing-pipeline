#include <random>
#include <fstream>
#include <sstream>
#include <map>
#include <filesystem>
#include <algorithm>
#include <iostream>

template<typename PointType>
Processor<PointType>::Processor() {}

template<typename PointType>
Processor<PointType>::Processor(const std::string& configFile) {
    if (!loadConfig(configFile)){
        std::cerr << "Failed to load configuration file" << std::endl;
    }

    std::cout << "configuration loaded successfully" << std::endl;
}

template<typename PointType>
Processor<PointType>::~Processor() {}

template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr Processor<PointType>::filterCloud(
    typename pcl::PointCloud<PointType>::Ptr cloud,
    float filterResolution, 
    Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint){

    // Reference: https://pointclouds.org/documentation/tutorials/voxel_grid.html and https://pointclouds.org/documentation/classpcl_1_1_crop_box_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html
    // ============= VOXEL GRID DOWNSAMPLING ============= 
    typename pcl::PointCloud<PointType>::Ptr filteredcloud = std::make_shared<pcl::PointCloud<PointType>>();
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterResolution, filterResolution, filterResolution);
    sor.filter(*filteredcloud);
    
    if (config_.enableCropBox) {
        typename pcl::PointCloud<PointType>::Ptr croppedCloud = std::make_shared<pcl::PointCloud<PointType>>();
        pcl::CropBox<PointType> cropBox;
        cropBox.setInputCloud(filteredcloud);
        cropBox.setMin(minPoint);
        cropBox.setMax(maxPoint);
        cropBox.filter(*croppedCloud);
        return croppedCloud;
    }

    return filteredcloud;
}

template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr Processor<PointType>::removeRegion(
    typename pcl::PointCloud<PointType>::Ptr cloud,
    Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint){

    typename pcl::PointCloud<PointType>::Ptr remainingCloud = std::make_shared<pcl::PointCloud<PointType>>();
    std::vector<int> indices;

    pcl::CropBox<PointType> regionfilter(true);
    regionfilter.setInputCloud(cloud);
    regionfilter.setMin(minPoint);
    regionfilter.setMax(maxPoint);
    regionfilter.filter(indices);
    
    pcl::PointIndices::Ptr inliers = std::make_shared<pcl::PointIndices>();
    for (int index : indices){
        inliers->indices.push_back(index);
    }

    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*remainingCloud);

    return remainingCloud;
}

template<typename PointType>
std::pair<typename pcl::PointCloud<PointType>::Ptr, typename pcl::PointCloud<PointType>::Ptr> Processor<PointType>::separateClouds(
    pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointType>::Ptr cloud){
    
    typename pcl::PointCloud<PointType>::Ptr obstacleCloud = std::make_shared<pcl::PointCloud<PointType>>();
    typename pcl::PointCloud<PointType>::Ptr planeCloud = std::make_shared<pcl::PointCloud<PointType>>();

    pcl::ExtractIndices<PointType> extract;
    for (int index : inliers->indices){
        planeCloud->points.push_back(cloud->points[index]);
    }

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacleCloud);

    std::pair<typename pcl::PointCloud<PointType>::Ptr, typename pcl::PointCloud<PointType>::Ptr> segResult(obstacleCloud, planeCloud);
    return segResult;
}

template<typename PointType>
std::pair<typename pcl::PointCloud<PointType>::Ptr, typename pcl::PointCloud<PointType>::Ptr> Processor<PointType>::segmentPlane(
    typename pcl::PointCloud<PointType>::Ptr cloud,
    int maxIterations,
    float distanceThreshold){
    
    auto inliers = std::make_shared<pcl::PointIndices>();
    auto coefficients = std::make_shared<pcl::ModelCoefficients>();

    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0){
        std::cerr << "Could not estimate a planar model for the given PointCloud" << std::endl;
        return std::pair<typename pcl::PointCloud<PointType>::Ptr, typename pcl::PointCloud<PointType>::Ptr>(cloud, cloud);
    }

    std::pair<typename pcl::PointCloud<PointType>::Ptr, typename pcl::PointCloud<PointType>::Ptr> segResult = separateClouds(
        inliers, cloud);
    
    return segResult;
}

template<typename PointType>
std::vector<typename pcl::PointCloud<PointType>::Ptr> Processor<PointType>::clustering(
    typename pcl::PointCloud<PointType>::Ptr cloud,
    float clusterTolerance,
    int minSize,
    int maxSize){
    
    std::vector<typename pcl::PointCloud<PointType>::Ptr> clusters;
    typename pcl::search::KdTree<PointType>::Ptr tree = std::make_unique<pcl::search::KdTree<PointType>>();
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    typename pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for (const auto& cluster : clusterIndices){
        typename pcl::PointCloud<PointType>::Ptr cloudCluster = std::make_shared<pcl::PointCloud<PointType>>();
        for (const auto& idx : cluster.indices){
            cloudCluster->push_back((*cloud)[idx]);
        }
        cloudCluster->width = cloudCluster->size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusters.push_back(cloudCluster);
    }

    return clusters;
}

template<typename PointType>
BoundingBox Processor<PointType>::boundingBox(typename pcl::PointCloud<PointType>::Ptr cluster){
    PointType minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    BoundingBox box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointType>
BoundingBoxQ Processor<PointType>::boundingBoxQ(typename pcl::PointCloud<PointType>::Ptr cluster){
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);
    Eigen::Matrix3f covarianceMatrix;
    computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covarianceMatrix);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(covarianceMatrix, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigenSolver.eigenvectors();
    Eigen::Vector3f eigenValuesPCA = eigenSolver.eigenvalues();

    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointType>::Ptr cloudPointsProjected (new pcl::PointCloud<PointType>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);

    PointType minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    BoundingBoxQ box;
    box.bboxTransform = bboxTransform;
    box.bboxQuaternion = bboxQuaternion;
    box.box_length = maxPoint.x - minPoint.x;
    box.box_width = maxPoint.y - minPoint.y;
    box.box_height = maxPoint.z - minPoint.z;
    
    return box;
}

template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr Processor<PointType>::loadPcd(std::string file){
    typename pcl::PointCloud<PointType>::Ptr cloud = std::make_shared<pcl::PointCloud<PointType>>();
    if (pcl::io::loadPCDFile<PointType>(file, *cloud) == -1){
        PCL_ERROR("Couldn't read file \n");
    }
    return cloud;
}

template<typename PointType>
void Processor<PointType>::savePcd(std::string file, typename pcl::PointCloud<PointType>::Ptr cloud){
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->size() << " data points to " << file << std::endl;
}

template<typename PointType>
bool Processor<PointType>::loadConfig(const std::string& configFile){
    std::ifstream file(configFile);
    if (!file.is_open()){
        std::cerr << "Error: Cannot open config file: " << configFile << std::endl;
        return false;
    }

    std::string line;
    std::map<std::string, std::string> configMap;

    while (std::getline(file, line)){
        line = trim(line);
        
        // empty lines or comments are ignored
        if (line.empty() || line[0] == '#' || line[0] == ';') {
            continue;
        }

        size_t pos = line.find('=');
        if (pos != std::string::npos){
            std::string key = trim(line.substr(0, pos));
            std::string value = trim(line.substr(pos + 1));
            configMap[key] = value;
        }
    }
    file.close();

    try {
        if (configMap.find("filterResolution") != configMap.end()) {
            config_.filterResolution = std::stof(configMap["filterResolution"]);
        }
        
        if (configMap.find("minPoint") != configMap.end()) {
            config_.minPoint = parseVector4f(configMap["minPoint"]);
        }
        
        if (configMap.find("maxPoint") != configMap.end()) {
            config_.maxPoint = parseVector4f(configMap["maxPoint"]);
        }
        
        if (configMap.find("enableCropBox") != configMap.end()) {
            config_.enableCropBox = (configMap["enableCropBox"] == "true" || configMap["enableCropBox"] == "1");
        }
        
        if (configMap.find("removeMinPoint") != configMap.end()) {
            config_.removeMinPoint = parseVector4f(configMap["removeMinPoint"]);
        }
        
        if (configMap.find("removeMaxPoint") != configMap.end()) {
            config_.removeMaxPoint = parseVector4f(configMap["removeMaxPoint"]);
        }
        
        if (configMap.find("enableRegionRemoval") != configMap.end()) {
            config_.enableRegionRemoval = (configMap["enableRegionRemoval"] == "true" || configMap["enableRegionRemoval"] == "1");
        }
        
        if (configMap.find("maxIterations") != configMap.end()) {
            config_.maxIterations = std::stoi(configMap["maxIterations"]);
        }
        
        if (configMap.find("distanceThreshold") != configMap.end()) {
            config_.distanceThreshold = std::stof(configMap["distanceThreshold"]);
        }
        
        if (configMap.find("clusterTolerance") != configMap.end()) {
            config_.clusterTolerance = std::stof(configMap["clusterTolerance"]);
        }
        
        if (configMap.find("minClusterSize") != configMap.end()) {
            config_.minClusterSize = std::stoi(configMap["minClusterSize"]);
        }
        
        if (configMap.find("maxClusterSize") != configMap.end()) {
            config_.maxClusterSize = std::stoi(configMap["maxClusterSize"]);
        }

        if (configMap.find("renderSegementation") != configMap.end()) {
            config_.renderSegementation = (configMap["renderSegementation"] == "true" || configMap["renderSegementation"] == "1");
        }

        if (configMap.find("renderAxisAlignedBoxes") != configMap.end()) {
            config_.renderAxisAlignedBoxes = (configMap["renderAxisAlignedBoxes"] == "true" || configMap["renderAxisAlignedBoxes"] == "1");
        }
        
        if (configMap.find("renderOrientedBoxes") != configMap.end()) {
            config_.renderOrientedBoxes = (configMap["renderOrientedBoxes"] == "true" || configMap["renderOrientedBoxes"] == "1");
        }

        if (configMap.find("renderClusters") != configMap.end()) {
            config_.renderClusters = (configMap["renderClusters"] == "true" || configMap["renderClusters"] == "1");
        }

        if (configMap.find("visualizerTurnedON") != configMap.end()) {
            config_.visualizerTurnedON = (configMap["visualizerTurnedON"] == "true" || configMap["visualizerTurnedON"] == "1");
        }
        
        if (configMap.find("obstacleCloudColor") != configMap.end()) {
            config_.obstacleCloudColor = parseVector3f(configMap["obstacleCloudColor"]);
        }

        if (configMap.find("groundPlaneColor") != configMap.end()) {
            config_.groundPlaneColor = parseVector3f(configMap["groundPlaneColor"]);
        }
        
    } catch (const std::exception& e){
        std::cerr << "Error parsing config file: "<< e.what() << std::endl;
        return false;
    }

    return true;
}

template<typename PointType>
Eigen::Vector4f Processor<PointType>::parseVector4f(const std::string& str){
    std::stringstream ss(str);
    std::string item;
    std::vector<float> values;

    std::string cleaned = str;
    if (cleaned.front() == '(' && cleaned.back() == ')'){
        cleaned = cleaned.substr(1, cleaned.size() - 2);
    }

    std::stringstream cleanedSS(cleaned);
    while (std::getline(cleanedSS, item, ',')){
        values.push_back(std::stof(trim(item)));
    }

    if (values.size() != 4){
        throw std::runtime_error("Vector4f must have 4 elements");
    }

    return Eigen::Vector4f(values[0], values[1], values[2], values[3]);
}

template<typename PointType>
Eigen::Vector3f Processor<PointType>::parseVector3f(const std::string& str){
    std::stringstream ss(str);
    std::string item;
    std::vector<float> values;

    std::string cleaned = str;
    if (cleaned.front() == '(' && cleaned.back() == ')'){
        cleaned = cleaned.substr(1, cleaned.size() - 2);
    }

    std::stringstream cleanedSS(cleaned);
    while (std::getline(cleanedSS, item, ',')){
        values.push_back(std::stof(trim(item)));
    }

    if (values.size() != 3){
        throw std::runtime_error("Vector3f must have 3 elements");
    }

    return Eigen::Vector3f(values[0], values[1], values[2]);
}

template<typename PointType>
std::string Processor<PointType>::trim(const std::string& str){
    size_t first = str.find_first_not_of(' ');
    if (first == std::string::npos){
        return "";
    }

    size_t last = str.find_last_not_of(' ');
    return str.substr(first, last - first + 1);
}

template<typename PointType>
void Processor<PointType>::printConfig() const {
    std::cout << "=== Current Configuration ===" << std::endl;
    std::cout << "filterResolution: " << config_.filterResolution << std::endl;
    std::cout << "minPoint: (" << config_.minPoint.transpose() << ")" << std::endl;
    std::cout << "maxPoint: (" << config_.maxPoint.transpose() << ")" << std::endl;
    std::cout << "enableCropBox: " << (config_.enableCropBox ? "true" : "false") << std::endl;
    std::cout << "removeMinPoint: (" << config_.removeMinPoint.transpose() << ")" << std::endl;
    std::cout << "removeMaxPoint: (" << config_.removeMaxPoint.transpose() << ")" << std::endl;
    std::cout << "enableRegionRemoval: " << (config_.enableRegionRemoval ? "true" : "false") << std::endl;
    std::cout << "maxIterations: " << config_.maxIterations << std::endl;
    std::cout << "distanceThreshold: " << config_.distanceThreshold << std::endl;
    std::cout << "clusterTolerance: " << config_.clusterTolerance << std::endl;
    std::cout << "minClusterSize: " << config_.minClusterSize << std::endl;
    std::cout << "maxClusterSize: " << config_.maxClusterSize << std::endl;
    std::cout << "renderSegementation: " << (config_.renderSegementation ? "true" : "false") << std::endl;
    std::cout << "renderAxisAlignedBoxes: " << (config_.renderAxisAlignedBoxes ? "true" : "false") << std::endl;
    std::cout << "renderOrientedBoxes: " << (config_.renderOrientedBoxes ? "true" : "false") << std::endl;
    std::cout << "renderClusters: " << (config_.renderClusters ? "true" : "false") << std::endl;
    std::cout << "visualizerTurnedON: " << (config_.visualizerTurnedON ? "true" : "false") << std::endl;
    std::cout << "obstacleCloudColor: (" << config_.obstacleCloudColor.transpose() << ")" << std::endl;
    std::cout << "groundPlaneColor: (" << config_.groundPlaneColor.transpose() << ")" << std::endl;
    std::cout << "=============================" << std::endl;
}

template<typename PointType>
std::vector<std::filesystem::path> Processor<PointType>::streamPCD(const std::string& path){
    std::vector<std::filesystem::path> paths{std::filesystem::directory_iterator(path), std::filesystem::directory_iterator()};
    std::sort(paths.begin(), paths.end());
    return paths;
}

template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr Processor<PointType>::processCloud(
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    const typename pcl::PointCloud<PointType>::Ptr& cloud){

    // ============= FILTERING ============= 
    typename pcl::PointCloud<PointType>::Ptr filteredCloud = this->filterCloud(cloud, config_.filterResolution, config_.minPoint, config_.maxPoint);

    // ============= REGION REMOVAL ============= 
    if (config_.enableRegionRemoval) {
        filteredCloud = this->removeRegion(filteredCloud, config_.removeMinPoint, config_.removeMaxPoint);
    }

    // ============= PLANE SEGMENTATION ============= 
    std::pair<typename pcl::PointCloud<PointType>::Ptr, typename pcl::PointCloud<PointType>::Ptr> segResult = this->segmentPlane(filteredCloud, config_.maxIterations, config_.distanceThreshold);

    // ============= CLUSTERING ============= 
    std::vector<typename pcl::PointCloud<PointType>::Ptr> clusters = this->clustering(segResult.first, config_.clusterTolerance, config_.minClusterSize, config_.maxClusterSize);
    
    // ============= VISUALIZATION ============= 
    if (config_.visualizerTurnedON){

        if (config_.renderSegementation){
            visualizePointCloud<PointType>(viewer, segResult.first, "obstacleCloud", config_.obstacleCloudColor);
        }
        int clusterId = 0;

        for (auto cluster : clusters){
            std::vector<float> color = colorPalette[clusterId % colorPalette.size()];
            Eigen::Vector3f colorVec(color[0], color[1], color[2]);
            if (config_.renderAxisAlignedBoxes){
                BoundingBox box = this->boundingBox(cluster);
                visualizeBox<BoundingBox>(viewer, box, clusterId, colorVec);
            }
            if (config_.renderOrientedBoxes){
                BoundingBoxQ box = this->boundingBoxQ(cluster);
                visualizeBox<BoundingBoxQ>(viewer, box, clusterId, colorVec);
            }
            if (config_.renderClusters){
                visualizePointCloud<PointType>(viewer, cluster, "cluster" + std::to_string(clusterId), colorVec);
            }
            clusterId++;
        }

        if (config_.renderSegementation){
            visualizePointCloud<PointType>(viewer, segResult.second, "planeCloud", config_.groundPlaneColor);
        }
    }

    return filteredCloud;
}
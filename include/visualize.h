#ifndef VISUALIZE_H
#define VISUALIZE_H

#include <pcl/visualization/pcl_visualizer.h>
#include "bounding_box.h"
#include <vector>

enum CameraAngle{
    XY,
    TopDown,
    Side,
    FPS
};

void initializeViewer(CameraAngle angle, pcl::visualization::PCLVisualizer::Ptr& viewer){
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    int distance = 16;

    switch (angle){
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1); break;
    }

    if (angle != FPS){
        viewer->addCoordinateSystem(1.0);
    }
}

/**
 * @brief Render a point cloud with a given name and color
 * 
 * @tparam PointType 
 * @param viewer 
 * @param cloud 
 * @param name 
 * @param color 
 */
template<typename PointType>
void visualizePointCloud(
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    const typename pcl::PointCloud<PointType>::Ptr& cloud,
    const std::string& name,
    const Eigen::Vector3f& color){
    
    if constexpr(std::is_same_v<PointType, pcl::PointXYZI>){
        if (color(0) == -1){
            // use intensity to color code
            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "intensity");
            viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, name);
        } else {
            // use color to color code
            viewer->addPointCloud<pcl::PointXYZI>(cloud, name);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color(0), color(1), color(2), name);
        }
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
    } else if constexpr(std::is_same_v<PointType, pcl::PointXYZ>) {
        viewer->addPointCloud<pcl::PointXYZ>(cloud, name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color(0), color(1), color(2), name);
    } else if constexpr(std::is_same_v<PointType, pcl::PointXYZRGB>) {
        if (color(0) == -1){
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
            viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, name);  
        } else {
            viewer->addPointCloud<pcl::PointXYZRGB>(cloud, name);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color(0), color(1), color(2), name);
        }
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name);
    } else {
        viewer->addPointCloud<PointType>(cloud, name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
        if (color(0) == -1){
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, name);
        } else {
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color(0), color(1), color(2), name);
        }
    }
}

template<typename BoxType>
void visualizeBox(
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    BoxType box,
    int id,
    const Eigen::Vector3f& color,
    float opacity = 0.5){
    
    if (opacity > 1.0) opacity = 1.0;
    if (opacity < 0.0) opacity = 0.0;

    std::string cube = "box" + std::to_string(id);
    std::string cubeFill = "boxFill" + std::to_string(id);

    if constexpr(std::is_same_v<BoxType, BoundingBox>){
        // Axis aligned bounding box (AABB): uses min/max coordinates
        viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color(0), color(1), color(2), cube);
        viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color(0), color(1), color(2), cubeFill);
    } else if constexpr(std::is_same_v<BoxType, BoundingBoxQ>){
        // Oriented bounding box (OBB): uses transform and quaternion
        viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.box_length, box.box_width, box.box_height, cube);
        viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.box_length, box.box_width, box.box_height, cubeFill);
    } else {
        static_assert(std::is_same_v<BoxType, BoundingBox> || std::is_same_v<BoxType, BoundingBoxQ>, "Unsupported box type");
    }
    // Common rendering properties for wireframe
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, 
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 
            color(0), color(1), color(2), cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

    // Common rendering properties for filled surface
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, 
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 
            color(0), color(1), color(2), cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity * 0.3f, cubeFill);
}

#endif
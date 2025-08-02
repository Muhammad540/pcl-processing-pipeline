# PCL Point Cloud Processing Pipeline

A configurable C++ library for point cloud processing built on top of the Point Cloud Library (PCL). This library is designed as a pipeline which can be used in autonomous driving and robotics applications, eliminating the need to implement common point cloud processing operations from scratch.

You just have to supply the point cloud input and set your desired parameters in the config file and the rest will be handled by the pipeline. It will output a processed point cloud in accordance with the parameters you have set. If you want to visualize the output, you can use the built in renderer as well. 

Below you can also find a simple example as to how you can use this library in your ROS projects or standalone applications.

## Features

### Processing Capabilities
- **Templated Design**: Works with any PCL point type (`pcl::PointXYZ`, `pcl::PointXYZI`, `pcl::PointXYZRGB`, etc.)
- **Voxel Grid Filtering**: Configurable downsampling for performance optimization
- **Region of Interest Cropping**: Spatial filtering to focus on relevant areas
- **Ego Vehicle Removal**: Remove points from the sensor's own vehicle/robot
- **Ground Plane Segmentation**: RANSAC based plane detection and separation
- **Euclidean Clustering**: KD tree based object clustering with configurable parameters
- **Bounding Box Generation**: Both axis aligned (AABB) and oriented (OBB) bounding boxes

### Visualization & I/O
- **Built in 3D Visualization**: Real time point cloud rendering with multiple camera angles
- **Configurable Colors**: Customizable colors for different point cloud components
- **Multiple Rendering Options**: Toggle between segmentation, clusters, and bounding boxes
- **PCD File Support**: Load, process, and save point cloud data
- **Streaming Mode**: Process sequences of PCD files with automatic iteration

### Configuration System
- **External Configuration**: All parameters configurable via text file `config.txt`
- **Test at runtime**: Change parameters without recompilation

## Installation

### Prerequisites
- PCL 1.12+
- C++17 compiler
- CMake 3.12+

### Usage Example
```cpp
#include "processor.h"

int main() {
    Processor<pcl::PointXYZI> processor("config.txt");
    auto cloud = processor.loadPcd("data.pcd");
    auto processed = processor.processCloud(nullptr, cloud);
    return 0;
```

### Configuration Parameters

| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `filterResolution` | float | Voxel grid leaf size (meters) | 0.2 |
| `minPoint` | Vector4f | ROI minimum bounds (x,y,z,1) | (-10,-5,-2,1) |
| `maxPoint` | Vector4f | ROI maximum bounds (x,y,z,1) | (30,7,1,1) |
| `enableCropBox` | bool | Enable spatial cropping | true |
| `enableRegionRemoval` | bool | Enable ego vehicle removal | false |
| `removeMinPoint` | Vector4f | Ego region min bounds | (-1.5,-1.7,-1,1) |
| `removeMaxPoint` | Vector4f | Ego region max bounds | (2.6,1.7,-0.4,1) |
| `maxIterations` | int | RANSAC max iterations | 100 |
| `distanceThreshold` | float | RANSAC distance threshold | 0.3 |
| `clusterTolerance` | float | Euclidean clustering tolerance | 1.0 |
| `minClusterSize` | int | Minimum cluster size | 20 |
| `maxClusterSize` | int | Maximum cluster size | 1000 |
| `obstacleCloudColor` | Vector3f | Obstacle point color (R,G,B) | (1,0,0) |
| `groundPlaneColor` | Vector3f | Ground plane color (R,G,B) | (0,1,0) |
| `visualizerTurnedON` | bool | Enable visualization | true |
| `CameraAngle` | int | Camera view (0-3) | 0 |
| `renderSegementation` | bool | Show segmented clouds | true |
| `renderAxisAlignedBoxes` | bool | Show AABB boxes | true |
| `renderOrientedBoxes` | bool | Show OBB boxes | false |
| `renderClusters` | bool | Show clustered objects | true |

## Integration

### ROS Integration
```cpp
// In your ROS node
#include "processor.h"

class PointCloudProcessor {
private:
    Processor<pcl::PointXYZI> processor_;
    
public:
    PointCloudProcessor() : processor_("config.txt") {}
    
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);
        
        // Process without visualization for ROS
        auto processed = processor_.processCloud(nullptr, cloud);
        
        // You can use the processed cloud for your own purposes
    }
};
```

## Future Improvements

### Planned Features
- [ ] **Object Detection**: Integration with deep learning models
- [ ] **Multi sensor Fusion**: Camera + LiDAR processing
- [ ] **ROS2 Wrapper**: Native ROS2 package
- [ ] **Performance Profiling**: Timing analysis and optimization
- [ ] **Pipeline Statistics**: Processing metrics and reporting
- [ ] **Advanced Filtering**: Statistical outlier removal, bilateral filtering
- [ ] **Tracking**: Multi frame object tracking
- [ ] **Export Formats**: Support for PLY, OBJ, and other formats

### Performance Optimizations
- [ ] **GPU Acceleration**: CUDA based processing
- [ ] **Multi threading**: Parallel processing stages
- [ ] **Memory Optimization**: Reduced memory footprint

### Usability Improvements
- [ ] **Python Bindings**: Python API for rapid prototyping
- [ ] **GUI Configuration**: Visual parameter tuning
- [ ] **Plugin System**: Extensible processing modules
- [ ] **Docker Support**: Containerized deployment

## Contributing

I welcome contributions!

## Acknowledgments

- Built as a wrapper around [Point Cloud Library (PCL)](https://pointclouds.org/)
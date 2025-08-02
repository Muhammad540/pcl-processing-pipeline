/**
 * An example demonstrating how to use the pcl_processor_lib.
 * This executable streams PCD files from a specified directory, processes them
 * through the pipeline, and visualizes the results in real time.
 */
#include "processor.h"
#include <memory>
#include <signal.h>
#include <atomic>

std::atomic<bool> shutdownRequested(false);

void signalHandler(int signal){
    std::cout << "Signal " << signal << " received. Shutting down..." << std::endl;
    shutdownRequested.store(true);
}

int main(int argc, char* argv[]){
    if (argc != 2){
        std::cerr << "Usage: " << argv[0] << " <path_to_pcd_files>" << std::endl;
        return 1;
    }

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
     // The config file is expected to be in the same directory as the executable but you can change it to any other path if you want
    auto processorCloud = std::make_unique<Processor<pcl::PointXYZI>>("config.txt");

    // --- Initialize Visualizer and Data Stream ---
    // There are 4 camera angles: | XY | TopDown | Side | FPS |
    CameraAngle setangle = CameraAngle::XY;
    initializeViewer(setangle, viewer);

    std::vector<std::filesystem::path> paths = processorCloud->streamPCD(argv[1]);
    if (paths.empty()){
        std::cerr << "No PCD files found in the directory" << std::endl;
        return 1;
    }
    auto pathIterator = paths.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

    while (!viewer->wasStopped() && !shutdownRequested.load()){
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        
        try {
            inputCloud = processorCloud->loadPcd(pathIterator->string());
            processorCloud->processCloud(viewer, inputCloud);
        } catch (const std::exception& e){
            std::cerr << "Error processing PCD file: " << e.what() << std::endl;
        }

        pathIterator++;
        if (pathIterator == paths.end()){
            pathIterator = paths.begin();
        }

        viewer->spinOnce();
    }
    std::cout << "Shutting down..." << std::endl;
    return 0;
}
cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(pointcloud_processing_pipeline VERSION 1.0.0)

find_package(PCL 1.2 REQUIRED)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_library(pcl_processor_lib INTERFACE)

target_include_directories(pcl_processor_lib INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(pcl_processor_lib INTERFACE ${PCL_LIBRARIES})
target_include_directories(pcl_processor_lib INTERFACE ${PCL_INCLUDE_DIRS})
target_compile_definitions(pcl_processor_lib INTERFACE ${PCL_DEFINITIONS})


add_executable(pcl_processor_example src/main.cpp)

target_link_libraries(pcl_processor_example PRIVATE pcl_processor_lib)
target_compile_options(pcl_processor_example PRIVATE -Wall -Wextra)

configure_file(${CMAKE_SOURCE_DIR}/src/config.txt ${CMAKE_BINARY_DIR}/config.txt COPYONLY)


install(DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

install(TARGETS pcl_processor_example DESTINATION bin)
install(FILES src/config.txt DESTINATION share/${PROJECT_NAME})
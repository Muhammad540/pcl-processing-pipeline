cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(pointcloud_processing_pipeline VERSION 1.0.0)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)

configure_file(${CMAKE_SOURCE_DIR}/src/config.txt ${CMAKE_BINARY_DIR}/config.txt COPYONLY)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES src/config.txt DESTINATION share/${PROJECT_NAME})
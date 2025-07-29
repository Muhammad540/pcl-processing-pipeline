#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include <Eigen/Geometry>

struct BoundingBoxQ {
    // translation to represent the position of the box
    Eigen::Vector3f position;
    // quaternion to represent the orientation of the box
    Eigen::Quaternionf bboxQuaternion;
    // length, width, height of the box
    float box_length, box_width, box_height;
};

struct BoundingBox {
    float x_min, y_min, z_min, x_max, y_max, z_max;
};

#endif
#ifndef DETECTEDBLOCK_CLASS_H_
#define DETECTEDBLOCK_CLASS_H_

class DetectedBlock{
    public:
        Eigen::Vector3d block_top_center;
        Eigen::Vector3d block_normal;
        double angle_around_z;


};

#endif
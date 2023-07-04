#ifndef DETECTBLOCKS_CLASS_H_
#define DETECTBLOCKS_CLASS_H_

#include <open3d/Open3D.h>
#include "detectedblock.h"


using namespace open3d;

template <typename T>
inline std::vector<size_t> sort_indexes(const std::vector<T> &v) {
    // initialize original index locations
    std::vector<size_t> idx(v.size());
    iota(idx.begin(), idx.end(), 0);
    std::stable_sort(idx.begin(), idx.end(),
                     [&v](size_t i1, size_t i2) { return v[i1] < v[i2]; });

    return idx;
}

//enum class DebugLevel { None, Verbal, Visual };
enum DebugLevel { None = 0, Verbal=1, Visual = 2 };

class Open3DPointCloud {
public:
    Open3DPointCloud(double _block_size, bool _use_plane, DebugLevel debug_level);
    void overrideMinClusterSize(int mcs);
    int FindLargestCloudIndex(const std::vector<std::vector<size_t>> _indexes);
    std::vector<size_t> ExtractLargestPlane(const geometry::PointCloud& plane_cloud_ptr,
        Eigen::Vector4d& plane_equation,
        std::shared_ptr<geometry::PointCloud>& leftovers_cloud_ptr);
    std::vector<std::vector<size_t>> FilterSegmentIndexes(std::vector<int> horiz_cloud_segments);
    Eigen::Vector3d IsolateLargestVerticalSide(//const std::shared_ptr<geometry::PointCloud> verticalfaces_cloud_ptr,
        const geometry::PointCloud& verticalfaces_cloud_ptr,
        std::shared_ptr<geometry::PointCloud>& best_normals_cloud);
    Eigen::Vector3d AverageOfVector3d(std::vector<Eigen::Vector3d> vectorVector3d);
    void PrintPlaneEquation(Eigen::Vector4d plane_equation);
    geometry::LineSet CreateLine(Eigen::Vector3d start, Eigen::Vector3d end);
    bool SavePointCloud(std::string filename_ply, geometry::PointCloud pcd);
    double to_degrees(double rad);
    Eigen::Vector3d GetObjectBounds(const geometry::PointCloud& _segment, Eigen::Vector3d& min_bounds,
        Eigen::Vector3d& max_bounds);
    void DebugVerticalNormalFace(geometry::PointCloud& best_normals_cloud,
        const Eigen::Vector3d vertical_side_normal);
    std::tuple<std::vector<size_t>, std::vector<size_t>, std::vector<size_t>>
        SeperateHorizandVertNorms(const std::vector<Eigen::Vector3d>& points,
            const std::vector<Eigen::Vector3d>& norms,
            const double floor);

    double PointToCloudMinDistance(geometry::PointCloud& pcd_down, Eigen::Vector3d tgtpt);

    std::vector<size_t> index_subset(std::vector<size_t> v1, std::vector<size_t> subsetofv1);

    std::tuple<Eigen::Vector3d, Eigen::Vector3d> rotationMatrixToEulerAngles(Eigen::Matrix4d R);

    std::shared_ptr<geometry::PointCloud> SegmentHorizontalSurface(
        /*Cant by const, calling estimatenormals...*/
        geometry::PointCloud &pcd_down,
        Eigen::Vector3d target_object_center,
        std::vector<double>& horizontal_surface_height,
        geometry::AxisAlignedBoundingBox& target_obb
    );

    void SegmentBlocks(
        /*Cant be const because you call estimate normals.  By ref is good enough.*/
        geometry::PointCloud& pcd_down,
        std::vector<double>& horizontal_surface_height,
        std::vector<DetectedBlock>& block_list,
        double floor_threshold
    );

    private:
        
        DetectedBlock ExtractBlock(std::vector<size_t> &horiz_indexes2, 
            std::vector<size_t> &vert_indexes2,
            std::shared_ptr<geometry::PointCloud> leftovers_segment,
            double surface_height);
        double FLOOR_THRESHOLD = -1.3;  //TO DO:  Fix this and create a new test file.

        DebugLevel debug_level = DebugLevel::Visual;
        bool use_plane = false;
        double block_size = 0.03;  //Block size in cm
        double max_block_size = block_size * 4.0; //set this to your tolerance
        size_t min_cluster_size = 30;
        double largest_plane_surface_height = 0.0;
        std::vector<Eigen::Vector3d> debug_colors = { Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0),
            Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 1.0, 1.0),
            Eigen::Vector3d(1.0, 0.5, 0.0), Eigen::Vector3d(1.0, 1.0, 0.0) };
        void VisualizeSegmentedCloud(std::vector<size_t> floor_indexes,
            std::vector<size_t> horiz_indexes, 
            std::vector<size_t> vert_indexes,
            geometry::PointCloud pcd_down);

};
#endif
#include "../../sharedlib/include/detectblocks.h"

#include <iostream>
#include <memory>
#include <thread>

#include "open3d/Open3D.h"


using namespace open3d;
// A simplified version of examples/cpp/Visualizer.cpp to demonstrate linking
// an external project to Open3D.
int main(int argc, char *argv[]) {
    std::vector<double> horizontal_surface_height;
    
    int floor_surface_height = 0.20;  //Anything below this is floor
    //auto plyfilename ="/home/valerie/edward2_ws/block_object_cloud.ply";
    auto plyfilename ="/home/valerie/sample_ros2/debugdetectblock.ply";
    auto pcd_down = open3d::io::CreatePointCloudFromFile(plyfilename);
    auto coord_axis = geometry::TriangleMesh::CreateCoordinateFrame(1.0, Eigen::Vector3d(0,0,0));

    pcd_down = pcd_down->UniformDownSample(2);
    visualization::DrawGeometries({pcd_down, coord_axis},
                                      "Target Point Cloud");
                                      
    std::vector<std::shared_ptr<const geometry::Geometry>> geometry_ptrs;
    geometry_ptrs.push_back(pcd_down);

    std::vector<double> horizontal_surface_heights;
    std::vector<DetectedBlock> block_list;

    Open3DPointCloud o3dpc(0.03f, false, DebugLevel::Visual );

    o3dpc.SegmentBlocks(
        *pcd_down,
        horizontal_surface_heights,
        block_list, 
        -1.3);

    std::cout << "FOUND " << block_list.size() << " BLOCKS!" << std::endl;
    for (auto &block : block_list){

        auto box_top_center_marker = geometry::TriangleMesh::CreateSphere(0.01);  //1 cm sphere
        box_top_center_marker->PaintUniformColor(Eigen::Vector3d(0.0, 1.0, 0.0));
        box_top_center_marker->Translate(block.block_top_center);
        geometry_ptrs.push_back(box_top_center_marker);

        Eigen::Vector3d end_point = block.block_top_center + block.block_normal*0.1;
        //pretend like it is perfectly horizontal:
        end_point[2] = block.block_top_center[2];
        auto normal_line = o3dpc.CreateLine(block.block_top_center, end_point);  //1 cm
        normal_line.PaintUniformColor(Eigen::Vector3d(0.10, 0.92, 0.8));
        auto normal_line_ptr = std::make_shared<geometry::LineSet>(normal_line);           
        geometry_ptrs.push_back(normal_line_ptr);
        geometry_ptrs.push_back(coord_axis);


        std::cout << "center: " << block.block_top_center << std::endl;
        std::cout << "normal: " << block.block_normal << std::endl;
        std::cout << "z rotation: " << o3dpc.to_degrees(block.angle_around_z) << std::endl;
        
        std::cout << "---------------------------------" << std::endl;
    }

    visualization::DrawGeometries(geometry_ptrs, 
                                      "Target Point Cloud");
                                     
    return 0;
}
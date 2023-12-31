
#include <gtest/gtest.h>
#include "../../sharedlib/include/detectblocks.h"
#include <iostream>
#include <memory>
#include <thread>
#include "open3d/Open3D.h"
#include <math.h>

#include <chrono>


#define EXPECT_IN_RANGE(VAL, MIN, MAX) \
    EXPECT_GE((VAL), (MIN));           \
    EXPECT_LE((VAL), (MAX))

#define ASSERT_IN_RANGE(VAL, MIN, MAX) \
    ASSERT_GE((VAL), (MIN));           \
    ASSERT_LE((VAL), (MAX))

double angleToHorizPlane(Eigen::Vector3d vv){
    Eigen::Vector3d xy = Eigen::Vector3d(0,0,1);
    auto dp = xy.dot(vv);
    return asin(dp);
}

double angleToVerticalPlane(Eigen::Vector3d vv){
    
    Eigen::Vector3d xz = Eigen::Vector3d(1,0,0);
    auto dp = xz.dot(vv);
    return acos(dp);
}



TEST(BlockDetectionTest, Cloud1) { 
    std::vector<double> horizontal_surface_height;
    int floor_surface_height = 0.20;  //Anything below this is floor
    //auto plyfilename ="../plys/counter_height_38.ply";
    //auto plyfilename ="../plys/debugdetectblock1.ply";
    auto plyfilename ="../plys/debugdetectblock2.ply";
    

    Eigen::Vector3d vv = Eigen::Vector3d(0.4,0.3,0.8);
    vv.normalize();
    std::cout <<vv << std::endl;

    std::cout << angleToVerticalPlane(vv) << std::endl;

    //auto plyfilename ="/home/valerie/blockdetection_ros2/plys/debugdetectblock2.ply";
    auto pcd_down = open3d::io::CreatePointCloudFromFile(plyfilename);
    auto coord_axis = geometry::TriangleMesh::CreateCoordinateFrame(0.30, Eigen::Vector3d(0,0, -0.630));

    pcd_down = pcd_down->UniformDownSample(2);

    visualization::DrawGeometries({pcd_down}, 
                                      "Incoming Point Cloud ");


    std::vector<std::shared_ptr<const geometry::Geometry>> geometry_ptrs;
    geometry_ptrs.push_back(pcd_down);

    std::vector<double> horizontal_surface_heights;
    std::vector<DetectedBlock> block_list;

    Open3DPointCloud o3dpc(0.03f, false, DebugLevel::Visual );

    auto start_time = std::chrono::high_resolution_clock::now();

    o3dpc.SegmentBlocks(
        *pcd_down,
        horizontal_surface_heights,
        block_list, 
        -1.3);

    auto end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Time difference:"
      << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << " ms" << std::endl;


    //std::cout << "SURFACE HEIGHT " << horizontal_surface_heights[0] << std::endl;
    
    std::cout << "FOUND " << block_list.size() << " BLOCKS!" << std::endl;
   
    
    for (auto &block : block_list){

        auto box_top_center_marker = geometry::TriangleMesh::CreateSphere(0.01);  //1 cm sphere
        box_top_center_marker->PaintUniformColor(Eigen::Vector3d(0.0, 1.0, 0.0));
        box_top_center_marker->Translate(block.block_top_center);
        geometry_ptrs.push_back(box_top_center_marker);

        Eigen::Vector3d end_point = block.block_top_center + block.block_normal * 1.0;
        //pretend like it is perfectly horizontal:
        end_point[2] = block.block_top_center[2];
        auto normal_line = o3dpc.CreateLine(block.block_top_center, end_point);  //1 cm
        normal_line.PaintUniformColor(Eigen::Vector3d(0.0, 1.0, 0.0));
        auto normal_line_ptr = std::make_shared<geometry::LineSet>(normal_line);           
        geometry_ptrs.push_back(normal_line_ptr);
        geometry_ptrs.push_back(coord_axis);

        std::cout << "center: " << block.block_top_center << std::endl;
        std::cout << "normal: " << block.block_normal << std::endl;
        
        std::cout << angleToHorizPlane(block.block_normal) * 57.29 << std::endl;

        std::cout << "z rotation: " << o3dpc.to_degrees(block.angle_around_z) << std::endl;
        
        std::cout << "---------------------------------" << std::endl;
    }

    visualization::DrawGeometries(geometry_ptrs, 
                                      "Target Point Cloud " + std::to_string(block_list.size()));
    
    ASSERT_EQ(block_list.size(), 3);
    //ASSERT_IN_RANGE(horizontal_surface_heights[0], -0.4, -0.38);
}
/*
TEST(SquareRootTest, NegativeNos) {
    ASSERT_EQ(-1.0, squareRoot(-15.0));
    ASSERT_EQ(-1.0, squareRoot(-0.2));
}
*/
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
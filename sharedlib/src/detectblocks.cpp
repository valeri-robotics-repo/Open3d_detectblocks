#include "../include/detectblocks.h"
#include <stdio.h>

using namespace open3d;



bool SavePointCloud(std::string filename_ply, geometry::PointCloud pcd) {
    bool success = false;

    if (io::WritePointCloud(filename_ply, pcd)) {
        utility::LogInfo("Successfully wrote {}", filename_ply);
        std::cout << "Successfully saved ply!!!" << std::endl;
        success = true;
    } else {
        std::cout << "Uh Oh, I failed to saved ply!!!" << std::endl;
        utility::LogError("Failed to write {}", filename_ply);
        success = false;
    }
    return success;
}

Open3DPointCloud::Open3DPointCloud(double _block_size, bool _use_plane, DebugLevel _debug_level ) : 
    block_size(_block_size), use_plane(_use_plane), debug_level(_debug_level) {

}

void Open3DPointCloud::overrideMinClusterSize(int mcs) {
    min_cluster_size = mcs;
}

Eigen::Vector3d Open3DPointCloud::AverageOfVector3d(std::vector<Eigen::Vector3d> vectorVector3d){

  auto sumOfVectors = accumulate(vectorVector3d.begin(), vectorVector3d.end(),
                                                    Eigen::Vector3d(0.0,0.0,0.0));
  auto avgOfVectors = sumOfVectors / vectorVector3d.size();
  return avgOfVectors;
}

int Open3DPointCloud::FindLargestCloudIndex(const std::vector<std::vector<size_t>> _indexes){
  int biggest_cloud_idx = -1;
  int cur_size = 0;
  for (int _i = 0; _i < _indexes.size(); _i ++ ){
                  auto cluster_indexes = _indexes[_i];
                  auto cluster_size = cluster_indexes.size();
                  //std::cout << cluster_size << std::endl;
                  if (cluster_size > cur_size){
                    biggest_cloud_idx = _i;
                    cur_size = cluster_size;
                    //std::cout << "CUR SIZE:" << cur_size << std::endl;
                  }
                }
  return biggest_cloud_idx;
}

void Open3DPointCloud::PrintPlaneEquation( Eigen::Vector4d plane_equation){
  std::cout << "SURFACE PLANE Equation" << plane_equation[0] << "x "
                      << plane_equation[1] << "y " << plane_equation[2] << "z "
                      << plane_equation[3]
                      << "(Constant offset)" 
                      << std::endl;
}

bool Open3DPointCloud::SavePointCloud(std::string filename_ply, geometry::PointCloud pcd) {
    
    bool success = false;
    
    if (io::WritePointCloud(filename_ply, pcd)) {
        //utility::LogInfo("Successfully wrote {}", filename_ply);
        std::cout << "Successfully saved ply!!!" << std::endl;
        success = true;
    } else {
        std::cout << "Uh Oh, I failed to saved ply!!!" << std::endl;
        //utility::LogError("Failed to write {}", filename_ply);
        success = false;
    }
    return success;
}

//For the point cloud we are getting, the floor is z (2), < 0.2 ...
std::tuple<std::vector<size_t>, std::vector<size_t>, std::vector<size_t>>
    Open3DPointCloud::SeperateHorizandVertNorms(const std::vector<Eigen::Vector3d> &points,
                          const std::vector<Eigen::Vector3d> &norms,
                          const double floor) {
    std::vector<size_t> floor_indexes;
    std::vector<size_t> horiz_indexes;
    std::vector<size_t> vert_indexes;

    size_t closest_idx = -1;

    double norm_min = 0.7;
    double norm_max = 0.9;

    for (size_t idx = 0; idx < norms.size(); idx++) {
        auto &point = points[idx];
        auto &norm = norms[idx];
        //std::cout << "z value:  " << norm(2) << std::endl;
        // get closest using only x, z:
        if (point(2) < floor) {
            if ((abs(norm(0)) < norm_min && (abs(norm(2)) > norm_max) &&
                 abs(norm(1)) < norm_min)) {
                floor_indexes.push_back(idx);
            }
        } else { //if (point(1) > ceiling) {
            if (point.norm() != 0.0) {
              if (abs(norm(2)) > norm_max) {
                horiz_indexes.push_back(idx);
              } else if ( abs(norm(2)) < norm_min) {
                vert_indexes.push_back(idx);
              }
            }
        }
    }
    #ifdef DEBUG
        std::cout << "FLOOR INDEXES:  " << floor_indexes.size() << std::endl;
        std::cout << "HORIZ INDEXES:  " << horiz_indexes.size() << std::endl;
        std::cout << "VERTICAL INDEXES:  " << vert_indexes.size() << std::endl;
    #endif
    return std::make_tuple(floor_indexes, horiz_indexes, vert_indexes);
}

std::vector<std::vector<size_t>> Open3DPointCloud::FilterSegmentIndexes(std::vector<int> horiz_cloud_segments ){
  std::vector<std::vector<size_t>> cloud_segments_indexes;

  if (horiz_cloud_segments.size() > 0) {

    auto y = sort_indexes(horiz_cloud_segments);
    //std::cout << "horiz_cloud_segments.size() " <<horiz_cloud_segments.size() <<std::endl;
    //std::cout << "y.size() " << y.size() <<std::endl;
    size_t prev_idx_of_point = y[0];
    auto prev_label_value =
                        horiz_cloud_segments[y[0]];  // Should almost always be

    std::vector<size_t> cloud_indexes;
    for (size_t idx_y = 0; idx_y < horiz_cloud_segments.size();
         idx_y++) {
         // y gives you the index from horiz_cloud_segments
         // horiz_cloud_segments gives the label value
                    
          size_t idx_of_point = y[idx_y];
            //std::cout << "  idx_of_point " << idx_of_point << std::endl;
            auto label_value = horiz_cloud_segments[idx_of_point];
            //a value of -1 is noise.  Exlcude it.
            if (label_value < 0) continue;
            if (label_value != prev_label_value) {
                prev_label_value = label_value;
                cloud_segments_indexes.push_back(cloud_indexes);
                cloud_indexes.clear();
            } else {
                cloud_indexes.push_back(idx_of_point);
            }
        }
        if (cloud_indexes.size() > 0) {
          cloud_segments_indexes.push_back(cloud_indexes);
        }
     }
     return cloud_segments_indexes;
}

double Open3DPointCloud::PointToCloudMinDistance(geometry::PointCloud& pcd_down, Eigen::Vector3d tgtpt){

      std::vector< Eigen::Vector3d > pts;
      pts.push_back(tgtpt);
      geometry::PointCloud one_point(pts);

      auto dists = pcd_down.ComputePointCloudDistance(one_point);
      std::cout << dists[0] << std::endl;
      auto min_dist =*min_element(dists.begin(), dists.end());
      return min_dist;
}

std::vector<size_t> Open3DPointCloud::index_subset(std::vector<size_t> v1, std::vector<size_t> subsetofv1){
  std::vector<size_t> sub;
      for(
          std::vector<size_t>::iterator pit = subsetofv1.begin();
          pit != subsetofv1.end();
          pit++){
            sub.push_back( v1[*pit]);
      }
      return sub;

}


Eigen::Vector3d Open3DPointCloud::GetObjectBounds(const geometry::PointCloud &_segment, Eigen::Vector3d &min_bounds,
  Eigen::Vector3d &max_bounds){  //unmutable by reference.
    Eigen::Vector3d diff_bounds(0.0,0.0,0.0);
    
    if (_segment.points_.size() < 3) std::cout << 
        "INSUFFICIENT POINTS IN CLOUD TO OBTAIN BOUNDS." << _segment.points_.size() << std::endl;
    try{
      geometry::OrientedBoundingBox target_block_obb = _segment.GetOrientedBoundingBox();
      min_bounds = target_block_obb.GetMinBound();
      max_bounds = target_block_obb.GetMaxBound();
      diff_bounds = max_bounds - min_bounds;
      if (debug_level >= DebugLevel::Verbal) {
          std::cout << "   MIN: " << target_block_obb.GetMinBound() << std::endl;
          std::cout << "   MAX: " << target_block_obb.GetMaxBound() << std::endl;
          std::cout << "   Diff: " << diff_bounds << std::endl;
      }
    } catch (std::exception &ex) {
          std::cout << "GetObjectBounds Error:  " << ex.what()
                      << std::endl;
                      return diff_bounds;
    }
    return diff_bounds;
}

std::vector<size_t> Open3DPointCloud::ExtractLargestPlane(const geometry::PointCloud &plane_cloud_ptr,
    Eigen::Vector4d &plane_equation, 
    std::shared_ptr<geometry::PointCloud> &leftovers_cloud_ptr){
    
    Eigen::Vector4d surface_plane;
    std::vector<size_t> surface_plane_indexes;

    std::tie(plane_equation, surface_plane_indexes) = plane_cloud_ptr.SegmentPlane(0.015, 4, 150);
    if (debug_level >= DebugLevel::Verbal) {
      PrintPlaneEquation(plane_equation);
    }

    
    
    //SELECT ONLY THE PLANE POINTS: 
    auto surface_cloud_ptr = plane_cloud_ptr.SelectByIndex(surface_plane_indexes);
    if (debug_level == 2){
      visualization::DrawGeometries({surface_cloud_ptr}, "Horizontal Surface ...");
    }
    return surface_plane_indexes;   
}

geometry::LineSet Open3DPointCloud::CreateLine(Eigen::Vector3d start, Eigen::Vector3d end){
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector2i> lines;

    points.push_back(start);
    points.push_back(end);
    lines.push_back(Eigen::Vector2i(0,1));
    geometry::LineSet ls(points, lines);
    return ls;
}

//This is the secret sauce...  Cluster by the normals and find the strongest normals cluster
//Create a new point could of the normals as points and run the DBSCAN_cluster on it...
Eigen::Vector3d Open3DPointCloud::IsolateLargestVerticalSide(const geometry::PointCloud &verticalfaces_cloud,
    std::shared_ptr<geometry::PointCloud>& best_normals_cloud){
    Eigen::Vector3d avg_norm = Eigen::Vector3d(0.0,0.0,0.0);
    geometry::PointCloud normals_cloud(verticalfaces_cloud.normals_);
    
    double normdist_eps = 0.05; //this is the normals distance, so this can be bigger or smaller depending upon the accuracy of your camera...
    std::vector<int> normals_clusters;
    if (normals_cloud.points_.size() >= 4 ){
      normals_clusters = normals_cloud.ClusterDBSCAN(normdist_eps, 4, false);
    }
    else {
      return Eigen::Vector3d(0.0, 0.0, 0.0);
    }
    std::vector<std::vector<size_t>> normals_clusters_indexes =  FilterSegmentIndexes(normals_clusters);

    //map indexes back to cloud:
    int biggest_cloud_idx = FindLargestCloudIndex(normals_clusters_indexes);

    if (biggest_cloud_idx < 0) {
      std::cout << "It was all noise :(" << std::endl;
      //try again,,,
      normdist_eps = 0.10;
      normals_clusters = normals_cloud.ClusterDBSCAN(normdist_eps, 4, false);
      normals_clusters_indexes =  FilterSegmentIndexes(normals_clusters);
      biggest_cloud_idx = FindLargestCloudIndex(normals_clusters_indexes);
      if (biggest_cloud_idx < 0) {
        return Eigen::Vector3d(0.0, 0.0, 0.0);
      }
    }
    auto normal = normals_clusters_indexes[biggest_cloud_idx];
    if (normal.size()> 0){
      best_normals_cloud = verticalfaces_cloud.SelectByIndex(normal);
      
      if (use_plane){
         //Get the PLANE:
        Eigen::Vector4d surface_plane_equation;
        std::vector<size_t> surface_plane_indexes;
        std::tie(surface_plane_equation, surface_plane_indexes) = best_normals_cloud->SegmentPlane(0.01, 4, 150);
        //PLANE OR AVERAGE, not both...
        if (debug_level >= 1) PrintPlaneEquation(surface_plane_equation);
        avg_norm = Eigen::Vector3d(surface_plane_equation[0], surface_plane_equation[1], surface_plane_equation[2]);
      }
      else{ //use_average
        avg_norm = AverageOfVector3d(best_normals_cloud->normals_);
      }
      if (debug_level >= 1){
        std::cout << "AVERAGE NORMAL: " << avg_norm << std::endl;
      }
    }
    else {
      std::cout << "Insufficient Normal Face Size." << std::endl;
    }
    return avg_norm;
}


void Open3DPointCloud::DebugVerticalNormalFace(geometry::PointCloud &best_normals_cloud,
  const Eigen::Vector3d vertical_side_normal){
  std::vector<std::shared_ptr<const geometry::Geometry>> block_ptrs;
  //Select a point on the vertical face:
  if (!best_normals_cloud.HasPoints()){
    return;
  }
  size_t a_point_idx = best_normals_cloud.points_.size()/2;
  auto a_point = best_normals_cloud.points_[a_point_idx];

  auto normal_line = CreateLine(a_point, a_point + vertical_side_normal*0.01);  //1 cm
  normal_line.PaintUniformColor(Eigen::Vector3d(0.10, 0.92, 0.8));
  auto normal_line_ptr = std::make_shared<geometry::LineSet>(normal_line);           
  block_ptrs.push_back( normal_line_ptr);
  best_normals_cloud.PaintUniformColor( debug_colors[0]  );
  block_ptrs.push_back(std::make_shared<geometry::PointCloud>(best_normals_cloud));

  visualization::DrawGeometries(block_ptrs, "Largest Vertical Normals Segment...", 640, 480, 50, 50, true);
}


std::shared_ptr<geometry::PointCloud> Open3DPointCloud::SegmentHorizontalSurface(
        geometry::PointCloud &pcd_down,
        Eigen::Vector3d target_object_center,
        std::vector<double> &horizontal_surface_height,
        geometry::AxisAlignedBoundingBox &target_aabb
        ) {
    try {
        
        if (!pcd_down.HasNormals()) {
          pcd_down.EstimateNormals( );
        }
        //This makes all normals outside:
        pcd_down.OrientNormalsTowardsCameraLocation(Eigen::Vector3d(0,0,0));
        std::vector<size_t> floor_indexes, horiz_indexes, vert_indexes;
        std::tie(floor_indexes, horiz_indexes, vert_indexes) =  SeperateHorizandVertNorms(pcd_down.points_, pcd_down.normals_, FLOOR_THRESHOLD);

        int color_index = 0;
        if (horiz_indexes.size() > 0) {
            //SELECT ONLY THE HORIZONTAL POINTS: 
            std::shared_ptr<geometry::PointCloud> horiz_cloud_ptr = pcd_down.SelectByIndex(horiz_indexes);
            if (debug_level == 2){
              visualization::DrawGeometries({horiz_cloud_ptr}, "Cloud reduced to horizontal normals only..."); 
            }
            Eigen::Vector4d horiz_plane_equation;
            std::shared_ptr<geometry::PointCloud> leftovers_cloud_ptr;
            auto surface_plane_indexes = ExtractLargestPlane( *horiz_cloud_ptr, horiz_plane_equation, leftovers_cloud_ptr);

            //Remove everything below the Largest horizontal plane:


            //Remap the selected indexes back to the original point cloud:
            std::vector<size_t> selected_horiz_indexes = index_subset(horiz_indexes, surface_plane_indexes);
    
            //Inverse selection of surface plane points from original cloud:
            leftovers_cloud_ptr = pcd_down.SelectByIndex(selected_horiz_indexes, true);
            if (debug_level == 2){
              visualization::DrawGeometries({leftovers_cloud_ptr}, "Leftover points should contain target ..."); 
            }

            horizontal_surface_height.push_back(horiz_plane_equation[3]);
            
            //Segment the leftovers
            double leftover_eps = 0.01; 
            if (leftovers_cloud_ptr->points_.size() < 12){
              std::cout << "OOH BAD" << std::endl;
            }
            std::vector<int> leftover_segments =  leftovers_cloud_ptr->ClusterDBSCAN(leftover_eps, 12, false);
            
            //sort each cluster
            std::vector<std::vector<size_t>> leftover_segments_indexes = FilterSegmentIndexes(leftover_segments);

            for (auto &cluster_indexes: leftover_segments_indexes){
              if (cluster_indexes.size() < min_cluster_size){  //Garbage
                continue;
              }
              auto leftovers_segment = leftovers_cloud_ptr->SelectByIndex(cluster_indexes);
              
              //THis is a cluster:
              Eigen::Vector3d min_bounds, max_bounds;
              Eigen::Vector3d diff_bounds = GetObjectBounds(*leftovers_segment, min_bounds, max_bounds);
            }
        } else {
            std::cout << "No horizontal points, Skipping." << std::endl;
        }        
    } catch (std::exception &ex) {
        std::cout << "SegmentPointCloud Error:  " << ex.what()
                    << std::endl;
    }
    
    return nullptr;
}

void Open3DPointCloud::VisualizeSegmentedCloud(std::vector<size_t> floor_indexes,
  std::vector<size_t> horiz_indexes, 
  std::vector<size_t> vert_indexes,
  geometry::PointCloud pcd_down  /*Pass a copy as not to alter the original*/
  ){
    
    std::shared_ptr<geometry::PointCloud> horiz_cloud_ptr = pcd_down.SelectByIndex(horiz_indexes);
    horiz_cloud_ptr->PaintUniformColor(Eigen::Vector3d(1.0,0.0, 0.0));

    std::shared_ptr<geometry::PointCloud> vert_cloud_ptr = pcd_down.SelectByIndex(vert_indexes);
    vert_cloud_ptr->PaintUniformColor(Eigen::Vector3d(0.0,1.0, 0.0));

    std::shared_ptr<geometry::PointCloud> floor_cloud_ptr = pcd_down.SelectByIndex(floor_indexes);
    floor_cloud_ptr->PaintUniformColor(Eigen::Vector3d(0.0, 0.0, 1.0));

    visualization::DrawGeometries({horiz_cloud_ptr, vert_cloud_ptr, floor_cloud_ptr}, "Horizontal (red),  Vertical (green), Floor (blue) "); 

}


void Open3DPointCloud::SegmentBlocks(
        geometry::PointCloud &pcd_down,
        std::vector<double> &horizontal_surface_height,
        std::vector<DetectedBlock> &block_list,
        double floor_threshold
        ) {
    try {

        FLOOR_THRESHOLD = floor_threshold;

        if (debug_level >= DebugLevel::Verbal) std::cout << "BLOCK DETECTION V1.0.2" << std::endl;

        if (!pcd_down.HasNormals()) {
          pcd_down.EstimateNormals();
        }
        pcd_down.OrientNormalsTowardsCameraLocation(Eigen::Vector3d(0, 0, 0));
        
        std::vector<size_t> floor_indexes, horiz_indexes, vert_indexes;
        std::tie(floor_indexes, horiz_indexes, vert_indexes) =  SeperateHorizandVertNorms(pcd_down.points_, pcd_down.normals_, FLOOR_THRESHOLD);

        if (debug_level == DebugLevel::Visual){
          VisualizeSegmentedCloud( floor_indexes, horiz_indexes, vert_indexes, pcd_down);
        }

        //SELECT ONLY THE VERTICAL POINTS: 
        std::shared_ptr<geometry::PointCloud> vert_cloud_ptr = pcd_down.SelectByIndex(vert_indexes);
        if (debug_level == 2){
          visualization::DrawGeometries({vert_cloud_ptr}, "Cloud reduced to VERTICAL normals only..."); 
        }

        int color_index = 0;
        if (horiz_indexes.size() > 0) {
            //SELECT ONLY THE HORIZONTAL POINTS: 
            std::shared_ptr<geometry::PointCloud> horiz_cloud_ptr = pcd_down.SelectByIndex(horiz_indexes);
            if (debug_level == 2){
              visualization::DrawGeometries({horiz_cloud_ptr}, "Cloud reduced to horizontal normals only..."); 
            }

            Eigen::Vector4d horiz_plane_equation;
            std::shared_ptr<geometry::PointCloud> leftovers_cloud_ptr;
            auto surface_plane_indexes = ExtractLargestPlane(*horiz_cloud_ptr, horiz_plane_equation, leftovers_cloud_ptr);
            

            //Remap the selected indexes back to the original point cloud:
            std::vector<size_t> selected_horiz_indexes = index_subset(horiz_indexes, surface_plane_indexes);
    
            //Inverse selection of surface plane points from original cloud:
            leftovers_cloud_ptr = pcd_down.SelectByIndex(selected_horiz_indexes, true);
            if (debug_level == DebugLevel::Visual){
              visualization::DrawGeometries({leftovers_cloud_ptr}, "Leftover points (after horiz surface removed) should contain target ..."); 
            }

            horizontal_surface_height.push_back(-1.0 * horiz_plane_equation[3]);
            if (debug_level >= DebugLevel::Verbal) std::cout << "horizontal_surface_height: " << -1 * horiz_plane_equation[3] << std::endl;
            largest_plane_surface_height = -1 * horiz_plane_equation[3];


            //Crop the point cloud so you only have points above the surface:
            const Eigen::Vector3d center(0.0, 0.0, largest_plane_surface_height);
            Eigen::Matrix3d R = Eigen::Matrix<double, 3, 3>::Identity();
            Eigen::Vector3d extent(2.0, 2.0, 0.2);
            open3d::geometry::OrientedBoundingBox obb(center, R, extent);

            leftovers_cloud_ptr = leftovers_cloud_ptr->Crop(obb);

            if (debug_level == DebugLevel::Visual){
              visualization::DrawGeometries({leftovers_cloud_ptr}, "Cropped point cloud"); 
            }

            
            //Segment the leftovers
            double leftover_eps = 0.01; 
            std::vector<int> leftover_segments;
            if (leftovers_cloud_ptr->points_.size() > 4){
               leftover_segments =  leftovers_cloud_ptr->ClusterDBSCAN(leftover_eps, 4, false);
            }
            else{
                return;
            }
            //sort each cluster
            std::vector<std::vector<size_t>> leftover_segments_indexes = FilterSegmentIndexes(leftover_segments);

            for (auto const &cluster_indexes : leftover_segments_indexes){
              if (cluster_indexes.size() < min_cluster_size){  //Garbage
                continue;
              }

              auto leftovers_segment = leftovers_cloud_ptr->SelectByIndex(cluster_indexes);
              //if (debug_level == DebugLevel::Visual) {
              //    visualization::DrawGeometries({ leftovers_segment }, "Cloud Cluster:  " + std::to_string(leftovers_segment->points_.size()), 640, 480, 50, 50, true);
              //}
              
              Eigen::Vector3d min_bounds, max_bounds;
              Eigen::Vector3d diff_bounds = GetObjectBounds(*leftovers_segment, min_bounds, max_bounds);
              
              if (diff_bounds[0] == 0.0) continue;
              //if (max_bounds[2] < horizontal_surface_height[0]) {
              //  if (debug_level >= DebugLevel::Verbal)  std::cout << "Skipping due to bad surface height!" << horizontal_surface_height[0] - block_size << " MIN:  " << min_bounds[2]  << std::endl;
              //  //continue;
              //}
              if (diff_bounds[0] < max_block_size && diff_bounds[1] < max_block_size && diff_bounds[2] < max_block_size){
                if (debug_level >= DebugLevel::Verbal) std::cout << "Found potential block. " << std::endl;

                if (debug_level == DebugLevel::Visual) {
                    visualization::DrawGeometries({ leftovers_segment }, "Potential Block:  " + std::to_string(leftovers_segment->points_.size()), 640, 480, 50, 50, true);
                }

                //try segmenting the faces:
                std::vector<size_t> floor_indexes_block, horiz_indexes_block, vert_indexes_block;
                std::tie(floor_indexes_block, horiz_indexes_block, vert_indexes_block) =  
                  SeperateHorizandVertNorms(leftovers_segment->points_, leftovers_segment->normals_, FLOOR_THRESHOLD);

                
                bool require_vertical_side = true;
                //Probably a real block should have at least some horizontal and vertical indexes...
                if (horiz_indexes_block.size() <= 10 ){

                  if (debug_level >= DebugLevel::Verbal) 
                    std::cout << "Insufficient Horizontal Surface" << std::endl;
                  continue;  //Not a block...
                }else{
                   if (require_vertical_side && vert_indexes_block.size()==0)
                      continue;  //Not a block...
                }

                DetectedBlock block = ExtractBlock(horiz_indexes_block, 
                        vert_indexes_block,
                        leftovers_segment,
                        largest_plane_surface_height);


                if (block.block_top_center.norm() > 0.0){
                  if (debug_level == DebugLevel::Visual) {
                    visualization::DrawGeometries({ leftovers_segment }, "ACTUAL Block:  " + std::to_string(leftovers_segment->points_.size()), 640, 480, 50, 50, true);
                  }
                  
                  block_list.push_back(block);
                }
              }
              else{
                if (debug_level >= DebugLevel::Verbal) {
                  std::cout << "Skipping non block DUE TO SIZE." << std::endl;
                }
              }
            }
        } else {
            std::cout << "No horizontal points, Skipping." << std::endl;
        }        
    } catch (std::exception &ex) {
        std::cout << "SegmentPointCloud Error:  " << ex.what()
                    << std::endl;
    }
}


DetectedBlock Open3DPointCloud::ExtractBlock(std::vector<size_t> &horiz_indexes, 
  std::vector<size_t> &vert_indexes,
  std::shared_ptr<geometry::PointCloud> leftovers_segment,
  double surface_height){
  DetectedBlock block;
  block.block_top_center = Eigen::Vector3d(0.0, 0.0, 0.0);

  
  Eigen::Vector4d block_top_plane;
  std::vector<size_t> block_top_plane_indexes;
  std::shared_ptr<geometry::PointCloud> horizface_cloud_ptr = leftovers_segment->SelectByIndex(horiz_indexes);



  if (debug_level == DebugLevel::Visual){
    visualization::DrawGeometries({horizface_cloud_ptr}, "ALL HORIZONTAL (with normals)...", 640, 480, 50, 50, true); 
  }
  Eigen::Vector3d block_top_center = horizface_cloud_ptr->GetCenter();
  
  if (block_top_center[2] < (surface_height - (block_size * 1.5))){
    return block;
  }
  
  //Tweak as needed to suit the accuracy of your depth camera...
  if ( horizface_cloud_ptr->points_.size() < 10 ) return block;
  std::tie(block_top_plane, block_top_plane_indexes) = horizface_cloud_ptr->SegmentPlane(0.005, 10, 2);
  if (debug_level >= DebugLevel::Verbal) {
      PrintPlaneEquation(block_top_plane);
  }
  
  if (debug_level == DebugLevel::Verbal){
    auto box_top_center_marker = geometry::TriangleMesh::CreateSphere(0.001);  //1 mm sphere
    box_top_center_marker->PaintUniformColor(debug_colors[0]);
    box_top_center_marker->Translate(block_top_center);
    if (debug_level == DebugLevel::Visual){
      visualization::DrawGeometries({horizface_cloud_ptr, box_top_center_marker}, "HORIZ FACE..." + std::to_string(block_top_plane[3])); 
    }
  }
  
  std::shared_ptr<geometry::PointCloud> verticalfaces_cloud_ptr = leftovers_segment->SelectByIndex(vert_indexes);

  if (debug_level == DebugLevel::Visual){
    visualization::DrawGeometries({verticalfaces_cloud_ptr}, "ALL VERTICAL BLOCK FACES (with normals)...", 640, 480, 50, 50, true); 
  }

  std::shared_ptr<geometry::PointCloud> best_normals_cloud;
  Eigen::Vector3d vertical_side_normal = Eigen::Vector3d(0.0, 0.0, 0.0);

  if (vert_indexes.size() > 4){
    vertical_side_normal = IsolateLargestVerticalSide(*verticalfaces_cloud_ptr, best_normals_cloud);
  }
  if (vertical_side_normal.norm() == 0.0) return block;
  if (debug_level == DebugLevel::Visual){
    DebugVerticalNormalFace(*best_normals_cloud, vertical_side_normal);
  }

  double sumOfZ = 0.0;

  for(auto &x: verticalfaces_cloud_ptr->points_){   
    sumOfZ += x[2];
  }
  double avgOfZ = sumOfZ/verticalfaces_cloud_ptr->points_.size();
  //Check for inside corner vs outside corner.  Blocks will always have outside corner
  if (avgOfZ > block_top_center[2] ){
    if (debug_level >= DebugLevel::Verbal) std::cout << "Probably not a block :( " << avgOfZ << std::endl;
    return block;
  }

  if (debug_level >= DebugLevel::Verbal) std::cout << "You made it!  You are a block! " << std::endl;
  block.block_top_center = block_top_center;
                block.block_normal = vertical_side_normal;
                auto angle_around_z = std::atan(vertical_side_normal[0]/vertical_side_normal[1]);
                if (debug_level > 0) std::cout << "angle_around_z: " << angle_around_z << std::endl;
                block.angle_around_z = angle_around_z;

  return block;
}


double Open3DPointCloud::to_degrees(double rad){
  return rad * (180.0/3.1415);
}


std::tuple<Eigen::Vector3d, Eigen::Vector3d> Open3DPointCloud::rotationMatrixToEulerAngles(Eigen::Matrix4d R) {
    Eigen::Vector3d angles;
    Eigen::Vector3d transform;

    double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
    if (sy < 1e-6) {
        angles[0]  = atan2(R(2, 1), R(2, 2));
        angles[1] = atan2(-R(2, 0), sy);
        angles[2] = atan2(R(1, 0), R(0, 0));

    } else {
        angles[0] = atan2(-R(1, 2), R(1, 1));
        angles[1] = atan2(-R(2, 0), sy);
        angles[2] = 0;
    }


    transform[0] = R(0, 3) * 100.0;
    transform[1] = R(1, 3) * 100.0;
    transform[2] = R(2, 3) * 100.0;

    return std::tie<Eigen::Vector3d, Eigen::Vector3d>(angles, transform);
}

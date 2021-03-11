/**
 * @brief Implementation for Plane Estimator
 * - reads a .pcd pointcloud file (assuming only points of a single plane are present)
 * - calculates the equation of a plane in it analytically 
 * - displays -
 *      - original pointcloud
 *      - calculated plane
 *      - normal on plane
 *      - cuboid representing camera over plane 
 * @usage ./pcd_plane_calculation <path_to_pcd_file>
 * 
 * @author Parth Chopra
 * @date   03-01-2021
 * @email  parthc@umich.edu 
 * 
 * Credit for template : https://github.com/jeffdelmerico/pointcloud_tutorial.git
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <string>
#include <Eigen/Dense>
#include <math.h>
#include <stdlib.h>
#include <time.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "pcl_visualizer_helper.cc"

const Eigen::IOFormat fmt(2, Eigen::DontAlignCols, "\t", " ", "", "", "", "");
class PlaneSegment {
  Eigen::Vector3f cloud_mean;
  Eigen::Vector3f normal;
  Eigen::Vector4f plane_parameters;
  float xx = 0.0, xy = 0.0, xz = 0.0, yy = 0.0, yz = 0.0, zz = 0.0;
  std::vector<float> det;
  pcl::visualization::PCLVisualizer::Ptr viewer;
  
  public:
        PlaneSegment() : cloud_mean(Eigen::Vector3f(0,0,0)), plane_parameters(Eigen::Vector4f(0,0,0,0)), normal(Eigen::Vector3f(0,0,0)) {
          srand(time(NULL));
        }
        Eigen::Vector3f getCloudMean(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
        void makeCloudZeroMean(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
        void restoreCloudFromZeroMean(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud); 
        void viewCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

        void getPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
        void printPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

        Eigen::Vector3f getNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud); 
        Eigen::Vector3f getNormal(const Eigen::Vector4f plane_parameters);
        Eigen::Vector3f getNormal(const Eigen::MatrixXf covariance);
        Eigen::Vector4f getPlaneParametersRANSAC(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int max_iterations = 100, float distance_threshold = 0.002);
 
        float getPlaneError(Eigen::Vector4f plane, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<int> points_to_fit);
        float getPlaneError(Eigen::Vector4f plane, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

        float pointToPlaneDistance(Eigen::Vector4f plane, Eigen::Vector3f point, Eigen::Vector3f point_on_plane);
        float pointToPlaneDistance(Eigen::Vector4f plane, pcl::PointXYZ point_pcl, pcl::PointXYZ point_on_plane);
        Eigen::Vector4f getPlaneProposalLeastSquares(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<int> points_to_fit);
        std::vector<int> getRandomPointIndices(int max_index, int n);

        Eigen::MatrixXf getFullCovariance(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

        void getClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
};

/* 
 * Make Clusters
 */
void PlaneSegment::getClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>) ;
  vg.setInputCloud (cloud);
  //vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.setLeafSize (0.1f, 0.1f, 0.1f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int nr_points = (int) cloud_filtered->size ();
  while (cloud_filtered->size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }
  this->viewer = simpleVis(cloud, *coefficients);

  //--------------------
  // -----Main loop-----
  //--------------------
  //std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

  std::chrono::milliseconds ms{100};
  while (!this->viewer->wasStopped ())
  {
    this->viewer->spinOnce (100);
    std::this_thread::sleep_for(ms);
  }
    //// Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    //// Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

    std::stringstream ss;
    ss << "cloud_plane_1" << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_plane, false); //*

    std::cout << "wrote cloud_plane_1.pcd";
    //this->viewCloud(cloud_plane);
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

 // // Creating the KdTree object for the search method of the extraction
 // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
 // tree->setInputCloud (cloud_filtered);

 // std::vector<pcl::PointIndices> cluster_indices;
 // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
 // ec.setClusterTolerance (0.08); // 2cm
 // ec.setMinClusterSize (100);
 // ec.setMaxClusterSize (80000);
 // ec.setSearchMethod (tree);
 // ec.setInputCloud (cloud_filtered);
 // ec.extract (cluster_indices);

 // int j = 0;
 // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
 // {
 //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
 //   for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
 //     cloud_cluster->push_back ((*cloud_filtered)[*pit]); //*
 //   cloud_cluster->width = cloud_cluster->size ();
 //   cloud_cluster->height = 1;
 //   cloud_cluster->is_dense = true;

 //   //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
 //   std::stringstream ss;
 //   ss << "cloud_cluster_" << j << ".pcd";
 //   //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
 //   //this->viewCloud(cloud_cluster);
 //   j++;
 // }
}
  //////////////

Eigen::Vector3f PlaneSegment::getCloudMean(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
 // double point_weight = 1.0 / cloud->points.size ();
 // std::cout << "Point weight : " << point_weight<<std::endl; 
 //  for (size_t i = 0; i < cloud->points.size (); ++i) {
 //   this->cloud_mean[0] += cloud->points[i].x * point_weight;
 //   this->cloud_mean[1] += cloud->points[i].y * point_weight;
 //   this->cloud_mean[2] += cloud->points[i].z * point_weight;
 //  }
 //  std::cout << "Calculated Centroid :    " << this->cloud_mean[0]
 //             << " "    << this->cloud_mean[1]
 //             << " "    << this->cloud_mean[2] << std::endl;
 pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D (*cloud, minPt, maxPt);
  //std::cout << "Max x: " << maxPt.x << std::endl;
  //std::cout << "Max y: " << maxPt.y << std::endl;
  //std::cout << "Max z: " << maxPt.z << std::endl;
  //std::cout << "Min x: " << minPt.x << std::endl;
  //std::cout << "Min y: " << minPt.y << std::endl;
  //std::cout << "Min z: " << minPt.z << std::endl;
  std::cout << "X range: " << maxPt.x - minPt.x <<" [ "<<minPt.x<<", "<<maxPt.x<<" ]\n";
  std::cout << "Y range: " << maxPt.y - minPt.y <<" [ "<<minPt.y<<", "<<maxPt.y<<" ]\n";
  std::cout << "Z range: " << maxPt.z - minPt.z <<" [ "<<minPt.z<<", "<<maxPt.z<<" ]\n";
    Eigen::Vector4f centroid;
    if (! pcl::compute3DCentroid(*cloud, centroid) ) {
      std::cout<<"invalid centroid";
    }
    std::cout<<"valid centroid\n";
    this->cloud_mean[0] = centroid[0];
    this->cloud_mean[1] = centroid[1];
    this->cloud_mean[2] = centroid[2];
    
   std::cout << "Centroid :    " << this->cloud_mean[0]
              << " "    << this->cloud_mean[1]
              << " "    << this->cloud_mean[2] << std::endl;
   std::cout<< "\nNorm, d is :" <<this->cloud_mean.norm();
   this->plane_parameters[3] = 0.0;//this->cloud_mean.norm();
   return this->cloud_mean;
}

void PlaneSegment::makeCloudZeroMean(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  this->getCloudMean(cloud);
  for (size_t i = 0; i < cloud->points.size (); ++i) {
    cloud->points[i].x -= this->cloud_mean[0];
    cloud->points[i].y -= this->cloud_mean[1];
    cloud->points[i].z -= this->cloud_mean[2];
  }
  //float temp;
  //for (size_t i = 0; i < cloud->points.size (); ++i) {
  //  temp = cloud->points[i].x;
  //  cloud->points[i].x = cloud->points[i].z;
  //  cloud->points[i].z = temp;
  //}
  //for (size_t i = 0; i < cloud->points.size (); ++i) {
  //  temp = cloud->points[i].x;
  //  cloud->points[i].x = cloud->points[i].y;
  //  cloud->points[i].y = cloud->points[i].z;
  //  cloud->points[i].z = temp;
  //}

}
void PlaneSegment::restoreCloudFromZeroMean(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  for (size_t i = 0; i < cloud->points.size (); ++i) {
    cloud->points[i].x += this->cloud_mean[0];
    cloud->points[i].y += this->cloud_mean[1];
    cloud->points[i].z += this->cloud_mean[2];
  }
}

std::vector<int> PlaneSegment::getRandomPointIndices(int max_index, int n) {
  std::vector<int> random_indices;
  while (n > 0) {
    int random_number = rand() % max_index;
    if (find(random_indices.begin(), random_indices.end(), random_number) == random_indices.end()) { 
      random_indices.push_back(random_number); // 0 to max_index
      n--;
    }
  }
  return random_indices;
}

Eigen::MatrixXf PlaneSegment::getFullCovariance(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  Eigen::MatrixXf covariance = Eigen::MatrixXf::Constant(3,3,0);
  for (size_t i = 0; i < cloud->points.size (); ++i) {
    covariance(0, 0) += cloud->points[i].x * cloud->points[i].x;
    covariance(0, 1) += cloud->points[i].x * cloud->points[i].y;
    covariance(0, 2) += cloud->points[i].x * cloud->points[i].z;
    covariance(1, 1) += cloud->points[i].y * cloud->points[i].y;
    covariance(1, 2) += cloud->points[i].y * cloud->points[i].z;
    covariance(2, 2) += cloud->points[i].z * cloud->points[i].z;
  }
  covariance(1, 0) = covariance(0, 1);
  covariance(2, 0) = covariance(0, 2);
  covariance(2, 1) = covariance(1, 2);
  covariance /= cloud->points.size ();
  return covariance;
}
Eigen::Vector4f PlaneSegment::getPlaneProposalLeastSquares(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<int> points_to_fit) {

  Eigen::Vector4f plane_in_world;
  Eigen::MatrixXf A = Eigen::MatrixXf::Constant(points_to_fit.size(), 3, 1.0);
  Eigen::VectorXf B = Eigen::VectorXf::Constant(points_to_fit.size(), 1.0);

  Eigen::Vector3f centroid(0, 0, 0);

  //pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  //std::cout<<" \n# Points in Cloud : "<<cloud->points.size()<<" \n# Points to fit : "<<points_to_fit.size()<<" indices : ";
  //for ( auto idx : points_to_fit )
  //  std::cout<<idx;
  //std::cout<<std::endl;

  int counter = 0;
  for ( int i : points_to_fit) {
    //std::cout<<" \ni : "<<i << " x,y,z: "<<cloud->points[i].x<<", "<<cloud->points[i].y<<", "<<cloud->points[i].z<<"\n";
    //plane_cloud->insert( plane_cloud->end(), cloud->points[i] );
    A(counter, 0) = cloud->points[i].x;
    A(counter, 1) = cloud->points[i].y;
    B[counter] = cloud->points[i].z;

    centroid[0] += cloud->points[i].x;
    centroid[1] += cloud->points[i].y;
    centroid[2] += cloud->points[i].z;  
    counter++;
  }
  //std::cout<<"\n A : \n\n"<<A;
  //std::cout<<"\n B : \n\n"<<B;

  //Eigen::Vector3f plane_abc = A.colPivHouseholderQr().solve(B);
  Eigen::Vector3f plane_abc = A.completeOrthogonalDecomposition().solve(B);
  //std::cout<<"\n QR_pinv(A)*B : \n\n"<<plane_abc;
  plane_abc = plane_abc.normalized();
  //std::cout<<"\n Normal from PInv : \n\n"<<plane_abc;
  plane_in_world[0] = plane_abc[0];
  plane_in_world[1] = plane_abc[1];
  plane_in_world[2] = plane_abc[2];
  centroid /= points_to_fit.size();
  std::cout<<"\n centroid : [ "<<centroid[0]<<", "<<centroid[1]<<", "<<centroid[2]<<"] ";
  plane_in_world[3] = plane_abc.dot(centroid);
    
  return plane_in_world; 
}

float PlaneSegment::pointToPlaneDistance(Eigen::Vector4f plane, Eigen::Vector3f point, Eigen::Vector3f point_on_plane) {
  return (point - point_on_plane).dot(this->getNormal(plane));
}
float PlaneSegment::pointToPlaneDistance(Eigen::Vector4f plane, pcl::PointXYZ point_pcl, pcl::PointXYZ point_on_plane_pcl) {
  Eigen::Vector3f point(point_pcl.x, point_pcl.y, point_pcl.z);
  Eigen::Vector3f point_on_plane(point_on_plane_pcl.x, point_on_plane_pcl.y, point_on_plane_pcl.z);
  return this->pointToPlaneDistance(plane, point, point_on_plane);
}
float PlaneSegment::getPlaneError(Eigen::Vector4f plane, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  float total_error = 0.0;
  int random_point_reference = this->getRandomPointIndices(cloud->points.size(), 1)[0];
  for (auto pt : cloud->points) {
    total_error += this->pointToPlaneDistance( plane, pt, cloud->points[random_point_reference] );
  }
  return std::abs(total_error);
}
float PlaneSegment::getPlaneError(Eigen::Vector4f plane, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<int> points_to_fit) {
  float total_error = 0.0;
  int random_point_reference = this->getRandomPointIndices(cloud->points.size(), 1)[0];
  for (auto index : points_to_fit) {
    total_error += this->pointToPlaneDistance( plane, cloud->points[index], cloud->points[random_point_reference] );
  }
  return std::abs(total_error);
}
Eigen::Vector4f PlaneSegment::getPlaneParametersRANSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int max_iterations, float distance_threshold) {
  int iteration = 0, sample_point_size = min(int(cloud->points.size()), 3), sample_size_threshold = min(int(cloud->points.size()), 10);

  Eigen::Vector4f best_plane_fit, plane_proposal;
  float best_error = 99.9, current_error;
  int max_inliers = 0;
  std::vector<int> proposed_inliers, also_inliers;
  while (iteration < max_iterations)  {
    std::cout<<"\n Iteration "<<iteration;
    proposed_inliers.clear();
    proposed_inliers = this->getRandomPointIndices(cloud->points.size(), sample_point_size);
    std::cout<<" -- "<<proposed_inliers.size()<<" proposed inliers -- ";

    plane_proposal = this->getPlaneProposalLeastSquares(cloud, proposed_inliers);
    current_error = this->getPlaneError( plane_proposal, cloud, proposed_inliers);
    std::cout<<"\n\n Proposed Plane : [ "<<plane_proposal[0]<<", "<<plane_proposal[1]
            <<", "<<plane_proposal[2]<<", "<<plane_proposal[3]<<"]  with error: "
            <<current_error<<std::endl;
    //if (current_error < 0.1)
    //  return plane_proposal;

    also_inliers.clear();
    for (int i = 0; i < cloud->points.size(); i++) {
      if (find(proposed_inliers.begin(), proposed_inliers.end(), i) == proposed_inliers.end()) {
        //if (this->getPlaneError(plane_proposal, cloud, std::vector<int>(i)) < distance_threshold)
        if (this->pointToPlaneDistance(plane_proposal, cloud->points[i], cloud->points[proposed_inliers[0]]) < distance_threshold) {
          also_inliers.push_back(i);
        }
      }
    }
    //if (current_error < best_error) {
    //  best_plane_fit = plane_proposal;
    //  std::cout<<"\n\n Best Plane yet: [ "<<plane_proposal[0]<<", "<<plane_proposal[1]
    //        <<", "<<plane_proposal[2]<<", "<<plane_proposal[3]<<"] with error: "
    //      <<current_error<<std::endl;
    //  best_error = current_error;
    //}
    std::cout<<" -- "<<also_inliers.size()<<" inliers added -- ";

    if (also_inliers.size() > sample_size_threshold) {
      // fit plane to all inliers
      proposed_inliers.insert(proposed_inliers.end(), also_inliers.begin(), also_inliers.end());
      std::cout<<" -- "<<proposed_inliers.size()<<" total inliers -- ";
      // better plane proposal
      plane_proposal =  this->getPlaneProposalLeastSquares( cloud, proposed_inliers );
      current_error = this->getPlaneError( plane_proposal, cloud, proposed_inliers);

      //if (current_error < 0.1)
    //  return plane_proposal;
      //if (current_error < best_error) {
      if (proposed_inliers.size() > max_inliers) {
        max_inliers = proposed_inliers.size();
        best_plane_fit = plane_proposal;
        std::cout<<"\n\n Improved Best Plane: [ "<<plane_proposal[0]<<", "<<plane_proposal[1]
              <<", "<<plane_proposal[2]<<", "<<plane_proposal[3]<<"] with error: "
            <<current_error<<std::endl;
        best_error = current_error;
      }
      else {
        std::cout<<"\n\n Improved Plane : [ "<<plane_proposal[0]<<", "<<plane_proposal[1]
              <<", "<<plane_proposal[2]<<", "<<plane_proposal[3]<<"] with error: "
            <<current_error<<std::endl;
      }
    }
    iteration++;
  }
  std::cout<<"\n\n Output Plane : [ "<<best_plane_fit[0]<<", "<<best_plane_fit[1]
            <<", "<<best_plane_fit[2]<<", "<<best_plane_fit[3]<<"] with error: "
          <<this->getPlaneError( best_plane_fit, cloud, proposed_inliers)<< " on inliers : "<<proposed_inliers.size();
  return best_plane_fit; 
}

void PlaneSegment::printPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  std::cout<<"\n--------\n Plane visualized : [ "<<this->plane_parameters[0]<<", "<<this->plane_parameters[1]
              <<", "<<this->plane_parameters[2]<<", "<<this->plane_parameters[3]<<"] with error : "<<this->getPlaneError( this->plane_parameters, cloud);
}
void PlaneSegment::getPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) { 

  //pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  //pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  //// Create the segmentation object
  //pcl::SACSegmentation<pcl::PointXYZ> seg;
  //// Mandatory
  //seg.setModelType (pcl::SACMODEL_PLANE);
  //seg.setMethodType (pcl::SAC_RANSAC);
  //seg.setDistanceThreshold (0.01);
  //seg.setInputCloud (cloud);
  //seg.segment (*inliers, *coefficients);
  //this->plane_parameters[0] = coefficients->values[0];
  //this->plane_parameters[1] = coefficients->values[1];
  //this->plane_parameters[2] = coefficients->values[2];
  //this->plane_parameters[3] = coefficients->values[3];

  this->plane_parameters = this->getPlaneParametersRANSAC(cloud);
  this->normal = this->getNormal(this->plane_parameters);
  std::cout<<"\n--------\n Plane visualized : [ "<<this->plane_parameters[0]<<", "<<this->plane_parameters[1]
              <<", "<<this->plane_parameters[2]<<", "<<this->plane_parameters[3]<<"] with error : "<<this->getPlaneError( this->plane_parameters, cloud);
  //this->det = std::vector<float>{0.0, 0.0, 0.0};
  //this->det[0] = this->yy*this->zz - this->yz*this->yz;
  //this->det[1] = this->xx*this->zz - this->xz*this->xz;
  //this->det[2] = this->xx*this->yy - this->xy*this->xy;

  //Eigen::Vector3f dir(0.0, 0.0, 0.0);

  //int det_max = std::max_element(this->det.begin(), this->det.begin() + 3) - this->det.begin(); 

  //switch(det_max) {
  //  case 0:
  //    dir[0] = this->det[0];
  //    dir[1] = this->xz * this->yz - this->xy * this->zz;
  //    dir[2] = this->xy * this->yz - this->xz * this->yy;
  //    break;
  //  case 1:
  //    dir[0] = this->xz * this->yz - this->xy * this->zz;
  //    dir[1] = this->det[1];
  //    dir[2] = this->xy * this->xz - this->yz * this->xx;
  //    break;
  //  case 2:
  //    dir[0] = this->xy * this->yz - this->xz * this->yy;
  //    dir[1] = this->xy * this->xz - this->yz * this->xx;
  //    dir[2] = this->det[2];
  //    break;
  //}
  //dir = dir.normalized();  
  //this->normal = dir;
  //std::cout << "Normal is :    " << dir[0]
  //            << " "    << dir[1]
  //            << " "    << dir[2] << std::endl;
  //this->plane_parameters[0] = ( this->xy * this->yz - this->xz * this->yy ) / ( this->xx*this->yy - this->xy*this->xy );
  //this->plane_parameters[1] = ( this->xy * this->xz - this->xx * this->yz ) / ( this->xx*this->yy - this->xy*this->xy );
  //this->plane_parameters[2] = 1.0;
  //this->plane_parameters[3] = 0.0;//this->plane_parameters[0] * this->cloud_mean[0] + this->plane_parameters[1] * this->cloud_mean[1] + 1.0 * this->cloud_mean[2] ;
  //std::cout<<" \n For Plane, a, b, c, d :" << this->plane_parameters[0] <<" "
  //                                        << this->plane_parameters[1] <<" "
  //                                        << this->plane_parameters[2] <<" "
  //                                        << this->plane_parameters[3] <<" \n\n";
}

Eigen::Vector3f PlaneSegment::getNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  //this->getPlaneAtMean(cloud);
  return this->getNormal(this->getPlaneParametersRANSAC(cloud));
  //return this->normal;
}

Eigen::Vector3f PlaneSegment::getNormal(const Eigen::Vector4f plane_parameters) {
  Eigen::Vector3f normal(plane_parameters[0], plane_parameters[1], plane_parameters[2]); 
  //std::cout<<" \n For Plane, a, b, c, d :" << plane_parameters[0] <<" "
  //                                        << plane_parameters[1] <<" "
  //                                        << plane_parameters[2] <<" "
  //                                        << plane_parameters[3] <<" \n\n";

  return normal.normalized();
}

Eigen::Vector3f PlaneSegment::getNormal(const Eigen::MatrixXf covariance) {
  Eigen::Vector3f normal(0, 0, 0);
  
  float det_x = covariance(1,1) * covariance(2,2) - pow(covariance(1,2),2);
  Eigen::Vector3f axis_dir_x(
    det_x,
    covariance(0,2)*covariance(1,2) - covariance(0,1)*covariance(2,2),
    covariance(0,1)*covariance(1,2) - covariance(0,2)*covariance(1,1)); 
  float weight = pow(det_x, 2);

  if ( normal.dot(axis_dir_x) < 0.0 ) { weight *= -1; }
  normal += axis_dir_x * weight;

  float det_y = covariance(0,0) * covariance(2,2) - pow(covariance(0,2),2);
  Eigen::Vector3f axis_dir_y(
    covariance(0,2)*covariance(1,2) - covariance(0,1)*covariance(2,2),
    det_y,
    covariance(0,1)*covariance(0,2) - covariance(1,2)*covariance(0,0)); 
  weight = pow(det_y, 2);

  if ( normal.dot(axis_dir_y) < 0.0 ) { weight *= -1; }
  normal += axis_dir_y * weight;
 
  float det_z = covariance(0,0) * covariance(1,1) - pow(covariance(0,1),2);
  Eigen::Vector3f axis_dir_z(
    covariance(0,1)*covariance(1,2) - covariance(0,2)*covariance(1,1),
    covariance(0,1)*covariance(0,2) - covariance(1,2)*covariance(0,0),
    det_z); 
  weight = pow(det_z, 2);

  if ( normal.dot(axis_dir_z) < 0.0 ) { weight *= -1; }
  normal += axis_dir_z * weight;
  return normal.normalized();
}

void PlaneSegment::viewCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  pcl::ModelCoefficients plane_coeff;
  
  plane_coeff.values.resize (4);    // We need 4 values
  plane_coeff.values[0] = this->plane_parameters.x();
  plane_coeff.values[1] = this->plane_parameters.y();
  plane_coeff.values[2] = this->plane_parameters.z();
  plane_coeff.values[3] = this->plane_parameters.w();

  pcl::ModelCoefficients camera_coeff;
 
  camera_coeff.values.resize (7);    // We need 7 values
  camera_coeff.values[0] = -1.0*this->normal[0];//this->cloud_mean[0] ;//+ 1.0*this->normal[0]; // x-origin
  camera_coeff.values[1] = -1.0*this->normal[1];//this->cloud_mean[1] ;//+ 1.0*this->normal[1]; // y-origin
  camera_coeff.values[2] = -1.0*this->normal[2];//this->cloud_mean[2] ;//+ 1.0*this->normal[2]; // z-origin
  camera_coeff.values[3] = 0.5*this->normal[0];//this->cloud_mean[0]; // relative x-end
  camera_coeff.values[4] = 0.5*this->normal[1];//this->cloud_mean[1]; // relative y-end
  camera_coeff.values[5] = 0.5*this->normal[2];//this->cloud_mean[2]; // relative z-end
  camera_coeff.values[6] = 5.0; // radius

  std::cout<<"\n Displaying Plane + Cone\n";
  //std::cout<<"\n Cone size: "<< camera_coeff.values.size();
  this->viewer = simpleVis(cloud, plane_coeff, camera_coeff);

  // // Display axis for camera + line for vector needing to appear upright
  // // use line for vector appearing upright to do axis-angle rotation with normal, create line + get axis at same position as camera cone
  //this->viewer->addCone (camera_coeff, "cone");

  std::cout<<"\n Normal : "<<this->normal[0]<<", "<<this->normal[1]<<", "<<this->normal[2]<<"\n ";
  // Add vector on plane
  Eigen::Vector3f vector_on_plane = Eigen::Vector3f(0, 0, 1).cross(this->normal);
  std::cout<<"\n vector_on_plane : "<<vector_on_plane[0]<<", "<<vector_on_plane[1]<<", "<<vector_on_plane[2]<<"\n "; // coplanar vector
  //camera_coeff.values[0] = this->cloud_mean[0]; // x-origin
  //camera_coeff.values[1] = this->cloud_mean[1]; // y-origin
  //camera_coeff.values[2] = this->cloud_mean[2]; // z-origin
  //camera_coeff.values[3] = vector_on_plane[0];//-0.5*this->normal[0];//this->cloud_mean[0]; // relative x-end
  //camera_coeff.values[4] = vector_on_plane[1];//-0.5*this->normal[1];//this->cloud_mean[1]; // relative y-end
  //camera_coeff.values[5] = vector_on_plane[2];//-0.5*this->normal[2];//this->cloud_mean[2]; // relative z-end
  //this->viewer->addCone (camera_coeff, "vector_cone");  

  float yaw_vector_on_plane = 0.0;
  float axis_angle = atan((Eigen::Vector2f(this->normal[0], this->normal[1]).norm())/this->normal[2]);

  //std::cout<< " \n Angle between plane and origin frame : " <<axis_angle;

  Eigen::Affine3f m;
  Eigen::AngleAxis<float> aa( -axis_angle, Eigen::Vector3f(1, 0, 0) );
  m  = Eigen::Translation3f(this->cloud_mean) * aa;

  Eigen::Quaternion<float> q;  q = aa;

  Eigen::AngleAxis<float> aa_yaw_on_plane( yaw_vector_on_plane,  Eigen::Vector3f(0, 0, 1)); // yaw for vector on plane
  m *= aa_yaw_on_plane;

  q = aa*aa_yaw_on_plane;
  //this->viewer->addCoordinateSystem (1.0, m, "second");

  this->viewer->addCube(-1.0*this->normal , q, 2*0.16, 2*0.1, 0.02, "camera_view");
//pcl::visualization::createCube (const Eigen::Vector3f &translation, const Eigen::Quaternionf &rotation,
//                                double width, double height, double depth)
  //--------------------
  // -----Main loop-----
  //--------------------
  std::chrono::milliseconds ms{100};
  while (!this->viewer->wasStopped ())
  {
    this->viewer->spinOnce (100);
    std::this_thread::sleep_for(ms);
  }
}
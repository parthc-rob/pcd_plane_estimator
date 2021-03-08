/**
 * @brief Extracts planes from .pcd file, writes respective planes to .pcd files
 * @usage ./plane_extractor <path_to_pcd_file>
 * 
 * @author Parth Chopra
 * @date   03-01-2021
 * @email  parthc@umich.edu 
 *
 */

#include "pcl_plane_calculation_lib.cc"

void getClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string filename_prefix, float leafsize = 0.05f);
 
int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::string file = "test_pcd.pcd";

  float voxelgridsize = 0.05f;
  if (argc>1)
    file = argv[1];
  if (argc>2)
    voxelgridsize = std::stof(argv[2]);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;

  getClusters(cloud, file.substr(0, file.find(".")), voxelgridsize);

  return (0);
}
/* 
 * Make Clusters
 */
void getClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string filename_prefix, float leafsize) {
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>) ;
  vg.setInputCloud (cloud);
  vg.setLeafSize (leafsize, leafsize, leafsize);
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
  int j = 0;

  pcl::visualization::PCLVisualizer::Ptr viewer;

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
  viewer = simpleVis(cloud, *coefficients);

  //--------------------
  // -----Main loop-----
  //--------------------
  std::chrono::milliseconds ms{100};
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
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
    ss << filename_prefix<<"_"<< j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_plane, false); //*

    std::cout << "\nwrote "<< filename_prefix<<"_"<< j << ".pcd";
    //this->viewCloud(cloud_plane);
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
    j++;
  }
}

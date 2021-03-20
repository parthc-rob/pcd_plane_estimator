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

#include "pcl_plane_calculation_lib.cc"

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::string file = "test_pcd.pcd";
  if (argc>1)
    file = argv[1];   

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;

  PlaneSegment plane;
  //plane.getCloudMean(cloud);
  plane.makeCloudZeroMean(cloud);
  plane.getPlane(cloud);
  //plane.restoreCloudFromZeroMean(cloud);

  //plane.printPlane(cloud);
  plane.viewCloud(cloud);

  return (0);
}

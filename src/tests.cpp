/**
 * @brief Runs tests for functions in headers 
 * @usage ./tests
 * 
 * @author Parth Chopra
 * @date   03-01-2021
 * @email  parthc@umich.edu 
 *
 */
#include <gtest/gtest.h>
#include <math.h>
#include "pcl_plane_calculation_lib.cc"


#include <pcl/features/normal_3d.h> // ground truthing

pcl::PointCloud<pcl::PointXYZ>::Ptr xy_cloud (new pcl::PointCloud<pcl::PointXYZ>);

PlaneSegment plane;

float norm_error_threshold = 1e-2;

////////////////////////////////////////////////////////////////////////////////////////////////
TEST (CentroidCalculationTest, PlanarPointCloud)
{
  xy_cloud->clear();
  xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (2, 2, 0));
  xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (0, 0, 0));
  xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (2, 0, 0));
  xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (0, 2, 0));

  ASSERT_TRUE ( ( plane.getCloudMean(xy_cloud) - Eigen::Vector3f(1, 1, 0) ).norm() < norm_error_threshold );
}
// Ref : https://www.geogebra.org/3d?lang=en
TEST (NormalCalculationTest, Plane1) {
    Eigen::Vector4f plane_parameters(1, 0, 0, 0);
    Eigen::Vector3f normal_gt(1, 0, 0);

    std::cout<<"\n Plane normal : " << plane.getNormal(plane_parameters).format(fmt);
    std::cout<<"\n Normal error : " <<( plane.getNormal(plane_parameters) - normal_gt ).format(fmt)
             <<std::endl<<( plane.getNormal(plane_parameters) - normal_gt ).norm()
             <<std::endl;
    ASSERT_TRUE ( ( plane.getNormal(plane_parameters) - normal_gt ).norm() < norm_error_threshold );
}
TEST (NormalCalculationTest, Plane2) {
    Eigen::Vector4f plane_parameters(1, 1, 1, 0);
    Eigen::Vector3f normal_gt(0.58, 0.58, 0.58);

    std::cout<<"\n Plane normal : " << plane.getNormal(plane_parameters).format(fmt);
    std::cout<<"\n Normal error : " <<( plane.getNormal(plane_parameters) - normal_gt ).format(fmt)
             <<std::endl<<( plane.getNormal(plane_parameters) - normal_gt ).norm()
             <<std::endl;
    ASSERT_TRUE ( ( plane.getNormal(plane_parameters) - normal_gt ).norm() < norm_error_threshold );
}
TEST (NormalCalculationTest, Plane3) {
    Eigen::Vector4f plane_parameters(1, -1, -1, 0);
    Eigen::Vector3f normal_gt(0.58, -0.58, -0.58);

    std::cout<<"\n Plane normal : " << plane.getNormal(plane_parameters).format(fmt);
    std::cout<<"\n Normal error : " <<( plane.getNormal(plane_parameters) - normal_gt ).format(fmt)
             <<std::endl<<( plane.getNormal(plane_parameters) - normal_gt ).norm()
             <<std::endl;
    ASSERT_TRUE ( ( plane.getNormal(plane_parameters) - normal_gt ).norm() < norm_error_threshold );
}
TEST (NormalCalculationTest, Plane4) {
    Eigen::Vector4f plane_parameters(1, 0, 0, 0.5);
    Eigen::Vector3f normal_gt(1, 0, 0);

    std::cout<<"\n Plane normal : " << plane.getNormal(plane_parameters).format(fmt);
    std::cout<<"\n Normal error : " <<( plane.getNormal(plane_parameters) - normal_gt ).format(fmt)
             <<std::endl<<( plane.getNormal(plane_parameters) - normal_gt ).norm()
             <<std::endl;
    ASSERT_TRUE ( ( plane.getNormal(plane_parameters) - normal_gt ).norm() < norm_error_threshold );
}
TEST (LeastSquaresPlaneTest, PlanarPointCloud1)
{
  xy_cloud->clear();
  xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (2, 2, 1));
  xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (0, 0, 1));
  xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (2, 0, 1));
  xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (0, 2, 1));
  std::vector<int> points_to_fit{0, 1, 2, 3};

  Eigen::Vector4f plane_parameters = plane.getPlaneProposalLeastSquares(xy_cloud, points_to_fit);

  Eigen::Vector3f normal_gt(0, 0, 1);
  ASSERT_TRUE ( ( plane.getNormal(plane_parameters) - normal_gt ).norm() < norm_error_threshold );
}
TEST (LeastSquaresPlaneTest, PlanarPointCloud2)
{
  xy_cloud->clear();
  xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (1, 0, 0));
  xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (1, 0, 2));
  xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (1, 2, 2));
  xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (1, 2, 0));
  std::vector<int> points_to_fit{0, 1, 2, 3};

  Eigen::Vector4f plane_parameters = plane.getPlaneProposalLeastSquares(xy_cloud, points_to_fit);

  Eigen::Vector3f normal_gt(1, 0, 0);
  ASSERT_TRUE ( ( plane.getNormal(plane_parameters) - normal_gt ).norm() < norm_error_threshold );
}
TEST (PointToPlaneTest, PointToPlane1)
{
  Eigen::Vector4f plane_params(0, 0, 1, 1);
  Eigen::Vector3f point(0, 0, 0), point_on_plane(20, -10, 1);
  float distance_gt = 1.0;
  ASSERT_TRUE ( plane.pointToPlaneDistance(plane_params, point, point_on_plane) - distance_gt < norm_error_threshold );
}
TEST (PointToPlaneTest, PointToPlane2)
{
  Eigen::Vector4f plane_params(1, 0, 0, 3.0);
  Eigen::Vector3f point(0.5, 1, 2), point_on_plane(3, -10, 10);
  float distance_gt = 2.5;
  ASSERT_TRUE ( plane.pointToPlaneDistance(plane_params, point, point_on_plane) - distance_gt < norm_error_threshold );
}
TEST (PointToPlaneTest, PointToPlane3)
{
  Eigen::Vector4f plane_params(1, 0, 1, 3.0);
  Eigen::Vector3f point(0, 0, 0), point_on_plane(0, 0, 3);
  float distance_gt = 3*sqrt(2);
  ASSERT_TRUE ( plane.pointToPlaneDistance(plane_params, point, point_on_plane) - distance_gt < norm_error_threshold );
}
// TEST (NormalUsingCovarianceTest, PlanarPointCloud1_ransac)
// {
//   xy_cloud->clear();
//   xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (2, 2, 1));
//   xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (0, 0, 1));
//   xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (2, 0, 1));
//   xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (0, 2, 1));
// 
//   std::cout<<"\n Plane normal : " << plane.getNormal(plane.getFullCovariance(xy_cloud));
//   std::cout<<"\n Normal error : " <<( plane.getNormal(plane.getFullCovariance(xy_cloud)) - Eigen::Vector3f(0.58, -0.58, -0.58) ).norm()
//            <<std::endl;
// 
//   Eigen::Vector3f normal_gt(0, 0, 1);
//   ASSERT_TRUE ( ( plane.getNormal(plane.getFullCovariance(xy_cloud)) - normal_gt ).norm() < norm_error_threshold );
// }
// TEST (NormalUsingCovarianceTest, PlanarPointCloud2_ransac)
// {
//   xy_cloud->clear();
//   xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (1, 0, 0));
//   xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (1, 0, 2));
//   xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (1, 2, 2));
//   xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (1, 2, 0));
// 
//   std::cout<<"\n Plane normal : " << plane.getNormal(plane.getFullCovariance(xy_cloud));
// 
//   Eigen::Vector3f normal_gt(1, 0, 0);
//   std::cout<<"\n Ground truth : " << normal_gt <<std::endl;
//   ASSERT_TRUE ( ( plane.getNormal(plane.getFullCovariance(xy_cloud)) - normal_gt ).norm() < norm_error_threshold );
// }
// TEST (NormalUsingCovarianceTest, PlanarPointCloud3_ransac)
// {
//   xy_cloud->clear();
//   xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (0, 0, 1));
//   xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (1, 0, 0));
//   xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (0, 1, 1));
//   xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (1, 1, 0));
//   //xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (3, -2, 0));
// 
//   std::cout<<"\n Plane normal : " << plane.getNormal(plane.getFullCovariance(xy_cloud))<<std::endl;
// 
//   Eigen::Vector3f normal_gt(0.71, 0, 0.71);
//   std::cout<<"\n Ground truth : " << normal_gt <<std::endl;
//   ASSERT_TRUE ( ( plane.getNormal(plane.getFullCovariance(xy_cloud)) - normal_gt ).norm() < norm_error_threshold );
// }
TEST (PlaneProposalCalculationTest, PlanarPointCloud1_ransac)
{
  xy_cloud->clear();
  xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (2, 2, 1));
  xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (0, 0, 1));
  xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (2, 0, 1));
  xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (0, 2, 1));

  std::cout<<"\n Plane normal : " << plane.getNormal(xy_cloud);
  std::cout<<"\n Normal error : " <<( plane.getNormal(xy_cloud) - Eigen::Vector3f(0.58, -0.58, -0.58) ).norm()
           <<std::endl;

  Eigen::Vector3f normal_gt(0, 0, 1);
  ASSERT_TRUE ( ( plane.getNormal(xy_cloud) - normal_gt ).norm() < norm_error_threshold );
}
TEST (PlaneProposalCalculationTest2, PlanarPointCloud2_ransac)
{
  xy_cloud->clear();
  xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (1, 0, 0));
  xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (1, 0, 2));
  xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (1, 2, 2));
  xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (1, 2, 0));

  std::cout<<"\n Plane normal : " << plane.getNormal(xy_cloud);

  Eigen::Vector3f normal_gt(1, 0, 0);
  std::cout<<"\n Ground truth : " << normal_gt <<std::endl;
  ASSERT_TRUE ( ( plane.getNormal(xy_cloud) - normal_gt ).norm() < norm_error_threshold );
}
TEST (PlaneProposalCalculationTest2, PlanarPointCloud3_ransac)
{
  xy_cloud->clear();
  xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (0, 0, 1));
  xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (1, 0, 0));
  xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (0, 1, 1));
  xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (1, 1, 0));
  //xy_cloud->insert (xy_cloud->end (), pcl::PointXYZ (3, -2, 0));

  std::cout<<"\n Plane normal : " << plane.getNormal(xy_cloud)<<std::endl;

  Eigen::Vector3f normal_gt(0.71, 0, 0.71);
  std::cout<<"\n Ground truth : " << normal_gt <<std::endl;
  ASSERT_TRUE ( ( plane.getNormal(xy_cloud) - normal_gt ).norm() < norm_error_threshold );
}
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
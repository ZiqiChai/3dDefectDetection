//C++
#include <iostream>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

//boost
#include <boost/thread/thread.hpp>

//OpenCV
#include "opencv2/imgproc/imgproc.hpp"

int
 main (int argc, char** argv)
{
  // Loading first scan of room.
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("room_scan1.pcd", *target_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << target_cloud->size () << " data points from room_scan1.pcd" << std::endl;

  // Loading second scan of room from new perspective.
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("room_scan2.pcd", *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << input_cloud->size () << " data points from room_scan2.pcd" << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
  approximate_voxel_filter.setInputCloud (input_cloud);
  approximate_voxel_filter.filter (*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size ()
            << " data points from room_scan2.pcd" << std::endl;



  // Set initial alignment estimate found using robot odometry.
  Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
  Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

  double start = (double)(cv::getTickCount());
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations (1000);
  icp.setInputSource(filtered_cloud);
  icp.setInputTarget(target_cloud);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align (Final, init_guess);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  double end = ((double)cv::getTickCount() - start) / cv::getTickFrequency();
  std::cout << "所用时间" << end <<std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*input_cloud, *output_cloud, icp.getFinalTransformation ());

  // Saving transformed input cloud.
  pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);

  // Initializing point cloud visualizer
  boost::shared_ptr<pcl::visualization::PCLVisualizer>
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);

  // Coloring and visualizing target cloud (red).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  target_color (target_cloud, 255, 0, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "target cloud");

  // Coloring and visualizing transformed input cloud (green).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  output_color (output_cloud, 0, 255, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud");

  // Starting visualizer
  viewer_final->addCoordinateSystem (1.0, 0 );
  // viewer_final->addCoordinateSystem (1.0, "global");
  viewer_final->initCameraParameters ();

  // Wait until visualizer window is closed.
  while (!viewer_final->wasStopped ())
  {
    viewer_final->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }



 return (0);
}

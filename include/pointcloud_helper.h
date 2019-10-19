#ifndef _ZIQI_POINTCLOUD_HELPER_
#define _ZIQI_POINTCLOUD_HELPER_
//C++
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <map>
#include <fstream>
#include <cmath>
#include <math.h>
#include <algorithm>

//OpenCV
#include <opencv2/opencv.hpp>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/intersections.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

//Eigen
#include <Eigen/Eigen>

//Customed
#include "pointcloud_helper.h"

#ifndef PI
#define PI 3.1415926535897932384626433832795028841971693993751058209749445
#endif
#ifndef pi
#define pi 3.1415926535897932384626433832795028841971693993751058209749445
#endif

using namespace cv;
using namespace std;
using namespace pcl;

cv::Mat getR2registeZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
int loadPointCloudData(string inputFilename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
Eigen::Matrix4f E2R(const double alpha, const double beta, const double gamma);
void getCenter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float &x, float &y, float &z);
Eigen::Matrix4f planeAlign(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
pcl::Normal getLoopNormal(
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
	const pcl::PointCloud<pcl::Normal>::Ptr& normal,
	pcl::PointXYZ& outer_max,
	pcl::PointXYZ& outer_min,
	pcl::PointXYZ& iner_max,
	pcl::PointXYZ& iner_min);

#endif

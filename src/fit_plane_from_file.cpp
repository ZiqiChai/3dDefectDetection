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
#include<pointcloud_helper.h>

#ifndef PI
#define PI 3.1415926535897932384626433832795028841971693993751058209749445
#endif
#ifndef pi
#define pi 3.1415926535897932384626433832795028841971693993751058209749445
#endif

using namespace cv;
using namespace std;
using namespace pcl;


int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	string inputFilename=argv[1];
	if(loadPointCloudData(inputFilename, cloud) != 0)
		exit(-1);
	cv::Mat cv_R = getR2registeZ(cloud);
	return 0;
}
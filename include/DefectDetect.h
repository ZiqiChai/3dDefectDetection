#ifndef DEFECTDETECT_H
#define DEFECTDETECT_H

//C++
#include <stdlib.h>
#include <string>
#include <sstream>

//boost
#include <boost/thread/thread.hpp>

//PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>

//OpenCV
#include <opencv2/opencv.hpp>

//Customed
#include "config.h"
#include "pointcloud_helper.h"
#include "dbscan.h"

using namespace std;
using namespace cv;

#ifndef PI
#define PI 3.1415926535897932384626433832795028841971693993751058209749445
#endif
#ifndef pi
#define pi 3.1415926535897932384626433832795028841971693993751058209749445
#endif
int process(pcl::PointCloud<pcl::PointXYZ>::Ptr origin_cloud);
#endif
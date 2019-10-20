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
#include "DefectDetect.h"

using namespace std;
using namespace cv;

#ifndef PI
#define PI 3.1415926535897932384626433832795028841971693993751058209749445
#endif
#ifndef pi
#define pi 3.1415926535897932384626433832795028841971693993751058209749445
#endif

int main(int argc,char**argv)
{
	string paraFileName = "../parameters/parameter.yml";
	Config::setParameterFile(paraFileName);

	//创建点云对象
	pcl::PointCloud<pcl::PointXYZ>::Ptr origin_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//加载点云数据，命令行参数为文件名，文件可以是.PCD/.pcd/.PLY/.ply四种文件之一，单位可以是mm也可以是m.
	string inputFilename=argv[1];
	if(loadPointCloudData(inputFilename, origin_cloud) != 0)
		exit(-1);

	process(origin_cloud);
	return 0;
}
//load PCD and save PLY Data
//C++
#include <iostream>
//PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

int main(int argc, char** argv)
{
	//创建点云对象
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//读取PCD文件
	if(pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) !=0)
	{
		PCL_ERROR("Couldn't read PCD file \n");
		return -1;
	}
	std::cout << "Loaded " << cloud->width * cloud->height << " data points from PCD file " << std::endl;
	// for (size_t i = 0; i < cloud->points.size(); ++i)
	// 	std::cout << " " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << "\r";
	pcl::io::savePLYFileASCII(argv[2], *cloud);
	return (0);
}
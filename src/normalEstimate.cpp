#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include "config.h"
#include <stdlib.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/transforms.h>

#include <opencv2/opencv.hpp>

//Customed
#include<pointcloud_helper.h>

using namespace std;
using namespace cv;

#ifndef PI
#define PI 3.1415926535897932384626433832795028841971693993751058209749445
#endif
#ifndef pi
#define pi 3.1415926535897932384626433832795028841971693993751058209749445
#endif

clock_t begin, end;

int cnt = 0;
double xmin = 0;
double ymin = 0;
double zmin = 0;
double xmax = 0;
double ymax = 0;
double zmax = 0;
int intensity = 50;
double dis = 0;
bool isUnitMillimetre = false;


Eigen::Matrix4f E2R(const double alpha, const double beta, const double gamma)
{
	Eigen::Matrix4f m;
	m << cos(beta)*cos(gamma), cos(gamma)*sin(alpha)*sin(beta) - cos(alpha)*sin(gamma), sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta), 0,
	cos(beta)*sin(gamma)   , cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma), cos(alpha)*sin(beta)*sin(gamma) - cos(gamma)*sin(alpha), 0,
	-sin(beta)             , cos(beta)*sin(alpha)                                 , cos(alpha)*cos(beta)                                 , 0,
	0                      , 0                                                    , 0                                                    , 1;
	return m;
}



Eigen::Matrix4f planeAlign(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Matrix4f tf;
	double a, b, c;
	Eigen::MatrixXf data = Eigen::MatrixXf::Zero(cloud->width, 3);
	tf << 1, 0, 0, 0,
	0, 1, 0, 0,
	0, 0, 1, 0,
	0, 0, 0, 1;

	Eigen::Vector3f mean;
	mean.fill(0);
	for(int i = 0; i < cloud->width; i++)
	{
		mean(0) += cloud->points[i].x;
		mean(1) += cloud->points[i].y;
		mean(2) += cloud->points[i].z;
	}
	mean /= cloud->width;
	for(int i = 0; i < cloud->width; i++)
	{
		data(i, 0) = cloud->points[i].x - mean(0);
		data(i, 1) = cloud->points[i].y - mean(1);
		data(i, 2) = cloud->points[i].z - mean(2);
	}

	Eigen::JacobiSVD<Eigen::MatrixXf> svd(data, Eigen::ComputeThinU | Eigen::ComputeThinV);

	Eigen::Matrix3f V;
	V = svd.matrixV();
	a = V(0, 2);
	b = V(1, 2);
	c = V(2, 2);
    //cout<<a<<" "<<b<<" "<<c<<endl;

	Eigen::Vector3f ver_y, ver_z, ver_plane;
	double rad_y, rad_z;
	ver_y << 0.0, 1.0, 0.0;
	ver_z << 0.0, 0.0, 1.0;
	ver_plane << a, b, c;
	rad_y = PI / 2.0 - acos(ver_plane.dot(ver_y) / (ver_plane.norm() * ver_y.norm()));
	rad_z = PI / 2.0 - acos(ver_plane.dot(ver_z) / (ver_plane.norm() * ver_z.norm()));
	tf = E2R(0, rad_z, 0) * E2R(0, 0, -rad_y);
	dis = -ver_plane.dot(mean);
	vector<double> distance;
	double temp;

	for(int i = 0; i < cloud->width; i++)
	{
		temp = abs(a * cloud->points[i].x + b * cloud->points[i].y + c * cloud->points[i].z + dis) / sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
		distance.push_back(temp);
	}

	sort(distance.begin(), distance.end());

	cout << "MAX distance between pointcloud with the plane : " << distance[distance.size() - 1] << endl;
	return tf;
}

int main(int argc,char**argv)
{
	string paraFileName = "../parameters/parameter.yml";
	Config::setParameterFile(paraFileName);

	//创建点云对象
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//创建法线的对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	//加载点云数据，命令行参数为文件名，文件可以是.PCD/.pcd/.PLY/.ply四种文件之一，单位可以是mm也可以是m.
	string inputFilename=argv[1];
	if(loadPointCloudData(inputFilename, cloud) != 0)
		exit(-1);

	//计算点云分布范围，如果分布范围大于1，说明单位是mm，小于1，说明单位是m.
	//用于存放三个轴的最小值
	pcl::PointXYZ min;
	//用于存放三个轴的最大值
	pcl::PointXYZ max;
	pcl::getMinMax3D(*cloud,min,max);
	std::cout<<"min.x = "<<min.x<<"  min.y = "<<min.y<<"  min.z = "<<min.z<<std::endl;
	std::cout<<"max.x = "<<max.x<<"  max.y = "<<max.y<<"  max.z = "<<max.z<<std::endl;
	if ( (std::abs(max.x-min.x)>1) | (std::abs(max.y-min.y)>1) | (std::abs(max.z-min.z)>1) )
	{
		isUnitMillimetre= true;
		std::cout<<"The Unit of this PointCloud is [mm]."<<std::endl;
	}
	else
	{
		isUnitMillimetre= false;
		std::cout<<"The Unit of this PointCloud is [m]."<<std::endl;
		for(pcl::PointCloud<pcl::PointXYZ>::iterator pt = cloud->points.begin(); pt < cloud->points.end(); pt++)
		{
			pt->x =1000*pt->x;
			pt->y =1000*pt->y;
			pt->z =1000*pt->z;
		}
		max.x = 1000*max.x;
		min.x = 1000*min.x;
		max.y = 1000*max.y;
		min.y = 1000*min.y;
		max.z = 1000*max.z;
		min.z = 1000*min.z;
		std::cout<<"min.x = "<<min.x<<"  min.y = "<<min.y<<"  min.z = "<<min.z<<std::endl;
		std::cout<<"max.x = "<<max.x<<"  max.y = "<<max.y<<"  max.z = "<<max.z<<std::endl;

	}


	//可视化
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud Original"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (cloud, 0, 255, 0);
	viewer->addPointCloud(cloud, single_color, "Original");
	viewer->addCoordinateSystem(0.3*(max.y-min.y));
	while(!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100));
	}


	//对点云进行平面拟合（可选先进行提速滤波和带通滤波以加快处理速度），并且将点云旋转到传感器轴与测量平面垂直
	if (Config::get<bool>("VoxelGrid_filter_before_fitting_plane") && Config::get<bool>("PASS_filter_before_fitting_plane"))
	{
		std::cout<<"VoxelGrid_filter_before_fitting_plane: "<<Config::get<bool>("VoxelGrid_filter_before_fitting_plane")<<std::endl;
		std::cout<<"PASS_filter_before_fitting_plane: "<<Config::get<bool>("PASS_filter_before_fitting_plane")<<std::endl;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> vg;
		//创建滤波对象
		vg.setInputCloud (cloud);
		//设置需要过滤的点云给滤波对象
		vg.setLeafSize (
			Config::get<float>("VoxelGrid_filter_before_fitting_plane_x"),
			Config::get<float>("VoxelGrid_filter_before_fitting_plane_y"),
			Config::get<float>("VoxelGrid_filter_before_fitting_plane_z"));
		//设置滤波时创建的体素体积为1cm的立方体
		vg.filter (*cloud_filtered);
		//执行滤波

		cv::Mat cv_R = getR2registeZ(cloud_filtered);
		Eigen::Matrix4f tf;
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				tf(i,j)=cv_R.at<double>(i,j);
			}
		}
		for (int i = 0; i < 4; ++i)
		{
			tf(i,3) = 0;
			tf(3,i) = 0;
		}
		tf(3,3)=1;
		std::cout<<"tf Matrix4f:\n"<<tf<<std::endl;
		pcl::transformPointCloud(*cloud, *cloud, tf.inverse());
		std::cout<<"tf Matrix4f:\n"<<tf<<std::endl;
	}
	else
	{
		cv::Mat cv_R = getR2registeZ(cloud);
		Eigen::Matrix4f tf;
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				tf(i,j)=cv_R.at<double>(i,j);
			}
		}
		for (int i = 0; i < 4; ++i)
		{
			tf(i,3) = 0;
			tf(3,i) = 0;
		}
		tf(3,3)=1;
		std::cout<<"tf Matrix4f:\n"<<tf<<std::endl;
		pcl::transformPointCloud(*cloud, *cloud, tf.inverse());
		std::cout<<"tf Matrix4f:\n"<<tf<<std::endl;
	}

	//可视化
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("Cloud Transformed"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2 (cloud, 0, 255, 0);
	viewer2->addPointCloud(cloud, single_color, "Transformed");
	viewer2->addCoordinateSystem(0.3*(max.y-min.y));
	while(!viewer2->wasStopped())
	{
		viewer2->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100));
	}



	if (Config::get<bool>("VoxelGrid_filter") && !Config::get<bool>("PASS_filter"))
	{
		std::cout<<"VoxelGrid_filter: "<<Config::get<bool>("VoxelGrid_filter")<<std::endl;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> vg;
		//创建滤波对象
		vg.setInputCloud (cloud);
		//设置需要过滤的点云给滤波对象
		vg.setLeafSize (Config::get<float>("VoxelGrid_x"), Config::get<float>("VoxelGrid_y"), Config::get<float>("VoxelGrid_z"));
		//设置滤波时创建的体素体积为1cm的立方体
		vg.filter (*cloud_filtered);
		//执行滤波

		//创建法线估计的对象
		pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normalEstimation;
		normalEstimation.setInputCloud(cloud_filtered);
		//对于每一个点都用半径为3cm的近邻搜索方式
		normalEstimation.setRadiusSearch(Config::get<float>("RadiusSearch"));
		//Kd_tree是一种数据结构便于管理点云以及搜索点云，法线估计对象会使用这种结构来找到哦啊最近邻点
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		normalEstimation.setSearchMethod(kdtree);
		//计算法线
		normalEstimation.compute(*normals);

		//可视化
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3(new pcl::visualization::PCLVisualizer("Cloud Normals"));
		viewer3->setBackgroundColor (0, 0, 0);
		// Coloring and visualizing target cloud (red).
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  target_color (cloud_filtered, 255, 0, 0);
		viewer3->addPointCloud<pcl::PointXYZ> (cloud_filtered, target_color, "cloud_filtered");
		viewer3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_filtered");
		viewer3->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_filtered, normals, 1, (0.1*(max.x-min.x)), "normals");
		viewer3->addCoordinateSystem(0.3*(max.y-min.y));
		while(!viewer3->wasStopped())
		{
			viewer3->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100));
		}
	}
	else if (Config::get<bool>("PASS_filter") && !Config::get<bool>("VoxelGrid_filter"))
	{
		std::cout<<"PASS_filter: "<<Config::get<bool>("PASS_filter")<<std::endl;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud (cloud);
		pass.setFilterFieldName (Config::get<string>("PASS_filter_x"));
		pass.setFilterLimits (Config::get<float>("PASS_filter_x_min"), Config::get<float>("PASS_filter_x_max"));
		std::cout<<"PASS_filter_x_min: "<<Config::get<float>("PASS_filter_x_min")<<std::endl;
		std::cout<<"PASS_filter_x_max: "<<Config::get<float>("PASS_filter_x_max")<<std::endl;

		pass.setFilterFieldName (Config::get<string>("PASS_filter_y"));
		pass.setFilterLimits (Config::get<float>("PASS_filter_y_min"), Config::get<float>("PASS_filter_y_max"));
		std::cout<<"PASS_filter_y_min: "<<Config::get<float>("PASS_filter_y_min")<<std::endl;
		std::cout<<"PASS_filter_y_max: "<<Config::get<float>("PASS_filter_y_max")<<std::endl;
		pass.filter (*cloud_filtered);

		//创建法线估计的对象
		pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normalEstimation;
		normalEstimation.setInputCloud(cloud_filtered);
		//对于每一个点都用半径为3cm的近邻搜索方式
		normalEstimation.setRadiusSearch(Config::get<float>("RadiusSearch"));
		//Kd_tree是一种数据结构便于管理点云以及搜索点云，法线估计对象会使用这种结构来找到哦啊最近邻点
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		normalEstimation.setSearchMethod(kdtree);
		//计算法线
		normalEstimation.compute(*normals);

		float max_theta_to_ZAxis =  Config::get<float>("max_theta_to_ZAxis");
		std::cout<<"max_theta_to_ZAxis is: "<<max_theta_to_ZAxis<<"[deg]."<<std::endl;
		float abs_z_expect = cos(max_theta_to_ZAxis/180*PI);
		std::vector<int> index;
		int tmp_index=0;

		for(pcl::PointCloud<pcl::Normal>::iterator pt = normals->points.begin(); pt < normals->points.end(); pt++)
		{
			if (std::abs(pt->normal_z) < abs_z_expect)
			{
				index.push_back(tmp_index);
				std::cout<<"index: "<<tmp_index <<" normal_x: "<<pt->normal_x <<" normal_y: "<<pt->normal_y
				<<" normal_z: "<<pt->normal_z <<" abs_z_expect: "<<abs_z_expect<<std::endl;
			}
			tmp_index++;
		}
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCP(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*cloud_filtered, index, *cloudCP);

		tmp_index =0;
		index.clear();
		float sum_curvature=0;
		float max_curvature=0;
		float avg_curvature=0;
		for(pcl::PointCloud<pcl::Normal>::iterator pt = normals->points.begin(); pt < normals->points.end(); pt++)
		{
			sum_curvature =sum_curvature + pt->curvature;
			if (max_curvature < pt->curvature)
			{
				max_curvature = pt->curvature;
			}
		}
		avg_curvature = sum_curvature/normals->points.size();
		// float curvature_expect = avg_curvature*Config::get<float>("curvature_expect_overshot_ratio");
		float curvature_expect = max_curvature-(max_curvature-avg_curvature)*Config::get<float>("curvature_expect_down_ratio");

		for(pcl::PointCloud<pcl::Normal>::iterator pt = normals->points.begin(); pt < normals->points.end(); pt++)
		{
			if (std::abs(pt->curvature) > curvature_expect)
			{
				index.push_back(tmp_index);
				std::cout<<"index: "<<tmp_index <<" curvature: "<< pt->curvature<<" curvature_expect: "<<curvature_expect<<std::endl;
			}
			tmp_index++;
		}
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCP2(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*cloud_filtered, index, *cloudCP2);


		float sum_z=0;
		float max_z=0;
		float avg_z=0;
		for(pcl::PointCloud<pcl::PointXYZ>::iterator pt = cloud_filtered->points.begin(); pt < cloud_filtered->points.end(); pt++)
		{
			sum_z = sum_z+ pt->z;
			if (max_z < pt->z)
			{
				max_z = pt->z;
			}
		}
		avg_z = sum_z/cloud_filtered->points.size();
		tmp_index =0;
		index.clear();
		float expect_z_overshot = avg_z + Config::get<float>("expect_z_overshot");

		for(pcl::PointCloud<pcl::PointXYZ>::iterator pt = cloud_filtered->points.begin(); pt < cloud_filtered->points.end(); pt++)
		{
			if (std::abs(pt->z) > expect_z_overshot)
			{
				index.push_back(tmp_index);
				std::cout<<"index: "<<tmp_index <<" pt->z: "<< pt->z<<" expect_z_overshot: "<<expect_z_overshot<<std::endl;
			}
			tmp_index++;
		}
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCP3(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*cloud_filtered, index, *cloudCP3);





		//可视化
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3(new pcl::visualization::PCLVisualizer("Cloud Normals"));
		viewer3->setBackgroundColor (0, 0, 0);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  target_color (cloud_filtered, 0, 255, 0);
		viewer3->addPointCloud<pcl::PointXYZ> (cloud_filtered, target_color, "cloud_filtered");
		viewer3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_filtered");

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  target_color_cloudCP (cloudCP, 255, 0, 0);
		viewer3->addPointCloud<pcl::PointXYZ> (cloudCP, target_color_cloudCP, "cloudCP");
		viewer3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloudCP");

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  target_color_cloudCP2 (cloudCP2, 0, 0, 255);
		viewer3->addPointCloud<pcl::PointXYZ> (cloudCP2, target_color_cloudCP2, "cloudCP2");
		viewer3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloudCP2");

		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  target_color_cloudCP3 (cloudCP3, 0, 255, 255);
		// viewer3->addPointCloud<pcl::PointXYZ> (cloudCP3, target_color_cloudCP3, "cloudCP3");
		// viewer3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloudCP3");


		if (Config::get<bool>("show_normal"))
		{
			viewer3->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_filtered, normals, 1, (0.1*(max.x-min.x)), "normals");
		}

		viewer3->addCoordinateSystem(0.3*(max.y-min.y));
		while(!viewer3->wasStopped())
		{
			viewer3->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100));
		}
	}
	else
	{
		//创建法线估计的对象
		pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normalEstimation;
		normalEstimation.setInputCloud(cloud);
		//对于每一个点都用半径为3cm的近邻搜索方式
		normalEstimation.setRadiusSearch(Config::get<float>("RadiusSearch"));
		//Kd_tree是一种数据结构便于管理点云以及搜索点云，法线估计对象会使用这种结构来找到哦啊最近邻点
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		normalEstimation.setSearchMethod(kdtree);
		//计算法线
		normalEstimation.compute(*normals);

		//可视化
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3(new pcl::visualization::PCLVisualizer("Cloud Normals"));
		viewer3->setBackgroundColor (0, 0, 0);
		// Coloring and visualizing target cloud (red).
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  target_color (cloud, 255, 0, 0);
		viewer3->addPointCloud<pcl::PointXYZ> (cloud, target_color, "cloud");
		viewer3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

		pcl::visualization::PointCloudColorHandlerCustom<pcl::Normal>  normal_color (normals, 255, 255, 255);
		viewer3->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 1, (0.1*(max.x-min.x)), "normals");


		viewer3->addCoordinateSystem(0.3*(max.y-min.y));

		while(!viewer3->wasStopped())
		{
			viewer3->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100));
		}
		// cv::Mat  image=cv::Mat::zeros(1080, 1920, CV_8UC3);
		// for(pcl::PointCloud<pcl::PointXYZ>::iterator pt = cloud_filtered->points.begin(); pt < cloud_filtered->points.end(); pt++)
		// {
		// 	std::cout<<pt->x <<" "<<pt->y <<" "<<pt->z <<std::endl;
		// 	std::cout<<int(5000*pt->y)<<" "<<int(5000*pt->x)<<" "<< int(5000*pt->y)+500 <<" "<< int(5000*pt->x)+500 <<std::endl;
		// 	image.at<cv::Vec3b>(int(5000*pt->y)+500,int(5000*pt->x)+500)[0]=int(5000*pt->x);
		// 	image.at<cv::Vec3b>(int(5000*pt->y)+500,int(5000*pt->x)+500)[1]=int(5000*pt->y);
		// 	image.at<cv::Vec3b>(int(5000*pt->y)+500,int(5000*pt->x)+500)[2]=int(5000*pt->z);

		// 	std::cout<<pt->x <<" "<<pt->y <<" "<<pt->z <<std::endl;
		// 	std::cout<<int(1000*pt->y)<<" "<<int(1000*pt->x)<<" "<< int(1000*pt->y)+100 <<" "<< int(1000*pt->x)+100 <<std::endl;
		// 	image.at<cv::Vec3b>(int(1000*pt->y)+100,int(1000*pt->x)+100)[0]=int(1000*pt->x);
		// 	image.at<cv::Vec3b>(int(1000*pt->y)+100,int(1000*pt->x)+100)[1]=int(1000*pt->y);
		// 	image.at<cv::Vec3b>(int(1000*pt->y)+100,int(1000*pt->x)+100)[2]=int(1000*pt->z);
		// }
		// cv::imshow("image",image);
		// cv::waitKey();
		// cv::imwrite("image.png",image);
	}
	return 0;
}
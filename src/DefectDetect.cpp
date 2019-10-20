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



int process(pcl::PointCloud<pcl::PointXYZ>::Ptr origin_cloud)
{
	// string paraFileName = "../parameters/parameter.yml";
	// Config::setParameterFile(paraFileName);

	//创建点云对象
	// pcl::PointCloud<pcl::PointXYZ>::Ptr origin_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//创建法线的对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//记录点云对齐的缸体变换矩阵
	Eigen::Matrix4f tf;

	//加载点云数据，命令行参数为文件名，文件可以是.PCD/.pcd/.PLY/.ply四种文件之一，单位可以是mm也可以是m.
	// string inputFilename=argv[1];
	// if(loadPointCloudData(inputFilename, origin_cloud) != 0)
	// 	exit(-1);
	pcl::copyPointCloud(*origin_cloud, *cloud);

	//计算点云分布范围，如果分布范围大于1，说明单位是mm，小于1，说明单位是m.
	//用于存放三个轴的最小值
	pcl::PointXYZ min;
	//用于存放三个轴的最大值
	pcl::PointXYZ max;
	pcl::getMinMax3D(*cloud,min,max);
	std::cout<<"min.x = "<<min.x<<"  max.x = "<<max.x<<std::endl;
	std::cout<<"min.y = "<<min.y<<"  max.y = "<<max.y<<std::endl;
	std::cout<<"min.z = "<<min.z<<"  max.z = "<<max.z<<std::endl;
	bool isUnitMillimetre = false;

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
		std::cout<<"min.x = "<<min.x<<"  max.x = "<<max.x<<std::endl;
		std::cout<<"min.y = "<<min.y<<"  max.y = "<<max.y<<std::endl;
		std::cout<<"min.z = "<<min.z<<"  max.z = "<<max.z<<std::endl;
	}
	//可视化
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud Original"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem(0.3*(max.y-min.y));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (cloud, 0, 255, 0);
	viewer->addPointCloud(cloud, single_color, "Original");
	while(!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100));
	}




	//对点云进行平面拟合（可选先进行提速滤波和带通滤波以加快处理速度），并且将点云旋转到传感器轴与测量平面垂直
	if (Config::get<bool>("VoxelGrid_filter_before_fitting_plane"))
	{
		std::cout<<"VoxelGrid_filter_before_fitting_plane: "<<Config::get<bool>("VoxelGrid_filter_before_fitting_plane")<<std::endl;
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
		// Eigen::Matrix4f tf;
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
	viewer2->setBackgroundColor (0, 0, 0);
	viewer2->addCoordinateSystem(0.3*(max.y-min.y));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2 (cloud, 0, 255, 0);
	viewer2->addPointCloud(cloud, single_color, "Transformed");
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
		viewer3->addCoordinateSystem(0.3*(max.y-min.y));
		// Coloring and visualizing target cloud (red).
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  target_color (cloud_filtered, 255, 0, 0);
		viewer3->addPointCloud<pcl::PointXYZ> (cloud_filtered, target_color, "cloud_filtered");
		viewer3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_filtered");
		viewer3->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_filtered, normals, 1, (0.1*(max.x-min.x)), "normals");
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
		pass.filter (*cloud_filtered);
		pass.setInputCloud (cloud_filtered);
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




		/* Detect defect using normal_z*/
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




		/* Detect defect using curvature*/
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




		/* Detect defect using pointcloud_z*/
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



		/*Cluster using DBSCN*/
		std::vector<dbscn::Point> points;
		dbscn::Point tmp_pt;
		for (size_t i = 0; i < cloudCP->points.size(); ++i)
		{
			tmp_pt.x=cloudCP->points[i].x;
			tmp_pt.y=cloudCP->points[i].y;
			points.push_back(tmp_pt);
		}
		vector<int> labels;
		int num_cluster = dbscn::dbscan(points, labels, Config::get<float>("exp"), Config::get<int>("MinPt"));
		cout<<"cluster size is "<<num_cluster<<endl;
		for(int i = 0; i < (int)points.size(); i++)
		{
			std::cout<<"Point("<<points[i].x<<", "<<points[i].y<<"): "<<labels[i]<<std::endl;
		}




		//可视化
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3(new pcl::visualization::PCLVisualizer("Defect Dtect Results"));
		viewer3->setBackgroundColor (0, 0, 0);
		viewer3->addCoordinateSystem(0.3*(max.y-min.y));
		if (Config::get<bool>("show_normal"))
		{
			viewer3->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_filtered, normals, 1, (0.1*(max.x-min.x)), "normals");
		}
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  cloud_color (cloud_filtered, 125, 125, 125);
		// viewer3->addPointCloud<pcl::PointXYZ> (cloud, cloud_color, "cloud");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  target_color (cloud_filtered, 0, 255, 0);
		viewer3->addPointCloud<pcl::PointXYZ> (cloud_filtered, target_color, "cloud_filtered");
		viewer3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_filtered");

		for (int i = 1; i <= num_cluster; i++)
		{
			stringstream ss;
			ss<<i;
			index.clear();
			for (int j = 0; j < (int)points.size(); j++)
			{
				if (labels[j] == i)
				{
					index.push_back(j);
				}
			}
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCPn(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud(*cloudCP, index, *cloudCPn);
			std::string cloud_id="cloudCP_"+ss.str();
			std::cout<<"\ncloud_id: "<<cloud_id <<", size: "<< cloudCPn->points.size() <<", MinPtDefect: "<<Config::get<int>("MinPtDefect")<<std::endl;
			float xc,yc,zc;
			getCenter(cloudCPn, xc,yc,zc);
			if (cloudCPn->points.size()< Config::get<int>("MinPtDefect") )
			{
				std::cout<<"cluster has less points than the threshold, drop this cludter!"<<std::endl;
				continue;
			}
			else if (xc < Config::get<float>("PASS_filter_x_min_offset") || xc >Config::get<float>("PASS_filter_x_max_offset") ||
				yc < Config::get<float>("PASS_filter_y_min_offset") || yc > Config::get<float>("PASS_filter_y_max_offset") )
			{
				std::cout<<"xc,yc,zc: "<<xc<<" "<<yc<<" "<<zc<<std::endl;
				std::cout<<"cluster centroid is too close to the edge, drop this cludter!"<<std::endl;
				continue;
			}
			else
			{
				std::cout<<"xc,yc,zc: "<<xc<<" "<<yc<<" "<<zc<<std::endl;
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  target_color_cloudCPn (cloudCPn, 255, 0, 0);
				viewer3->addPointCloud<pcl::PointXYZ> (cloudCPn, target_color_cloudCPn, cloud_id);
				viewer3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, cloud_id);
				viewer3->spinOnce(100);
				getchar();

				pcl::PointXYZ min_cloudCPn;
				pcl::PointXYZ max_cloudCPn;
				pcl::getMinMax3D(*cloudCPn,min_cloudCPn,max_cloudCPn);

				std::cout
				<<"min_x: "<<min_cloudCPn.x<<"max_x: "<<max_cloudCPn.x<<"\n"
				<<"min_y: "<<min_cloudCPn.y<<"max_y: "<<max_cloudCPn.y<<"\n"
				<<"min_z: "<<min_cloudCPn.z<<"max_z: "<<max_cloudCPn.z<<"\n"
				<<std::endl;

				pcl::transformPointCloud(*cloud, *cloud, tf.inverse());
				getchar();
				pcl::PointXYZ outer_max;
				pcl::PointXYZ outer_min;
				pcl::PointXYZ iner_max;
				pcl::PointXYZ iner_min;
				outer_min.x=min_cloudCPn.x-Config::get<float>("normal_estimate_area_ratio")*std::abs(max_cloudCPn.x-min_cloudCPn.x)/2;
				outer_max.x=max_cloudCPn.x+Config::get<float>("normal_estimate_area_ratio")*std::abs(max_cloudCPn.x-min_cloudCPn.x)/2;
				iner_min.x=min_cloudCPn.x;
				iner_max.x=max_cloudCPn.x;
				outer_min.y=min_cloudCPn.y-Config::get<float>("normal_estimate_area_ratio")*std::abs(max_cloudCPn.y-min_cloudCPn.y)/2;
				outer_max.y=max_cloudCPn.y+Config::get<float>("normal_estimate_area_ratio")*std::abs(max_cloudCPn.y-min_cloudCPn.y)/2;
				iner_min.y=min_cloudCPn.y;
				iner_max.y=max_cloudCPn.y;
				outer_min.z=min_cloudCPn.z-Config::get<float>("normal_estimate_area_ratio")*std::abs(max_cloudCPn.z-min_cloudCPn.z)/2;
				outer_max.z=max_cloudCPn.z+Config::get<float>("normal_estimate_area_ratio")*std::abs(max_cloudCPn.z-min_cloudCPn.z)/2;
				iner_min.z=min_cloudCPn.z;
				iner_max.z=max_cloudCPn.z;
				pcl::Normal avg_normal_estimation_area = getLoopNormal(cloud_filtered, normals,outer_max,outer_min, iner_max,iner_min);
				std::cout<<"avg_normal_estimation_area: "
				<<avg_normal_estimation_area.normal_x<<" "
				<<avg_normal_estimation_area.normal_y<<" "
				<<avg_normal_estimation_area.normal_z<<std::endl;



				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_CPn(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::PassThrough<pcl::PointXYZ> pass;
				pass.setInputCloud (cloud);
				pass.setFilterFieldName ("x");
				pass.setFilterLimits (outer_min.x,outer_max.x);
				std::cout<<"PASS_filter_x_min: "<<outer_min.x<<std::endl;
				std::cout<<"PASS_filter_x_max: "<<outer_max.x<<std::endl;
				pass.filter (*cloud_filtered_CPn);
				pass.setFilterFieldName ("y");
				pass.setFilterLimits (outer_min.y,outer_max.y);
				std::cout<<"PASS_filter_y_min: "<<outer_min.y<<std::endl;
				std::cout<<"PASS_filter_y_max: "<<outer_max.y<<std::endl;
				pass.filter (*cloud_filtered_CPn);

				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  normal_estimate_area_cloudCPn (cloud_filtered_CPn, 255, 255, 255);
				viewer3->addPointCloud<pcl::PointXYZ> (cloud_filtered_CPn, normal_estimate_area_cloudCPn, cloud_id+"_normal_estimate_area");
				viewer3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, cloud_id+"_normal_estimate_area");
				viewer3->spinOnce(100);
				getchar();

				pass.setInputCloud (cloud_filtered_CPn);
				pass.setFilterFieldName ("x");
				pass.setFilterLimits (min_cloudCPn.x, max_cloudCPn.x);
				std::cout<<"PASS_filter_x_min: "<<min_cloudCPn.x<<std::endl;
				std::cout<<"PASS_filter_x_max: "<<max_cloudCPn.x<<std::endl;
				pass.setFilterLimitsNegative (true);
				pass.filter (*cloud_filtered_CPn);
				pass.setFilterFieldName ("y");
				pass.setFilterLimits (min_cloudCPn.y,max_cloudCPn.y);
				std::cout<<"PASS_filter_y_min: "<<min_cloudCPn.y<<std::endl;
				std::cout<<"PASS_filter_y_max: "<<max_cloudCPn.y<<std::endl;
				pass.setFilterLimitsNegative (true);
				pass.filter (*cloud_filtered_CPn);

				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  normal_estimate_area_cloudCPn2 (cloud_filtered_CPn, 0, 255, 255);
				viewer3->addPointCloud<pcl::PointXYZ> (cloud_filtered_CPn, normal_estimate_area_cloudCPn2, cloud_id+"_normal_estimate_area2");
				viewer3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, cloud_id+"_normal_estimate_area2");
				viewer3->spinOnce(100);
				getchar();

			}
		}



		//可视化，法向量检测到的缺陷
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  target_color_cloudCP (cloudCP, 255, 0, 0);
		// viewer3->addPointCloud<pcl::PointXYZ> (cloudCP, target_color_cloudCP, "cloudCP");
		// viewer3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloudCP");

		//可视化，曲率检测到的缺陷
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  target_color_cloudCP2 (cloudCP2, 0, 0, 255);
		// viewer3->addPointCloud<pcl::PointXYZ> (cloudCP2, target_color_cloudCP2, "cloudCP2");
		// viewer3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloudCP2");

		//可视化，Z阈值检测到的缺陷
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  target_color_cloudCP3 (cloudCP3, 0, 255, 255);
		// viewer3->addPointCloud<pcl::PointXYZ> (cloudCP3, target_color_cloudCP3, "cloudCP3");
		// viewer3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloudCP3");

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
		viewer3->addCoordinateSystem(0.3*(max.y-min.y));
		// Coloring and visualizing target cloud (red).
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  target_color (cloud, 255, 0, 0);
		viewer3->addPointCloud<pcl::PointXYZ> (cloud, target_color, "cloud");
		viewer3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::Normal>  normal_color (normals, 255, 255, 255);
		viewer3->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 1, (0.1*(max.x-min.x)), "normals");
		while(!viewer3->wasStopped())
		{
			viewer3->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100));
		}
	}
	return 0;
}
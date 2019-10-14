#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

using namespace std;
using namespace cv;
using namespace pcl;

cv::Point project(const pcl::PointXYZ &pt, const cv::Mat &projection_matrix)
{
	cv::Mat pt_3D(4, 1, CV_32FC1);

	pt_3D.at<float>(0) = pt.x;
	pt_3D.at<float>(1) = pt.y;
	pt_3D.at<float>(2) = pt.z;
	pt_3D.at<float>(3) = 1.0f;

	cv::Mat pt_2D = projection_matrix * pt_3D;

	float w = pt_2D.at<float>(2);
	float x = pt_2D.at<float>(0) / w;
	float y = pt_2D.at<float>(1) / w;
	return cv::Point(x, y);
}

void callback(const sensor_msgs::PointCloud2ConstPtr& msg_pc)
{

	// // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr (new pcl::PointCloud<pcl::PointXYZ>);
	// // if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloudptr) == -1)
	// // {
	// // 	PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
	// // 	return (-1);
	// // }
	// // std::cout	<< "Loaded " << cloudptr->width * cloudptr->height << " data points from pcd fields: "<< std::endl;

	// pcl::PointCloud<pcl::PointXYZ> cloud;
	// pcl::fromROSMsg(*msg_pc, cloud);

	// // cloud=*cloudptr;
	// cv::Mat projection_matrix;

	// float p[12]={671.860209, 0.000000, 643.055016, 0, 0.000000, 671.864080, 374.010332, 0, 0.00, 0.00, 1.0};
	// cv::Mat(3, 4, CV_32FC1, &p).copyTo(projection_matrix);
	// 	cv::Rect frame(0, 0, 640, 480);
	// cv::Mat  image=Mat::zeros(480,640, CV_8UC3);

	// for(pcl::PointCloud<pcl::PointXYZ>::iterator pt = cloud.points.begin(); pt < cloud.points.end(); pt++)
	// {
	// 	if (pt->z < 0)
	// 	{
	// 		continue;
	// 	}
	// 	cv::Point point2d = project(*pt, projection_matrix);
	// 	if (point2d.inside(frame))
	// 	{
	// 		std::cout<<point2d.x<<" "<<point2d.y<<std::endl;
	// 		std::cout<<pt->x <<" "<<pt->y <<" "<<pt->z <<std::endl;
	// 		// image.at<cv::Vec3b>(point2d.y,point2d.x)[0]=1000*pt->x;
	// 		// image.at<cv::Vec3b>(point2d.y,point2d.x)[1]=1000*pt->y;
	// 		image.at<cv::Vec3b>(point2d.y,point2d.x)[2]=1000*pt->z;
	// 	}

	// }
	// imshow("image",image);
	// waitKey();
}

int main (int argc, char** argv)
{

	ros::init(argc, argv, "spacialnet");
	// ros::NodeHandle n;
	// ROS_INFO_STREAM("haha ^_^ haha ^_^ haha ^_^ haha ^_^ haha ^_^ haha ^_^ haha ^_^ haha ^_^ haha ^_^ haha");

	// ros::Subscriber sub = n.subscribe("/camera/depth_registered/points", 10, callback);

	// // message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, "/camera/depth_registered/points", 1);
	// // message_filters::Subscriber<sensor_msgs::Image> image_sub(n, "/camera/rgb/image_rect_color", 1);

	// // typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
	// // Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, image_sub);
	// // sync.registerCallback(boost::bind(&callback, _1, _2));
	// ros::spin();

	return 0;
}
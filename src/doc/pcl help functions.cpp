pcl::PassThrough<pcl::PointXYZ> pass;
pass.setInputCloud (cloud);            //设置输入点云
pass.setFilterFieldName ("z");         //设置过滤时所需要点云类型的Z字段
pass.setFilterLimits (0.0, 1.0);        //设置在过滤字段的范围
//pass.setFilterLimitsNegative (true);   //设置保留范围内还是过滤掉范围内
pass.filter (*cloud_filtered);


//the mapping tells you to that points of the oldcloud the new ones correspond
//but we  will not use it
std::vector<int> mapping;
pcl::removeNaNFromPointCloud(_p_cloud, _p_cloud, mapping);
std::cout<<"point  size: "<<_p_cloud.points.size()<<std::endl;





pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PLYReader::read("duckscenario.ply", *input_cloud, 0);
std::cout << "Loaded " << input_cloud->size () << " data points from duckscenario.ply" << std::endl;



//可视化
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud Original"));
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (cloud, 0, 255, 0);
viewer->addPointCloud(cloud, single_color, "Original");
viewer->addCoordinateSystem(0.1*(max.y-min.y));
while(!viewer->wasStopped())
{
	viewer->spinOnce(100);
	boost::this_thread::sleep(boost::posix_time::microseconds(100));
}
viewer->removePointCloud("cloud");
viewer->removeAllPointClouds();
viewer->close();


cv::Mat  image=cv::Mat::zeros(1080, 1920, CV_8UC3);
for(pcl::PointCloud<pcl::PointXYZ>::iterator pt = cloud_filtered->points.begin(); pt < cloud_filtered->points.end(); pt++)
{
	std::cout<<pt->x <<" "<<pt->y <<" "<<pt->z <<std::endl;
	std::cout<<int(5000*pt->y)<<" "<<int(5000*pt->x)<<" "<< int(5000*pt->y)+500 <<" "<< int(5000*pt->x)+500 <<std::endl;
	image.at<cv::Vec3b>(int(5000*pt->y)+500,int(5000*pt->x)+500)[0]=int(5000*pt->x);
	image.at<cv::Vec3b>(int(5000*pt->y)+500,int(5000*pt->x)+500)[1]=int(5000*pt->y);
	image.at<cv::Vec3b>(int(5000*pt->y)+500,int(5000*pt->x)+500)[2]=int(5000*pt->z);

	std::cout<<pt->x <<" "<<pt->y <<" "<<pt->z <<std::endl;
	std::cout<<int(1000*pt->y)<<" "<<int(1000*pt->x)<<" "<< int(1000*pt->y)+100 <<" "<< int(1000*pt->x)+100 <<std::endl;
	image.at<cv::Vec3b>(int(1000*pt->y)+100,int(1000*pt->x)+100)[0]=int(1000*pt->x);
	image.at<cv::Vec3b>(int(1000*pt->y)+100,int(1000*pt->x)+100)[1]=int(1000*pt->y);
	image.at<cv::Vec3b>(int(1000*pt->y)+100,int(1000*pt->x)+100)[2]=int(1000*pt->z);
}
cv::imshow("image",image);
cv::waitKey();
cv::imwrite("image.png",image);
pcl::PassThrough<pcl::PointXYZ> pass;
pass.setInputCloud (cloud);            //设置输入点云
pass.setFilterFieldName ("z");         //设置过滤时所需要点云类型的Z字段
pass.setFilterLimits (0.0, 1.0);        //设置在过滤字段的范围
//pass.setFilterLimitsNegative (true);   //设置保留范围内还是过滤掉范围内
pass.filter (*cloud_filtered);








pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PLYReader::read("duckscenario.ply", *input_cloud, 0);
std::cout << "Loaded " << input_cloud->size () << " data points from duckscenario.ply" << std::endl;
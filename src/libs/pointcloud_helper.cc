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

cv::Mat getR2registeZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	double start = (double)(cv::getTickCount());

	//事实说明：在for循环内重新定义平面拟合实例和在for循环外重新定义其他名称的平面拟合实例都说明了：点云不变的情况下，平面方程系数是不会变化的。
	std::ofstream outfile_a ("./plane equation A.txt",std::ios_base::trunc);
	std::ofstream outfile_b ("./plane equation B.txt",std::ios_base::trunc);
	std::ofstream outfile_c ("./plane equation C.txt",std::ios_base::trunc);
	std::ofstream outfile_d ("./plane equation D.txt",std::ios_base::trunc);

	//fit plane equation
	// 保存局内点索引
	std::vector<int> inliers;
	//平面方程系数
	Eigen::VectorXf coef = Eigen::VectorXf::Zero(4 , 1);
	Eigen::VectorXf coef_opt = Eigen::VectorXf::Zero(4 , 1);
	Eigen::VectorXf coef_opt2 = Eigen::VectorXf::Zero(4 , 1);
	// 采样一致性模型对象
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
	ransac.setDistanceThreshold(0.01);
	ransac.computeModel();
	ransac.getInliers(inliers);

	ransac.getModelCoefficients(coef);
	outfile_a<<acosf(coef[0])/PI*180<<"\n";
	outfile_b<<acosf(coef[1])/PI*180<<"\n";
	outfile_c<<acosf(coef[2])/PI*180<<"\n";
	outfile_d<<coef[3]<<"\n";

	for (int i = 0; i < 10; ++i)
	{
		ransac.refineModel();
		ransac.getModelCoefficients(coef);
		outfile_a<<i<<" "<<acosf(coef[0])/PI*180<<"\n";
		outfile_b<<i<<" "<<acosf(coef[1])/PI*180<<"\n";
		outfile_c<<i<<" "<<acosf(coef[2])/PI*180<<"\n";
		outfile_d<<i<<" "<<coef[3]<<"\n";
	}

	model_p->optimizeModelCoefficients(inliers,coef,coef_opt);
	outfile_a<<acosf(coef_opt[0])/PI*180<<"\n";
	outfile_b<<acosf(coef_opt[1])/PI*180<<"\n";
	outfile_c<<acosf(coef_opt[2])/PI*180<<"\n";
	outfile_d<<coef_opt[3]<<"\n";

	model_p->optimizeModelCoefficients(inliers,coef_opt,coef_opt2);
	outfile_a<<acosf(coef_opt2[0])/PI*180<<"\n";
	outfile_b<<acosf(coef_opt2[1])/PI*180<<"\n";
	outfile_c<<acosf(coef_opt2[2])/PI*180<<"\n";
	outfile_d<<coef_opt2[3]<<"\n";

	outfile_a.close();
	outfile_b.close();
	outfile_c.close();
	outfile_d.close();

	std::cout<<"plane's normal vector: "<<coef[0]<<" "<<coef[1]<<" "<<coef[2]<<std::endl;


	std::cout<<"\n******************************************************************************************************"<<std::endl;
	Eigen::Vector3d NZ(0.0, 0.0, 1.0);
	Eigen::Vector3d tmp_NZ;
	tmp_NZ(0) = coef[0];
	tmp_NZ(1) = coef[1];
	tmp_NZ(2) = coef[2];
	std::cout<<"tmp_NZ:\n"<<tmp_NZ<<std::endl;
	tmp_NZ.normalize();
	std::cout<<"\ntmp_NZ (normalized):\n"<<tmp_NZ<<std::endl;
	double angle = acos(NZ.dot(tmp_NZ));
	Eigen::Vector3d axis = NZ.cross(tmp_NZ);
	axis.normalize();
	Eigen::Vector3d axisangle = axis*angle;

	std::cout
	<< "\nNZ:\n"<< NZ <<"\n"
	<< "\ntmp_NZ:\n"	<< tmp_NZ <<"\n"
	<< "\ndot:\n"<< NZ.dot(tmp_NZ) <<"\n"
	<< "\nangle[rad]: " << angle
	<< "\nangle[deg]: " << angle*180/pi <<"\n"
	<< "\naxis (normalized):\n" << axis<<"\n"
	<< "\naxisangle[rad]:\n"<<axisangle<<"\n"
	<<std::endl;

	cv::Vec3d cv_axisangle;
	for (int i = 0; i < 3; ++i)
	{
		cv_axisangle[i]=axisangle[i];
		std::cout<<"axisangle["<<"i"<<"] :"<<axisangle[i]<<"  cv_axisangle["<<"i"<<"] :"<<cv_axisangle[i]<<std::endl;
	}
	cv::Mat cv_rotation_matrix(cv::Mat::zeros(cv::Size(3,3),CV_64F));
	cv::Rodrigues(cv_axisangle, cv_rotation_matrix);
	std::cout << "\ncv_rotation_matrix:" << "\n" << cv_rotation_matrix << std::endl;
	std::cout<<"\n******************************************************************************************************"<<std::endl;

	double end = ((double)cv::getTickCount() - start) / cv::getTickFrequency();
	std::cout << "getR2registeZ() 所用时间为：" << end <<std::endl;
	return cv_rotation_matrix;
}

int loadPointCloudData(string inputFilename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	double start = (double)(cv::getTickCount());

	string pcd=".pcd";
	string ply=".ply";
	string PCD=".PCD";
	string PLY=".PLY";
	string::size_type id1;
	string::size_type id2;


	id1=inputFilename.find(pcd);//在inputFilename中查找pcd.
	if(id1 == string::npos )//不存在。
	{
		id1=inputFilename.find(PCD);
		if(id1 == string::npos )
		{
			std::cout<<"inputFile is not .pcd file."<<std::endl;
		}
		else//存在。
		{
		//读取PCD文件
			if(pcl::io::loadPCDFile<pcl::PointXYZ>(inputFilename,*cloud) !=0)
			{
				return -1;
			}
		}
	}
	else//存在。
	{
		//读取PCD文件
		if(pcl::io::loadPCDFile<pcl::PointXYZ>(inputFilename,*cloud) !=0)
		{
			return -1;
		}
	}

	id2=inputFilename.find(ply);//在inputFilename中查找pcd.
	if(id2 == string::npos )//不存在。
	{
		id2=inputFilename.find(PLY);
		if(id2 == string::npos )
		{
			std::cout<<"inputFile is not .ply file."<<std::endl;
		}
		else//存在。
		{
		//读取PCD文件
			if(pcl::io::loadPLYFile<pcl::PointXYZ>(inputFilename,*cloud) !=0)
			{
				return -1;
			}
		}
	}
	else//存在。
	{
		//读取PCD文件
		if(pcl::io::loadPLYFile<pcl::PointXYZ>(inputFilename,*cloud) !=0)
		{
			return -1;
		}
	}

	if( (id1 == string::npos) && (id2 == string::npos) )
	{
		double end = ((double)cv::getTickCount() - start) / cv::getTickFrequency();
		std::cout << "loadPointCloudData() 所用时间为：" << end <<std::endl;
		std::cout << "loadPointCloudData() 执行状态为：" << "FAILED!" <<std::endl;
		return -1;
	}
	else
	{
		double end = ((double)cv::getTickCount() - start) / cv::getTickFrequency();
		std::cout << "loadPointCloudData() 所用时间为：" << end <<std::endl;
		std::cout << "loadPointCloudData() 执行状态为：" << "SUCCESS!" <<std::endl;
		return 0;
	}
}



Eigen::Matrix4f E2R(const double alpha, const double beta, const double gamma)
{
	Eigen::Matrix4f m;
	m << cos(beta)*cos(gamma), cos(gamma)*sin(alpha)*sin(beta) - cos(alpha)*sin(gamma), sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta), 0,
	cos(beta)*sin(gamma)   , cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma), cos(alpha)*sin(beta)*sin(gamma) - cos(gamma)*sin(alpha), 0,
	-sin(beta)             , cos(beta)*sin(alpha)                                 , cos(alpha)*cos(beta)                                 , 0,
	0                      , 0                                                    , 0                                                    , 1;
	return m;
}

void getCenter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float &xc, float &yc, float &zc)
{
	float sum_x = 0;
	float sum_y = 0;
	float sum_z = 0;
	for(pcl::PointCloud<pcl::PointXYZ>::iterator pt = cloud->points.begin(); pt < cloud->points.end(); pt++)
	{
		sum_x = sum_x+ pt->x;sum_y = sum_y+ pt->y;sum_z = sum_z+ pt->z;
	}
	xc = sum_x/cloud->points.size();
	yc = sum_y/cloud->points.size();
	zc = sum_z/cloud->points.size();
}


void getMinMax3D(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
	float &min_x, float &max_x, float &min_y,
	float &max_y, float &min_z, float &max_z)
{
	for(pcl::PointCloud<pcl::PointXYZ>::iterator pt = cloud->points.begin(); pt < cloud->points.end(); pt++)
	{
		if (min_x > pt->x)
			min_x = pt->x;
		if (max_x < pt->x)
			max_x = pt->x;
		if (min_y > pt->y)
			min_y = pt->y;
		if (max_y < pt->y)
			max_y = pt->y;
		if (min_z > pt->z)
			min_z = pt->z;
		if (max_z < pt->z)
			max_z = pt->z;
	}
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
	double dis = 0;
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

pcl::Normal getLoopNormal(
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
	const pcl::PointCloud<pcl::Normal>::Ptr& normal,
	pcl::PointXYZ& outer_max,
	pcl::PointXYZ& outer_min,
	pcl::PointXYZ& iner_max,
	pcl::PointXYZ& iner_min)
{
	pcl::Normal avg_normal;
	float nx=0;
	float ny=0;
	float nz=0;
	int count=0;

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		if ((cloud->points[i].x < outer_max.x) && (cloud->points[i].x > iner_max.x))
		{
			if ((cloud->points[i].y < outer_max.y) && (cloud->points[i].y > outer_min.y))
			{
				nx=nx+normal->points[i].normal_x;
				ny=ny+normal->points[i].normal_y;
				nz=nz+normal->points[i].normal_z;
				std::cout<<" x, y, z: "<<cloud->points[i].x<<" "<<cloud->points[i].y<<" "<<cloud->points[i].z<<std::endl;
				count++;
			}
		}
		if( (cloud->points[i].x < iner_min.x) && (cloud->points[i].x > outer_min.x))
		{
			if ((cloud->points[i].y < outer_max.y) && (cloud->points[i].y > outer_min.y))
			{
				nx=nx+normal->points[i].normal_x;
				ny=ny+normal->points[i].normal_y;
				nz=nz+normal->points[i].normal_z;
				std::cout<<" x, y, z: "<<cloud->points[i].x<<" "<<cloud->points[i].y<<" "<<cloud->points[i].z<<std::endl;
				count++;
			}
		}
		if ((cloud->points[i].x < iner_max.x) && (cloud->points[i].x > iner_min.x))
		{
			if ((cloud->points[i].y < outer_max.y) && (cloud->points[i].y > iner_max.y))
			{
				nx=nx+normal->points[i].normal_x;
				ny=ny+normal->points[i].normal_y;
				nz=nz+normal->points[i].normal_z;
				std::cout<<" x, y, z: "<<cloud->points[i].x<<" "<<cloud->points[i].y<<" "<<cloud->points[i].z<<std::endl;
				count++;
			}
			if ((cloud->points[i].y < iner_min.y) && (cloud->points[i].y > outer_min.y))
			{
				nx=nx+normal->points[i].normal_x;
				ny=ny+normal->points[i].normal_y;
				nz=nz+normal->points[i].normal_z;
				std::cout<<" x, y, z: "<<cloud->points[i].x<<" "<<cloud->points[i].y<<" "<<cloud->points[i].z<<std::endl;
				count++;
			}
		}
	}
	avg_normal.normal_x=nx/count;
	avg_normal.normal_y=ny/count;
	avg_normal.normal_z=nz/count;
	float d=sqrt(pow( avg_normal.normal_x, 2) + pow( avg_normal.normal_y, 2) + pow( avg_normal.normal_z, 2));
	avg_normal.normal_x=avg_normal.normal_x/d;
	avg_normal.normal_y=avg_normal.normal_y/d;
	avg_normal.normal_z=avg_normal.normal_z/d;
	return avg_normal;
}
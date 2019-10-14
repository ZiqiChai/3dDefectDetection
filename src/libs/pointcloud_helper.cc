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
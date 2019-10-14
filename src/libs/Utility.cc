#include "../../include/Utility.h"



/*----------------------------
 * 功能 : 判断生成的随机数点，是否重复
 *----------------------------
 * 函数 : check
 * 返回 : bool，true表示重复，false表示不重复
 *
 * 参数 : index     	   [in]	已生成的点下标，vector
 * 参数 : k         	   [in]	需要比较的点
  */
bool check(vector<pair<int,int>> points, pair<int,int> new_points)
{
	for(int i = 0; i < points.size(); i++)
	{
		if( new_points.first == points[i].first &&  new_points.second == points[i].second)
			return true;
		else if(new_points.second == points[i].first &&  new_points.first == points[i].second)
			return true;
	}
	return false;
}
/*----------------------------
 * 功能 : 输入样本点，返回拟合直线参数
 *----------------------------
 * 函数 : getLine
 * 返回 : 拟合直线参数：a0 a1
 *
 * 参数 : samplePoints		[in]	样本，vector
 * 参数 : sampleWeights		[in]	样本权重，vector
 * 参数 : Mat(2*1)	        [out]	a0 a1,存储在Mat中
 */
template<class T>
Mat getLine(vector<T> samplePoints,vector<T> sampleWeights = vector<T>(1) )
{
	Mat left = (Mat_<T>(2,2) << 0, 0, 0, 0);
	Mat right = (Mat_<T>(2,1) << 0, 0);

    if(sampleWeights.size() == 1)
		sampleWeights = vector<T>(samplePoints.size(),1);

	for(int i = 0; i < samplePoints.size();i ++)
	{
		left.at<T>(0,0) += sampleWeights[i];
		left.at<T>(0,1) += (sampleWeights[i]) * i;
		left.at<T>(1,0) += (sampleWeights[i]) * i;
		left.at<T>(1,1) += (sampleWeights[i]) * i * i;
		right.at<T>(0,0) += (sampleWeights[i]) * (samplePoints[i]);
		right.at<T>(1,0) += (sampleWeights[i]) * (samplePoints[i]) * i;

	}
	return left.inv() * right;
}

/*----------------------------
 * 功能 : 计算点到直线距离
 *----------------------------
 * 函数 : getDistance
 * 返回 : 点到直线距离
 *
 * 参数 : y		           [in]	点y坐标
 * 参数 : x		           [in]	点x坐标（vector下标）
 * 参数 : line	           [in]	直线参数（Mat）
 * 参数 : distance	       [out]点到直线距离
 */
double getDistance(double x,double y,Mat line)
{
	return abs(line.at<double>(1) * x - y + line.at<double>(0)) / sqrt(line.at<double>(1) * line.at<double>(1) +1);
}
/*----------------------------
 * 功能 : Ransac
 *----------------------------
 * 函数 : ransac
 * 返回 : 拟合直线参数
 *
 * 参数 : numOfp		   [in]	观测样本点数
 * 参数 : samples		   [in]	样本vector
 * 参数 : k  	           [in]	迭代次数
 * 参数 : threshold_In	   [in] 判断为内点的阈值
 * 参数 : average	       [in] 内点取值均值，引用
 * 参数 : Result	       [in] 拟合直线参数（Mat）
 */

/*----------------------------
 * 功能 : Ransac
 *----------------------------
 * 函数 : ransac
 * 返回 : 拟合直线参数
 *
 * 参数 : num		       	[in]	观测样本点数
 * 参数 : samples		   	[in]	样本vector
 * 参数 : max_iteration     	[in]	迭代次数
 * 参数 : threshold_In	   	[in] 	判断为内点的阈值
 * 参数 : average	       	[in] 	内点取值均值，引用
 * 参数 : Result	           	[in] 	拟合直线参数（Mat）
 */

Mat ransac(int num, std::vector<double> samples, int max_iteration,double threshold_In, double& average)
{
	RNG rng;						                     //随机数生成器
	vector<double> weights(num,1);
//	double threshold_In = 1;							//判断为内点的距离阈值
//	int max_iteration = 200;										//迭代次数
	int num_ofInliners = 0;   						    //内点数目
	Mat Result = (Mat_<double>(2,1) << 0, 0);			//迭代结果直线

	if (samples.size() < 2)
	{
		cout<<"Too little samples!"<<endl;
		return Result;
	}

	vector<pair<int,int>> randomP_array;         //随机点对容器
	vector<int> p1_array;                       //随机选取点容器 1
	vector<int> p2_array;                       //随机选取点容器 2

	for(int i = 0; i < max_iteration; i++)
	{
		// 随机选取两点，计算方程
		int p1 = 0;
		int p2 = 0;
		pair<int,int> p(0,0);
 		while(p1 == p2 || check(randomP_array,p))
 		{
 			p1 = rng.uniform(0,num);  //随机选取两个点（下标）
 			p2 = rng.uniform(0,num);
 		}
		p.first = p1;
		p.second = p2;
	    randomP_array.push_back(p);
		vector<double> linePara;
		vector<double> lineWeight;
		linePara.push_back(samples[p1]);
		linePara.push_back(samples[p2]);
		lineWeight.push_back(1);
		lineWeight.push_back(1);
		Mat x = (Mat_<double>(2,1) << 0, 0);
		x = getLine(linePara,lineWeight);

		// 计算点到直线距离，判断是否为内点
		vector<double> inliners;          //内点容器
		for(int j = 0; j < num; j++)
		{
			if(getDistance(j,samples[j],x) < threshold_In)
				inliners.push_back(samples[j]);
		}
		// 根据内点重新估计模型
		Mat lineTemp = getLine(inliners);
		if(inliners.size() > num_ofInliners)
		{
			Result = lineTemp;
			num_ofInliners = inliners.size();
			//计算内点取值均值
			double sum = 0;
			for (vector<double>::iterator iter = inliners.begin();iter != inliners.end();iter++)
			{
				sum += *iter;
			}
			average = sum / inliners.size();;
		}
	}
   return Result;
}
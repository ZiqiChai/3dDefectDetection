//C++
#include <stdlib.h>
#include <iostream>

//OpenCV
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

//functions
Mat ransac(int num, std::vector<double> samples, int max_iteration, double threshold_In, double& average);

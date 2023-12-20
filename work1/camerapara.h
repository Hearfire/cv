#pragma once
#include<opencv2/opencv.hpp>


class camerapara {
public:
	double fx, fy, cx, cy;
	cv::Mat leftRTMat, rightRTMat;
};

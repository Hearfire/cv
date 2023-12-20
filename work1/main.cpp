#include<opencv2/opencv.hpp>
#include<iostream>
#include<fstream>
#include<sstream>
#include <string>
#include<chrono>
#include"stereomatch.h"
#include"camerapara.h"



//The folder musht contain two images and camera parameter
//input folder_path,output fx, fy, cx, cy;  leftRTMat,rightRTMat
bool readin(const std::string folder_path, camerapara&para, cv::Mat& left_image, cv::Mat& right_image) {
	std::string camparapath = folder_path+"\\para.txt";
	left_image = cv::imread(folder_path+"\\left.jpg");
	right_image = cv::imread(folder_path+"\\right.jpg");

	////test
	//std::string camparapath = "D:\\cvwork\\work1\\work1\\stereo-rectify\\2\\para.txt";
	//left_image = cv::imread("D:\\cvwork\\work1\\work1\\stereo-rectify\\2\\left.jpg");
	//right_image = cv::imread("D:\\cvwork\\work1\\work1\\stereo-rectify\\2\\right.jpg");

	//check image
	if (left_image.empty() || right_image.empty()) {
		std::cout << "images error" << std::endl;
		return 0;
	}
	int width = left_image.cols;
	int height = left_image.rows;
	if (right_image.cols != left_image.cols || right_image.rows != left_image.rows) {
		std::cout << "images size error" << std::endl;
	}

	//read in para
	std::ifstream file(camparapath);

	if (!file.is_open()) {
		std::cout << "file open error" << std::endl;
		file.close();
		return false;
	}

	file.ignore(256, ':') >>para.fx;
	file.ignore(256, ':') >> para.fy;
	file.ignore(256, ':') >> para.cx;
	file.ignore(256, ':') >> para.cy;

	//std::cout << fx <<" " << fy << " " << cx << " " << cy;
	double leftRT[3][4];
	double rightRT[3][4];

	std::string line;
	while (std::getline(file, line)) {
		if (!line.empty())break;
	}

	for (int i = 0; i < 3; ++i) {
		std::getline(file, line);
		std::istringstream iss(line);
		for (int j = 0; j < 4; ++j) {
			iss >> leftRT[i][j];
		}
	}

	while (std::getline(file, line)) {
		if (!line.empty())break;
	}
	//double rfocus;//still unkonw
	//file >> rfocus;
	//std::getline(file, line);
	for (int i = 0; i < 3; ++i) {
		std::getline(file, line);
		std::istringstream iss(line);
		for (int j = 0; j < 4; ++j) {
			iss >> rightRT[i][j];
		}
	}
	file.close();

	para.leftRTMat = cv::Mat(3, 4, CV_64F, leftRT);
	para.rightRTMat = cv::Mat(3, 4, CV_64F, rightRT);
	return true;
}


//input camera parameter,two image; output rectified_left,right_image
bool stereorectify(const camerapara& para, const cv::Mat& left_image, const cv::Mat& right_image, cv::Mat& rectified_left, cv::Mat& rectified_right) {

	//build extrinsic matrix
	cv::Mat left_rotation_matrix = para.leftRTMat(cv::Rect(0, 0, 3, 3)).clone();
	cv::Mat left_translation_vector = para.leftRTMat(cv::Rect(3, 0, 1, 3)).clone();

	cv::Mat right_rotation_matrix = para.rightRTMat(cv::Rect(0, 0, 3, 3)).clone();
	cv::Mat right_translation_vector = para.rightRTMat(cv::Rect(3, 0, 1, 3)).clone();

	//build camera matrix
	cv::Mat K_left = (cv::Mat_<double>(3, 3) << para.fx, 0, para.cx,
		0, para.fy, para.cy,
		0, 0, 1);
	cv::Mat K_right = K_left;

	//Calculate the rotation matrix and displacement vector of the left eye relative to the right eye
	cv::Mat R = right_rotation_matrix *left_rotation_matrix .t();
	cv::Mat T = -R *left_translation_vector  + right_translation_vector;

	
	int width = left_image.cols;
	int height = right_image.rows;
	//stereoRectify
	cv::Mat R1, R2, P1, P2, Q;
	cv::Rect validROI[2];
	cv::stereoRectify(K_left, cv::Mat(), K_right, cv::Mat(),
		cv::Size(width, height), R, T,
		R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0, cv::Size(width, height), &validROI[0], &validROI[1]);

	cv::Mat map_left_x, map_left_y, map_right_x, map_right_y;
	cv::initUndistortRectifyMap(K_left, cv::Mat(), R1, P1, cv::Size(width, height), CV_32FC1, map_left_x, map_left_y);
	cv::initUndistortRectifyMap(K_right, cv::Mat(), R2, P2, cv::Size(width, height), CV_32FC1, map_right_x, map_right_y);

	cv::remap(left_image, rectified_left, map_left_x, map_left_y, cv::INTER_LINEAR);
	cv::remap(right_image, rectified_right, map_right_x, map_right_y, cv::INTER_LINEAR);

	return true;

}

bool ractifyshow(const cv::Mat& left_image, const cv::Mat& right_image, const cv::Mat& rectified_left, const cv::Mat& rectified_right) {
	cv::Mat img_src, img_rtf;
	cv::hconcat(left_image, right_image, img_src);
	cv::hconcat(rectified_left, rectified_right, img_rtf);

	//cv::Mat rectified_left_valid = rectified_left(validROI[1]).clone();
	//cv::Mat rectified_right_valid = rectified_right(validROI[1]).clone();
	//cv::hconcat(rectified_left_valid, rectified_right_valid, img_rtf);

	int width = left_image.cols;
	int height = right_image.rows;
	// draw
	for (int i = 1, iend = 8; i < iend; i++) {
		int h = height / iend * i;
		cv::line(img_src, cv::Point2i(0, h), cv::Point2i(width * 2, h), cv::Scalar(0, 0, 255));
		cv::line(img_rtf, cv::Point2i(0, h), cv::Point2i(width * 2, h), cv::Scalar(0, 0, 255));
	}
	imshow("image_src", img_src);
	imshow("image_rtf", img_rtf);
	cv::waitKey(0);

	return true;
}

int main() {
	std::cout << "please input your folder path,two images and camera parameters are required:" << endl;
	std::string folder_path;
	std::cin >> folder_path;	

	//test
	//std::string folder_path = "";

	camerapara para;
	cv::Mat left_image, right_image;
	if (!readin(folder_path, para,left_image,right_image)) {
		std::cout << "readin error" << std::endl;
		return 0;
	}

	cv::Mat rectified_left, rectified_right;
	if (!stereorectify(para, left_image, right_image, rectified_left, rectified_right)) {
		std::cout << "stereorectify error" << std::endl;
		return 0;
	}

	ractifyshow(left_image, right_image, rectified_left, rectified_right);


	int width = left_image.cols;
	int height = right_image.rows;



	while (true) {
		//stereo match
		int flag;
		std::cout << "please choose the algorithm:" << std::endl << "1.SAD" << std::endl << "2.NCC" << std::endl << "3.exit" << std::endl;
		std::cin >> flag;

		auto start_time = std::chrono::high_resolution_clock::now();

		if (flag == 1) {
			SAD mySAD;
			imshow("Disparity", mySAD.computerSAD(rectified_left, rectified_right));


			auto end_time = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
			std::cout << "Program execution time: " << duration.count() << " milliseconds" << std::endl;
			std::cout << "image width: " << width << "  " << "height: " << height << std::endl;
			waitKey(0);
		}
		else if (flag == 2) {
			NCC myncc;
			imshow("Disparity", myncc.computerNCC(rectified_left, rectified_right,  true));


			auto end_time = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
			std::cout << "Program execution time: " << duration.count() << " milliseconds" << std::endl;
			std::cout << "image width: " << width << "  " << "height: " << height << std::endl;
			waitKey(0);
		}
		else if (flag == 3) {
			return 0;
		}
		else {
			std::cout << "invalid algorithm!" << endl;
		}
	 }
	
	std::cout<< "unexpected exits!!!" << endl;
	return 0;
}
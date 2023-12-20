#pragma once
#include<opencv2/opencv.hpp>
#include<iostream>
#include<string>
using namespace cv;
using namespace std;
class SAD
{
public:
	SAD() :winSize(7), DSR(30) {}
	SAD(int _winSize, int _DSR) :winSize(_winSize), DSR(_DSR) {}
	Mat computerSAD(Mat& L, Mat& R); //¼ÆËãSAD
private:
	int winSize; //¾í»ýºËµÄ³ß´ç
	int DSR;     //ÊÓ²îËÑË÷·¶Î§

};

class NCC
{
public:
	NCC() :max_offset(79), kernel_size(3) {};
	NCC(int _max_offset,int _kernel_size):max_offset(_max_offset), kernel_size(_kernel_size) {};
	Mat computerNCC(Mat in1, Mat in2,  bool add_constant); //¼ÆËã
private:
	int max_offset;
	int kernel_size; // window size
};
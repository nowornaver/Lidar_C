#pragma once
#include <opencv2/highgui/highgui.hpp>
#include<iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include <cmath>
using namespace std;
using namespace cv;

class RoadLaneDetector
{
private:
	double img_size, img_center;
	//double left_m, right_m;
	Point left_b, right_b;

	//관심 영역 범위 계산시 사용 
	double poly_bottom_width = 0.85;  //사다리꼴 아래쪽 가장자리 너비 계산을 위한 백분율
	double poly_top_width = 0.07;     //사다리꼴 위쪽 가장자리 너비 계산을 위한 백분율
	double poly_height = 0.4;         //사다리꼴 높이 계산을 위한 백분율
	//steering angle


	

public:
	bool left_detect = false, right_detect = false;
	double right_angle , left_angle;
	double left_m , right_m;
	Mat filter_colors(Mat img_frame);
	Mat limit_region(Mat img_edges); 
	vector<Vec4i> houghLines(Mat img_mask);
	vector<vector<Vec4i> > separateLine(Mat img_edges, vector<Vec4i> lines); 
	vector<Point> regression(vector<vector<Vec4i> > separated_lines, Mat img_input);
	string predictDir();
	Mat drawLine(Mat img_input, vector<Point> lane, string dir);
	
	
	
	double calculateAngle(double slope1,double slope2); //// 기울기를 각도로 변환하는 함수
	double calculateSteeringAngle(double right_m,double left_m); //// 조향 각도를 계산하는 함수
	double controlSteering(double right_m, double left_m);

	    // 다항식 피팅 함수와 멤버 변수
    Mat polyfit(const vector<Point>& points, int degree);
    vector<Point> fitPolynomialLane(const vector<Point>& points, int degree);
    double polynomial(const Mat& coef, double x);





	

};
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "RoadLaneDetector.h"
#include <iostream>
#include <string>
#include <vector>
Mat RoadLaneDetector::filter_colors(Mat img_frame) {
	/*
		흰색/노란색 색상의 범위를 정해 해당되는 차선을 필터링한다.
	*/
	Mat output;
	UMat img_lab;
		UMat img_hls;
	UMat white_mask, white_image;
	UMat yellow_mask, yellow_image;
	img_frame.copyTo(output);

	//차선 색깔 범위 
	Scalar lower_white = Scalar(0, 200, 200); //흰색 차선 (RGB)
	Scalar upper_white = Scalar(255, 255, 255);
	Scalar lower_yellow = Scalar(0, 150, 150); //노란색 차선 (HSV)
	Scalar upper_yellow = Scalar(40, 255, 255);
    cvtColor(output, img_hls, COLOR_BGR2HLS);
	//흰색 필터링
	inRange(img_hls, lower_white, upper_white, white_mask);
	bitwise_and(output, output, white_image, white_mask);

	cvtColor(output, img_lab, COLOR_BGR2HSV);

	//노란색 필터링
	inRange(img_lab, lower_yellow, upper_yellow, yellow_mask);
	bitwise_and(output, output, yellow_image, yellow_mask);

	//두 영상을 합친다.
	addWeighted(white_image, 1.0, yellow_image, 1.0, 0.0, output);
	return output;
}


Mat RoadLaneDetector::limit_region(Mat img_edges) {
	/*
		관심 영역의 가장자리만 감지되도록 마스킹한다.
		관심 영역의 가장자리만 표시되는 이진 영상을 반환한다.
	*/
	int width = img_edges.cols;
	int height = img_edges.rows;

	Mat output;
	Mat mask = Mat::zeros(height, width, CV_8UC1);
	
	//관심 영역 정점 계산
	Point points[4]{
		Point((width * (1 - poly_bottom_width)) / 2, height),
		Point((width * (1 - poly_top_width)) / 2, height - height * poly_height),
		Point(width - (width * (1 - poly_top_width)) / 2, height - height * poly_height),
		Point(width - (width * (1 - poly_bottom_width)) / 2, height)
	};
	
	//정점으로 정의된 다각형 내부의 색상을 채워 그린다.
	fillConvexPoly(mask, points, 4, Scalar(255, 0, 0));

	//결과를 얻기 위해 edges 이미지와 mask를 곱한다.
	bitwise_and(img_edges, mask, output);
	return output;
}

vector<Vec4i> RoadLaneDetector::houghLines(Mat img_mask) {
	/*
		관심영역으로 마스킹 된 이미지에서 모든 선을 추출하여 반환
	*/
	vector<Vec4i> line;

	//확률적용 허프변환 직선 검출 함수 
	HoughLinesP(img_mask, line, 1,  CV_PI / 180, 20, 10, 20);
	return line;
}

vector<vector<Vec4i>> RoadLaneDetector::separateLine(Mat img_edges, vector<Vec4i> lines) {
	/*
		검출된 모든 허프변환 직선들을 기울기 별로 정렬한다.
		선을 기울기와 대략적인 위치에 따라 좌우로 분류한다.
	*/
	
	vector<vector<Vec4i>> output(2);
	Point p1, p2;
	vector<double> slopes;
	vector<Vec4i> final_lines, left_lines, right_lines;
	double slope_thresh = 0.3;

	//검출된 직선들의 기울기를 계산
	for (int i = 0; i < lines.size(); i++) {
		Vec4i line = lines[i];
		p1 = Point(line[0], line[1]); //직선의 시작점
		p2 = Point(line[2], line[3]); //직선의 끝나는점

		double slope;
		if (p2.x - p1.x == 0)  //너무 수직인 직선은 특별한 값
			slope = 999.0;
		else
			slope = (p2.y - p1.y) / (double)(p2.x - p1.x);

		//기울기가 너무 수평인 선은 제외
		if (abs(slope) > slope_thresh) {
			slopes.push_back(slope);
			final_lines.push_back(line);
		}
	}

	//선들을 좌우 선으로 분류
	img_center = (double)((img_edges.cols / 2));

	for (int i = 0; i < final_lines.size(); i++) {
		p1 = Point(final_lines[i][0], final_lines[i][1]); //직선의 시작점과 끝나는 점
		p2 = Point(final_lines[i][2], final_lines[i][3]); //직선의 시작점과 끝나는 점 

		if (slopes[i] > 0 && p1.x > img_center && p2.x > img_center) {
			right_detect = true;
			right_lines.push_back(final_lines[i]);
		}
		else if (slopes[i] < 0 && p1.x < img_center && p2.x < img_center ) {
			left_detect = true;
			left_lines.push_back(final_lines[i]);
		}
	}

	output[0] = right_lines;
	output[1] = left_lines;
	return output;
}

vector<Point> RoadLaneDetector::regression(vector<vector<Vec4i>> separatedLines, Mat img_input) {
	/*
		선형 회귀를 통해 좌우 차선 각각의 가장 적합한 선을 찾는다.
	*/
	vector<Point> output(4);
	Point p1, p2, p3, p4;
	Vec4d left_line, right_line;
	vector<Point> left_points, right_points;

	if (right_detect) {
		for (auto i : separatedLines[0]) {
			p1 = Point(i[0], i[1]); //현재 직선의 시작점 좌표
			p2 = Point(i[2], i[3]); //현재 직선의 끝점 좌표

			right_points.push_back(p1);
			right_points.push_back(p2);
		}

		if (right_points.size() > 0) {
			//주어진 contour에 최적화된 직선 추출
			fitLine(right_points, right_line, DIST_L2, 0, 0.01, 0.01);
			
			right_m = right_line[1] / right_line[0];  //기울기
			right_b = Point(right_line[2], right_line[3]); //직선위의 한 점 
		}
	}
	
	if (left_detect) {
		for (auto j : separatedLines[1]) {
			p3 = Point(j[0], j[1]);
			p4 = Point(j[2], j[3]);

			left_points.push_back(p3);
			left_points.push_back(p4);
		}

		if (left_points.size() > 0) {
			//주어진 contour에 최적화된 직선 추출
			fitLine(left_points, left_line, DIST_L2, 0, 0.01, 0.01);
			
			left_m = left_line[1] / left_line[0];  //기울기
			left_b = Point(left_line[2], left_line[3]); 
		}
	}

	//좌우 선 각각의 두 점을 계산한다.
	//y = m*x + b  --> x = (y-b) / m
	int y1 = img_input.rows;
	int y2 = 470;

	double right_x1 = ((y1 - right_b.y) / right_m) + right_b.x;
	double right_x2 = ((y2 - right_b.y) / right_m) + right_b.x;

	double left_x1 = ((y1 - left_b.y) / left_m) + left_b.x;
	double left_x2 = ((y2 - left_b.y) / left_m) + left_b.x;

	output[0] = Point(right_x1, y1); //에 오른쪽 차선의 첫 번째 점 (right_x1, y1)을 저장합니다.
	output[1] = Point(right_x2, y2); //output[1]에 오른쪽 차선의 두 번째 점 (right_x2, y2)을 저장합니다.
	output[2] = Point(left_x1, y1); //output[2]에 왼쪽 차선의 첫 번째 점 (left_x1, y1)을 저장합니다.
	output[3] = Point(left_x2, y2); //output[3]에 왼쪽 차선의 두 번째 점 (left_x2, y2)을 저장합니다.


	//    // 다항식 피팅을 통한 차선 검출
    // vector<Point> right_lane = fitPolynomialLane(right_points, 2);
    // vector<Point> left_lane = fitPolynomialLane(left_points, 2);

    // // 차선의 첫 번째 점과 마지막 점을 출력에 저장
    // output[0] = right_lane.front();
    // output[1] = right_lane.back();
    // output[2] = left_lane.front();
    // output[3] = left_lane.back();
	return output;
}

string RoadLaneDetector::predictDir() {
	/*
		두 차선이 교차하는 지점(사라지는 점)이 중심점으로부터
		왼쪽에 있는지 오른쪽에 있는지로 진행방향을 예측한다.
	*/
	
	string output;
	double x, threshold =  10;
	
	//두 차선이 교차하는 지점 계산
	x = (double)(((right_m * right_b.x) - (left_m * left_b.x) - right_b.y + left_b.y) / (right_m - left_m));

	if (x >= (img_center - threshold) && x <= (img_center + threshold))
		output = "Straight";
	else if (x > img_center + threshold)
		output = "Right Turn";
	else if (x < img_center - threshold)
		output = "Left Turn";
	
	return output;
}
double RoadLaneDetector::calculateAngle(double slope1 , double slope2) { // 기울기를 각도로 변환하는 함수

    double angle = atan((slope1 - slope2) / (1 + slope1 * slope2));
    return angle * 180 / CV_PI; // 라디안을 도(degree)로 변환
}


double RoadLaneDetector::calculateSteeringAngle(double right_m, double left_m) {
	// 차량의 현재 방향의 기울기 (기본적으로 0)
	double vihicle_m = 0 ;
	// 오른쪽 차선과의 각도
	right_angle = calculateAngle(vihicle_m, right_m);
	// 왼쪽 차선과의 각도
	left_angle = calculateAngle(vihicle_m, left_m);

	double steering_angle = (right_angle + left_angle) / 2;

	return steering_angle;
}
double RoadLaneDetector:: controlSteering (double right_m , double left_m) {
	double steering_angle = calculateSteeringAngle(right_m, left_m);

	return steering_angle;
}
Mat RoadLaneDetector::drawLine(Mat img_input, vector<Point> lane, string dir) {
	/*
		좌우 차선을 경계로 하는 내부 다각형을 투명하게 색을 채운다.
		예측 진행 방향 텍스트를 영상에 출력한다.
		좌우 차선을 영상에 선으로 그린다.
	*/
	
	vector<Point> poly_points;
	Mat output;
	img_input.copyTo(output);

	poly_points.push_back(lane[2]);
	poly_points.push_back(lane[0]);
	poly_points.push_back(lane[1]);
	poly_points.push_back(lane[3]);



	fillConvexPoly(output, poly_points, Scalar(0,230, 30), LINE_AA, 0);  //다각형 색 채우기
	addWeighted(output, 0.3, img_input, 0.7, 0, img_input);  //영상 합하기

	//예측 진행 방향 텍스트를 영상에 출력
	putText(img_input, dir, Point(520, 100), FONT_HERSHEY_PLAIN, 3, Scalar(255, 255, 255), 3, LINE_AA);

	//좌우 차선 선 그리기
	line(img_input, lane[0], lane[1], Scalar(0, 255, 255), 5, LINE_AA);
	line(img_input, lane[2], lane[3], Scalar(0, 255, 255), 5, LINE_AA);
	
	return img_input;
}

// // 다항식 피팅 함수
// Mat RoadLaneDetector::polyfit(const vector<Point>& points, int degree) {
//     Mat1d X(points.size(), degree + 1);
//     Mat1d y(points.size(), 1);

//     for (size_t i = 0; i < points.size(); ++i) {
//         for (int j = 0; j <= degree; ++j) {
//             X(i, j) = pow(points[i].x, j);
//         }
//         y(i, 0) = points[i].y;
//     }

//     Mat1d coef;
//     solve(X, y, coef, DECOMP_QR);

//     return coef;
// }

// // 다항식 함수
// double RoadLaneDetector::polynomial(const Mat& coef, double x) {
//     double y = 0.0;
//     for (int i = 0; i < coef.rows; ++i) {
//         y += coef.at<double>(i, 0) * pow(x, i);
//     }
//     return y;
// }

// vector<Point> RoadLaneDetector::fitPolynomialLane(const vector<Point>& points, int degree) {
//     Mat coef = polyfit(points, degree);
//     vector<Point> fittedLane;

//     for (int x = 0; x < 1280; ++x) { // assuming image width is 1280
//         double y = polynomial(coef, x);
//         fittedLane.push_back(Point(x, y));
//     }

//     return fittedLane;
// }


#include <opencv2/highgui/highgui.hpp>
#include<iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include <opencv2/dnn.hpp>  // DNN 모듈을 포함합니다.
using namespace std;
using namespace cv;

class RoadLaneDetector
{
private:
    double previous_heading_error = 0.0; // 이전 헤딩 에러
double wheelbase = 0.75;


public:
int prev_rx[12] = {0}; 
int prev_y_vals_right[12] = {0};
double left_coeffs_array[3] = {0}; // 왼쪽 차선 계수
double right_coeffs_array[3] = {0}; // 오른쪽 차선 계수
 double coeffs_vector[3] = {0};
    int currentSize = 0; 
    int currentSize1 = 0 ;
    vector<int> right_cluster_indices;
    vector<int> ry_cluster_indicies; 
    vector <int> left_cluster_indices;
    bool stopDetection;
    int lx[12] = {0} ; 
    int rx[12] = {0} ;
    int count = 0 ; 
Mat transformed_frame;
    // x, y 값을 저장할 벡터 생성
    vector<int> x_vals;
    vector<int> y_vals;
    int y_vals_left[12] = {0};
    int y_vals_right[12] = {0};
    int Radius_Of_Curvature ;
vector<int> histogram;
Mat transform_matrix;
double ROC(const int *x_vals , const int*y_vals);
void clusterLeftLane(const vector<int>& lx, const vector<int>& ly, double threshold);
void clusterRightLane(const vector<int>& rx, const vector<int>& ry, double threshold);
double calculateRadius(double a, double b, double x);
Mat Reverse_transformed(Mat result , Mat transfrom_matrix);
Mat bird_eye_view (Mat img_frame);
vector <int> Start_lane_detection (Mat mask);
Mat sliding_window(Mat img_frame,Mat mask,int left_base,int right_base);
Mat img_filter(Mat transformed_frame);
void polyfit(const int* x_vals, const int* y_vals, int degree);
// double calculateCurvature(double a, double b, double x);

double calculateCurvature(const vector<double>& coeffs, double x);
};
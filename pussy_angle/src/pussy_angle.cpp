#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "RoadLaneDetector.h"
#include <fstream>  // 파일 입출력을 위한 헤더 추가
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
using namespace cv;
using namespace std;

void on_trackbar(int, void*) {}

int l_h = 0, l_s = 0, l_v = 100; // 초기값 설정
int u_h = 255, u_s = 255, u_v = 255;
int l_h_hls = 0, l_l = 0, l_s_hls = 100; // HLS 하한 값
int u_h_hls = 255, u_l = 255, u_s_hls = 255; // HLS 상한 값
void createTrackbars() {
    // 트랙바 생성
    namedWindow("Trackbars");
    createTrackbar("L - H", "Trackbars", &l_h, 179, on_trackbar);
    createTrackbar("L - S", "Trackbars", &l_s, 255, on_trackbar);
    createTrackbar("L - V", "Trackbars", &l_v, 255, on_trackbar);
    createTrackbar("U - H", "Trackbars", &u_h, 179, on_trackbar);
    createTrackbar("U - S", "Trackbars", &u_s, 255, on_trackbar);
    createTrackbar("U - V", "Trackbars", &u_v, 255, on_trackbar);



	createTrackbar("L - H1", "Trackbars", &l_h, 179, on_trackbar);
    createTrackbar("L - L1", "Trackbars", &l_s, 255, on_trackbar);
    createTrackbar("L - S1", "Trackbars", &l_v, 255, on_trackbar);
    createTrackbar("U - H1", "Trackbars", &u_h, 179, on_trackbar);
    createTrackbar("U - L1", "Trackbars", &u_s, 255, on_trackbar);
    createTrackbar("U - S1", "Trackbars", &u_v, 255, on_trackbar);
}
double steering_left_angle=0;
double steering_left_angle_degrees=0 ;
double steering_right_angle=0;
double steering_right_angle_degrees=0;
double steering_angle=0;
int main(int argc , char **argv)
{

    ofstream csv_file1("polynomial_coefficients.csv");
    if (csv_file1.is_open()) {
        csv_file1 << "Frame,Polynomial Degree,Left Coefficients,Right Coefficients\n";
    } else {
        cerr << "Unable to open file: polynomial_coefficients.csv" << endl;
        return -1;
    }
    ofstream csv_file("lane_coordinates.csv");
    if (!csv_file.is_open()) {
        cerr << "Error: Could not open the file 'lane_coordinates.csv' for writing." << endl;
        return -1;
    }

    csv_file << "Left X,Left Y,Right X,Right Y\n";  // CSV 헤더

	RoadLaneDetector roadLaneDetector;

	Mat img_frame, img_filter, img_edges, img_mask, img_lines, img_result;
	Mat bird_of_eye_view;
	vector<Vec4i> lines;
	vector<vector<Vec4i>> separated_lines;
	vector<Point> lane;
	string dir;
	   string turnDirection;

	vector<double> polyregre;
    VideoCapture video("/dev/video2", cv::CAP_V4L2);
	// VideoCapture video("test_video.mp4");
		if (!video.isOpened())
	{
		cout << "동영상 파일을 열 수 없습니다. \n" << endl;
		getchar();
		return -1;
	}

	video.read(img_frame);
	
	if (img_frame.empty())
	{
		cout << "영상 프레임이 비어 있습니다.\n";
		return -1;
	}

	VideoWriter writer;
	int codec = VideoWriter::fourcc('X', 'V', 'I', 'D'); // 원하는 코덱 선택
	double fps = 25.0;                                    // 프레임
	string filename = "./result.avi";                     // 결과 파일

	writer.open(filename, codec, fps, img_frame.size(), true);
	if (!writer.isOpened())
	{
		cout << "출력을 위한 비디오 파일을 열 수 없습니다. \n";
		return -1;
	}
	// get() 함수로 카메라 또는 동영상 파일로부터 여러 정보를 받아 올 수 있음. 매개변수는 열거형 상수 Properties 가 들어감
    ros::init(argc, argv, "push_angle_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(60);
    geometry_msgs::Twist push;
    ros::Publisher publisher_push = n.advertise<geometry_msgs::Twist>("push", 1000);
    int img_center = 640;

int frame_count = 0 ;
	while (ros::ok)
	{        frame_count++;

        if (!video.read(img_frame))
            break;

        cv::resize(img_frame, img_frame, cv::Size(1280, 720));

        Mat transformed_frame = roadLaneDetector.bird_eye_view(img_frame);
        Mat mask = roadLaneDetector.img_filter(transformed_frame);
        
        vector<int> l = roadLaneDetector.Start_lane_detection(mask);
        Mat msk = mask.clone();

        Mat result = roadLaneDetector.sliding_window(transformed_frame, mask, l[0], l[1]);

        const double WHEELBASE = 0.75;

        if (roadLaneDetector.lx.size() != 0 && roadLaneDetector.lx.size() > 4) {
            vector<double> left_coeffs = roadLaneDetector.polyfit(roadLaneDetector.lx, roadLaneDetector.y_vals_left, 2);
            vector<double> right_coeffs = roadLaneDetector.polyfit(roadLaneDetector.rx, roadLaneDetector.y_vals_right, 2);


            // double root =-left_coeffs[1]+sqrt(left_coeffs[1]*left_coeffs[1] - 4 * left_coeffs[0]*left_coeffs[2])/2*left_coeffs[0];
            // double root1 =-left_coeffs[1]-sqrt(left_coeffs[1]*left_coeffs[1] - 4 * left_coeffs[0]*left_coeffs[2])/2*left_coeffs[0];
            // double d = left_coeffs[1]*left_coeffs[1] - 4 * left_coeffs[0]*left_coeffs[2];

            // cout <<"d = "<<d <<endl;

            // 다항식 계수 저장
            if (csv_file1.is_open()) {
                    cout << "csv_file_open" <<endl;
                for (const auto& coeff : left_coeffs) {
                    csv_file1 << coeff << " ";
                }
                for (const auto& coeff : right_coeffs) {
                    csv_file1 << coeff << " ";
                }
                csv_file1 << "\n";
            }

            // 슬라이딩 윈도우로 얻은 좌표 저장
            if (csv_file.is_open()) {
                for (size_t i = 0; i < roadLaneDetector.lx.size(); ++i) {
                    csv_file  << roadLaneDetector.lx[i] << "," << roadLaneDetector.y_vals_left[i] << ","
                             << roadLaneDetector.rx[i] << "," << roadLaneDetector.y_vals_right[i] << "\n";
                }
            }

            for (double x : roadLaneDetector.lx) {
                double curvature = roadLaneDetector.calculateCurvature(left_coeffs, static_cast<double>(x));
                steering_left_angle = atan(WHEELBASE / curvature);
                steering_left_angle_degrees = steering_left_angle * (180.0 / M_PI);
                cout.precision(4);
                // cout << "Calculated curvature at x = " << x << " is: " << curvature << endl;
                        circle(transformed_frame , Point(roadLaneDetector.lx[x], 0) ,5 , (0,0,255),-1);  //720 이랑의 교점 y= 720 이랑의 교점

                                    double middleX = (roadLaneDetector.rx[x] + roadLaneDetector.lx[x]) / 2;

            // 중앙값과 비교하여 좌회전 또는 우회전 판단
         
            if (middleX < img_center) {
                turnDirection = "Left Turn";
            } else if (middleX > img_center) {
                turnDirection = "Right Turn";
            } else {
                turnDirection = "Straight";
            }

            cout << "Detected turn direction: " << turnDirection << endl;

            }

            for (double x : roadLaneDetector.rx) {
                double curvature = roadLaneDetector.calculateCurvature(right_coeffs, static_cast<double>(x));
                steering_right_angle = atan(WHEELBASE / curvature);
                steering_right_angle_degrees = steering_right_angle * (180.0 / M_PI);

                circle(transformed_frame , Point((roadLaneDetector.rx[x]+roadLaneDetector.lx[x])/2, 0) ,5 , (0,0,255),-1);  //720 이랑의 교점 y= 720 이랑의 교점

            }

        
            if (turnDirection == "right Turn") {
                steering_angle =  -(steering_left_angle_degrees + steering_right_angle_degrees) / 2;

            } 
            else {
            steering_angle = (steering_left_angle_degrees + steering_right_angle_degrees) / 2;
            }
        }
        // circle(transformed_frame , Point(1280,0 ) ,5 , (0,0,255),-1);  //
        // circle(transformed_frame , Point(0,0 ) ,5 , (0,0,255),-1);  //y=0 과의 교점 이랑의 교점 을 구해서 시점과 종점 구해서 그 두개를 빼서 좌회전 우회전 구분.
    


        cout <<"Steering_angle"<<steering_angle <<endl;

        imshow("img_frame", img_frame);
        imshow("transformed_frame", transformed_frame);
        imshow("mask", mask);
        imshow("result", result);

        writer.write(img_frame);

        if (waitKey(100) == 27) {
            break;
        }

push.angular.z = steering_angle ;
publisher_push.publish(push);
cout <<"Steering_angle" <<steering_angle<<endl;

// 곡률 (단위: 1/미터)

// 방향각 계산


// 결과를 디그리로 변환
		// imshow("img_frame",img_frame);
		// imshow("transformed_frame",transformed_frame);
		imshow("mask",mask);
		imshow("result",result);
	writer.write(img_frame);
		// esc 키 종료
		if (waitKey(100) == 27){
	
			break; 
			
			}

	}



	video.release();
	writer.release();
	
	return 0;
}

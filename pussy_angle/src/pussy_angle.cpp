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

int l_h = 0, l_s = 0, l_v = 150; // 초기값 설정
int u_h = 255, u_s = 255, u_v = 255;
int l_h_hls = 0, l_l = 0, l_s_hls = 230; // HLS 하한 값
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

double steering_left_angle = 0;
double steering_left_angle_degrees = 0;
double steering_right_angle = 0;
double steering_right_angle_degrees = 0;
double steering_angle = 0;
RoadLaneDetector roadLaneDetector;

int main(int argc , char**argv) {


    createTrackbars();
    int imageCenterX = 640;  // 이미지 중앙 X값 계산
    ofstream csv_file1("polynomial_coefficients.csv");
    if (csv_file1.is_open()) {
        csv_file1 << "Left Coefficients,Right Coefficients\n";
    } else {
        cerr << "Unable to open file: polynomial_coefficients.csv" << endl;
        return -1;
    }

    ofstream csv_file("lane_coordinates.csv");
    if (!csv_file.is_open()) {
        cerr << "Error: Could not open the file 'lane_coordinates.csv' for writing." << endl;
        return -1;
    }

    csv_file << "Frame,Left X,Left Y,Right X,Right Y\n";  // CSV 헤더
    Mat img_frame, img_filter, img_edges, img_mask, img_lines, img_result;
    vector<Vec4i> lines;
    vector<vector<Vec4i>> separated_lines;
    vector<Point> lane;
    string dir;
    vector<double> polyregre;
   string turnDirection;

    VideoCapture video("asdf.mp4");
    // VideoCapture video("/dev/video2", cv::CAP_V4L2);

    if (!video.isOpened()) {
        cout << "동영상 파일을 열 수 없습니다. \n" << endl;
        getchar();
        return -1;
    }

    video.read(img_frame);
    if (img_frame.empty()) {
        cout << "영상 프레임이 비어 있습니다.\n";
        return -1;
    }

    VideoWriter writer;
    int codec = VideoWriter::fourcc('X', 'V', 'I', 'D');
    double fps = 25.0;
    string filename = "./result.avi";

    writer.open(filename, codec, fps, img_frame.size(), true);
    if (!writer.isOpened()) {
        cout << "출력을 위한 비디오 파일을 열 수 없습니다. \n";
        return -1;
    }
    const double WHEELBASE = 0.75;
    int img_center = 640;
    int frame_count = 0;
        ros::init(argc, argv, "push_angle_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(60);
    geometry_msgs::Twist push;
    ros::Publisher publisher_push = n.advertise<geometry_msgs::Twist>("push", 1000);
    while (ros::ok) {
            int sum=0;
            int average = 0; 

        frame_count++;
    
        if (!video.read(img_frame))
            break;

        cv::resize(img_frame, img_frame, cv::Size(1280, 720));
        roadLaneDetector.transformed_frame = roadLaneDetector.bird_eye_view(img_frame);

        Mat mask = roadLaneDetector.img_filter(roadLaneDetector.transformed_frame);
 int count = 0 ; 

// 오늘 할 것 차선이 없을떄 즉 데이터가 없을때 어떻게 할 것인지. y값은 보정 쉬운데 x값은 어떻게 보정할지에 대해서.
// 다항회귀 하고 곡률 구해서 각도 주는 코드를 다시 만들어야함 잘못 됏음;
// prev_rx 그전 rx 배열에 저장을 해주어야겠다 목표는 터미널 창에 그전 값이 잘 출력되어야함
        vector<int> l = roadLaneDetector.Start_lane_detection(mask);
        Mat msk = mask.clone();
        Mat result = roadLaneDetector.sliding_window(roadLaneDetector.transformed_frame, mask, l[0], l[1]);
        for (int i = 0 ; i < 12 ; i ++) {
            // cout <<"roadLaneDetector.rx [i] = " <<roadLaneDetector.rx[i] <<endl; 
            // cout <<"roadLaneDetector.y_vals_right[i] = " <<roadLaneDetector.y_vals_right[i] <<endl; 
        if (roadLaneDetector.rx[i] !=0 )  {
            count++;
            sum += roadLaneDetector.rx[i];

        }




            // cout <<"roadLaneDetector.lx[i] = " <<roadLaneDetector.lx[i] <<endl; 
            // cout <<"roadLaneDetector.y_vals_left[i] = " <<roadLaneDetector.y_vals_left[i] <<endl; 
            //평균을 구해서 그 데이터들을 넣어주고 싶은데 빈곳에다가 근데  갯수를 모름 갯수를 세주
        }
 if (count >0) {       
average = sum/count;
 }
for (int i = 0 ; i <12 ; i ++) {
    if (roadLaneDetector.rx[i]==0) {
        roadLaneDetector.rx[i]=average;
    }
}

// for (int i = 0 ; i <12 ; i ++) {
//     cout <<"prev_rx = " <<roadLaneDetector.prev_rx[i] <<endl; 
//     cout <<" prev_y_vals_right" <<roadLaneDetector.prev_y_vals_right[i] <<endl; 
// }


        if (roadLaneDetector.currentSize ==12) {
        roadLaneDetector.polyfit(roadLaneDetector.lx , roadLaneDetector.y_vals_left,2);

            for (int i = 0; i < 3; i++) {
        roadLaneDetector.left_coeffs_array[i] = roadLaneDetector.coeffs_vector[i];
    }
        if (csv_file1.is_open()) {
        for (const auto& coeff : roadLaneDetector.left_coeffs_array) {
            csv_file1 << coeff << " ";
        }
        csv_file1.flush();  // 버퍼 플러시
    } else {
        std::cerr << "Failed to open file for writing left_coeffs." << std::endl;
    }


        roadLaneDetector.currentSize = 0 ; 
        
        }
        if (roadLaneDetector.currentSize1==12) {
        roadLaneDetector.polyfit(roadLaneDetector.rx , roadLaneDetector.y_vals_right,2);
        for (int i = 0; i < 3; i++) {
        roadLaneDetector.right_coeffs_array[i] = roadLaneDetector.coeffs_vector[i];
        }
 if (csv_file1.is_open()) {
        for (const auto& coeff : roadLaneDetector.right_coeffs_array) {
            csv_file1 << coeff << " ";
        }
        csv_file1 << "\n";
        csv_file1.flush();  // 버퍼 플러시
    } else {
        std::cerr << "Failed to open file for writing right_coeffs." << std::endl;
    }
            
            
            
            roadLaneDetector.currentSize1 = 0 ;
        }

double left_steering_angle =roadLaneDetector.ROC(roadLaneDetector.lx,roadLaneDetector.y_vals_left);
double right_steering_angle = roadLaneDetector.ROC(roadLaneDetector.rx,roadLaneDetector.y_vals_right);

double real_steering_angle = (left_steering_angle+right_steering_angle)/2;
// cout <<"roadLaneDetector.lx[0] = " <<roadLaneDetector.lx[0] <<endl; 
// cout <<"roadLaneDetector.rx[0] = "  <<roadLaneDetector.rx[0] <<endl;
// cout <<"roadLaneDetector.lx[11] = " <<roadLaneDetector.lx[11] <<endl;
// cout <<"roadLaneDetector.rx[11] = " <<roadLaneDetector.rx[11] <<endl;  

// circle(roadLaneDetector.transformed_frame, Point((roadLaneDetector.lx[11]+roadLaneDetector.rx[11])/2,720) ,5 , (0,0,255),-1);  //720 이랑의 교점 y= 720 이랑의 교점
// circle(roadLaneDetector.transformed_frame, Point(640,720) ,5 , (0,0,255),-1);  //720 이랑의 교점 y= 720 이랑의 교점
if (img_center <  (roadLaneDetector.lx[11]+roadLaneDetector.rx[11])/2) {
    real_steering_angle = real_steering_angle*-1;
}


cout <<"real_steering_angle = " <<real_steering_angle <<endl; 

 push.angular.z = real_steering_angle ;
publisher_push.publish(push);

            // 슬라이딩 윈도우로 얻은 좌표 저장
            if (csv_file.is_open()) {
                for (size_t i = 0; i < sizeof(roadLaneDetector.lx)/sizeof(roadLaneDetector.lx[0]); ++i) {
                    if (roadLaneDetector.lx!=0 && roadLaneDetector.rx !=0) {
                    csv_file  << roadLaneDetector.lx[i] << "," << roadLaneDetector.y_vals_left[i] << ","
                             << roadLaneDetector.rx[i] << "," << roadLaneDetector.y_vals_right[i] << "\n";
                
                }
                csv_file.flush();
                }

            }

       

        imshow("img_frame", img_frame);
        imshow("transformed_frame", roadLaneDetector.transformed_frame);
        imshow("mask", mask);
        imshow("result", result);

        writer.write(img_frame);

        if (waitKey(10) == 27) {
            break;
        }
    }

    video.release();
    writer.release();
    csv_file1.close();
    csv_file.close();

    return 0;
}

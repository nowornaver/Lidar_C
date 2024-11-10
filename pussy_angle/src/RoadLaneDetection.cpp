#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "RoadLaneDetector.h"
#include <iostream>
#include <string>
#include <vector>

// L = 75cm


// 2차 미분 계산 함수


// 곡률 계산 함수
double RoadLaneDetector:: calculateCurvature(const vector<double>& coeffs, double x) {
    double curvature = 0 ;

    if (coeffs.size() !=0 && x!=0) {
    double a0 = coeffs[0]; //상수
    double a1 = coeffs[1]; //1차
    double a2 = coeffs[2]; //2차
    double first_derivative = a1 + 2 * a2 * x; //1차 도함수
    double second_derivative = 2 * a2; //2차 도함수

    curvature = abs(second_derivative * first_derivative) / abs(pow(first_derivative , 3));
    }
    return 1/curvature;
}

void RoadLaneDetector::polyfit(const int* x_vals, const int* y_vals, int degree) {
     



    int n = 12; //n 행의 수 


    // cout <<"n = " <<n <<endl; 
    Mat X(n, degree + 1, CV_64F); //degree+1 열
    Mat Y(n, 1, CV_64F);

    for (int i = 0; i < n; i++) {
        if (x_vals[i]!= 0) {
        Y.at<double>(i, 0) = static_cast<double>(y_vals[i]);
        for (int j = 0; j <= degree; j++) {
            X.at<double>(i, j) = pow(x_vals[i], j); //x_vals[i]^j
        }

        }
    }

    // (X^T * X) * a = X^T * Y 를 풀어서 회귀계수 a를 구함
    Mat Xt = X.t(); //행렬 x의 전치행렬 
    Mat XtX = Xt * X;
    Mat XtY = Xt * Y;
    Mat coeffs = XtX.inv() * XtY;
    for (int i = 0; i < coeffs.rows; i++) {
        coeffs_vector[i]= (coeffs.at<double>(i, 0));
    }

    
    
    }
    

Mat RoadLaneDetector::Reverse_transformed(Mat result , Mat transfrom_matrix) {
Mat a ; 
cv::warpPerspective(result, a, transfrom_matrix, cv::Size(640, 480));



    return a;
}


Mat RoadLaneDetector::bird_eye_view (Mat img_frame) {

int width = 1280;
int height = 720;
		Point2f src_vertices[4];

    src_vertices[0] = Point(width*0.20,height*0.9);  // Bottom-left
    src_vertices[1] = Point(width * 0.90, height*0.9);  // Bottom-right
    src_vertices[2] = Point(width * 0.58, height * 0.65);  // Top-right
    src_vertices[3] = Point(width * 0.40, height * 0.65);  // Top-left
	// cv::circle(img_frame,src_vertices[0],5,(0,0,255),-1);
	// cv::circle(img_frame,src_vertices[1],5,(0,0,255),-1);
	// cv::circle(img_frame,src_vertices[2],5,(0,0,255),-1);
	// cv::circle(img_frame,src_vertices[3],5,(0,0,255),-1);
	 Point2f dst_vertices[4];
    dst_vertices[0] = Point(width * 0.1,height);  // Bottom-left
    dst_vertices[1] = Point(width * 0.8, height);  // Bottom-right
    dst_vertices[2] = Point(width*0.8, 0);  // Top-right
    dst_vertices[3] = Point(width*0.1, 0);  // Top-left
    // cv::circle(img_frame,dst_vertices[0],5,(0,0,255),-1);
	// cv::circle(img_frame,dst_vertices[1],5,(0,0,255),-1);
	// cv::circle(img_frame,dst_vertices[2],5,(0,0,255),-1);
	// cv::circle(img_frame,dst_vertices[3],5,(0,0,255),-1);
	Mat matrix = cv::getPerspectiveTransform(src_vertices,dst_vertices);
    transform_matrix = cv::getPerspectiveTransform(dst_vertices,src_vertices);
	Mat transformed_frame;
cv::warpPerspective(img_frame, transformed_frame, matrix, cv::Size(1280, 720));
    Mat mask = Mat::zeros(img_frame.size(), CV_8UC1);
    Point poly[1][4];

    poly[0][0] = dst_vertices[0];
    poly[0][1] = dst_vertices[1];
    poly[0][2] = dst_vertices[2];
    poly[0][3] = dst_vertices[3];
    const Point* pts[1] = { poly[0] };
    int npts = 4;

    // 다각형 영역을 1로 채우기
    cv::fillPoly(mask, pts, &npts, 1, Scalar(255));
 Mat masked_frame;
    cv::bitwise_and(transformed_frame, transformed_frame, masked_frame, mask);
    return masked_frame; 
}
vector<int> RoadLaneDetector::Start_lane_detection(Mat mask) {

    vector<int> lane(2, 0); // 크기가 2인 벡터로 초기화 (왼쪽과 오른쪽 차선 위치)
 histogram.resize(mask.cols, 0); // 멤버 변수 histogram 초기화
         for (int i = 0; i < mask.cols; i++) {
            if (countNonZero(mask.col(i)) >100)
            histogram[i] = countNonZero(mask.col(i));
        }

        // 히스토그램 출력
  
		int midpoint = histogram.size() / 2; //midpoint = 320

// // 4. 왼쪽 차선의 시작 위치를 찾습니다.
int left_base = 0;
for (int i = 0; i < midpoint; i++) {
    if (histogram[i] > histogram[left_base]) {
        left_base = i;
        lane[0] = left_base;
    }
}

// // 5. 오른쪽 차선의 시작 위치를 찾습니다.
int right_base = midpoint;
for (int i = midpoint; i < histogram.size(); i++) {
    if (histogram[i] > histogram[right_base]) {
        right_base = i;
        lane[1]= right_base;
    }
}


    return lane;
}

Mat RoadLaneDetector::sliding_window(Mat img_frame, Mat mask, int left_base, int right_base) {
    // cout <<"left_base" <<left_base<<endl ; 
    Mat msk = mask.clone();
    int y = mask.rows;
    currentSize1 = 0;
// cout <<"count " <<count << endl; 
if (left_base!=0 && right_base!=0 ) {


    while (y > 0) { //6번 실행됌 while 문  //720 
  //y = 720 600 480 
        // 왼쪽 차선 범위 설정
        int left_start = max(0, left_base - 80);  // 윈도우 너비 확장 
        int left_end = min(mask.cols, left_base + 80);  // 윈도우 너비 확장
        if (left_start >= left_end) break; // 유효한 범위가 없으면 종료
        Mat img_left = mask.rowRange(y - 60, y).colRange(left_start, left_end);  // 윈도우 높이 확장 (120 픽셀로 증가)


        // imshow("img_left",img_left);
        // cout <<"Y1 =" <<y <<endl; 

        // vector<vector<Point>> contours_left;
            // cout <<"y1 = " <<y <<endl ; //720
        int non_zero_count = countNonZero(img_left);
        // cout <<"non_zero_count" <<non_zero_count <<endl;  //if non_zero_count =0 일 경우에 어떻게 할 것인가. 즉 흰색 픽셀이 나오지 않았을 경우 어떻게 할것인가. 정해야댐
        // 그리고 최소한의 임계값(threshold) 흰색 픽셀이 몇개 이상인지도 정하면 노이즈에 강력하게 만들 수 있을 것 같음.
        // cv::findContours(img_left, contours_left, RETR_TREE, CHAIN_APPROX_SIMPLE); //M.m00 값이 0이 안나오게 하는게 내 목표 
        
        // 왼쪽 차선의 중심 계산
        if (non_zero_count> 0 ) {
            // cout <<"Y2 = " <<y <<endl; 
            Moments M = moments(img_left,true);  //슬라이딩 윈도우 첫번째 y1 = 660 , y2 = 540 y3= 420 y3 = 300 y4 = 180 y5 = 60
            // cout <<"M.m00 = " <<M.m00 <<endl; 
            if (M.m00 != 0) { 
                int cx = static_cast<int>(M.m10 / M.m00);
                if (currentSize <12) {  //currentSize 여기서 012345 까지 돔
                // cout <<"CurrentSize"<<currentSize<<endl;
                // cout <<"Y3 = " <<y <<endl; 

                // cout <<"left_start+cx" <<left_start+cx <<endl;   
                lx[currentSize]=(left_start + cx); //lx 차선 픽셀 중심의 좌표

                // x_vals.push_back(left_start+c
                left_base = lx[currentSize];
                // cout <<"y-60 = "<<y-60 <<endl; 
                y_vals_left[currentSize] = y - 60; // y 값 추가 (중앙으로)
                // circle(transformed_frame, Point(lx[0],720) ,5 , (0,0,255),-1);  //720 이랑의 교점 y= 720 이랑의 교점
                // circle(transformed_frame, Point(lx[11], y_vals_left[11]) ,5 , (0,0,255),-1);  //720 이랑의 교점 y= 720 이랑의 교점
                // circle(transformed_frame, Point((lx[0]+lx[11])/2, 360) ,5 , (0,0,255),-1);  //720 이랑의 교점 y= 720 이랑의 교점
                // circle(transformed_frame, Point(lx[6], y_vals_left[6]) ,5 , (0,0,255),-1);  //720 이랑의 교점 y= 720 이랑의 교점


                            currentSize++;

                // cout <<"y_vals_left[0]" <<y_vals_left[0] <<endl ; 
                // y_vals.push_back(y-60);
                // cout <<currentSize <<endl; 
                }


                

            }


        }

//데이터 보정은 나중에 하고 일단은 그냥 곡률로 넘어가야겠다.. ㅅ1ㅂ
        // // 오른쪽 차선 범위 설정
        int right_start = max(0, right_base - 80);  // 윈도우 너비 확장
        int right_end = min(mask.cols, right_base + 80);  // 윈도우 너비 확장
        if (right_start >= right_end) break; // 유효한 범위가 없으면 종료

        Mat img_right = mask.rowRange(y - 60, y).colRange(right_start, right_end);  // 윈도우 높이 확장 (120 픽셀로 증가)
        int non_zero_count1 = countNonZero(img_right);
        // cout <<"non_zero_count1 = " <<non_zero_count1 <<endl; 
        // 오른쪽 차선의 중심 계산
        // if (non_zero_count1 == 0 ) {
        //     y_vals_right[currentSize1] = y-60;
        //     rx[currentSize1];

        // }
        if (non_zero_count1 > 0) {
            Moments M = moments(img_right , true);
            // cout <<"M.m00 = " <<M.m00<<endl; 
            if (M.m00 != 0) {
                int cx = static_cast<int>(M.m10 / M.m00);
                if (currentSize1 <12) {
                rx[currentSize1]=(right_start + cx);
                // cout <<"right_start + cx" << right_start + cx <<endl ; 
                right_base = right_start + cx;
                y_vals_right[currentSize1]=(y - 60); // y 값 추가 (중앙으로)
                // cout <<"rx[currentSize] = " << rx[currentSize] <<endl ; 
                // cout <<"y_vals_right[currentSize] = " << y_vals_right[currentSize] <<endl ; 

                circle(transformed_frame, Point(rx[currentSize1], y_vals_right[currentSize1]) ,5 , (0,0,255),-1);  //720 이랑의 교점 y= 720 이랑의 교점

                currentSize1++;

            }
        }
        
        }
        if (non_zero_count1 == 0 ) {

 
            rx[currentSize1] =prev_rx[currentSize1];
            y_vals_right[currentSize1] = y-60;
            currentSize1++;
        }

       

        // 현재 윈도우 표시
        cv::rectangle(msk, Point(left_base - 80, y), Point(left_base + 80, y - 60), Scalar(255, 255, 255), 2);
        cv::rectangle(msk, Point(right_base - 80, y), Point(right_base + 80, y - 60), Scalar(255, 255, 255), 2);

        // 다음 윈도우로 이동
        // cout <<"Y"<<y <<endl;
        // cout <<"CurrentSize" <<currentSize <<endl;



        y -= 60;  // 윈도우가 큰 만큼 더 많이 이동

        //왜 lx[0] 이 0이 나오는지?

    


    }
    
}
    return msk;
}
double RoadLaneDetector::ROC(const int *x_vals , const int*y_vals) {
//R = h/2 + w^2/8H
double steering_angle = 0;
double W = sqrt( pow(x_vals[0]-x_vals[11],2) +pow(720,2) ) ;
Point M = Point((x_vals[0] + x_vals[11]) / 2, 360); //A B 의 중점
if (x_vals[0]-x_vals[11] != 0) {
double m = 720/(x_vals[0]-x_vals[11]); //기울기
    double m_inv = -1.0 / m; // 기울기 m의 역수
    
      double min_difference = std::numeric_limits<double>::max();
    Point C = Point(x_vals[0],y_vals[0]); // 중점을 초기화

    // Arc 위의 점들 중에서 기울기 -m_inv와 가장 가까운 점을 찾음
    for (int i = 0; i < 11; i++) {

        double m_candidate = static_cast<double>(y_vals[i] - M.y) / (x_vals[i] - M.x);
        double difference = std::abs(m_candidate - m_inv);
        if (difference < min_difference) {
            min_difference = difference;
            C = Point(x_vals[i], y_vals[i]);
            
        }
    }

    // H 값 계산: 중점 M과 점 C 사이의 거리
    double H = sqrt(pow(C.x - M.x, 2) + pow(C.y - M.y, 2));
    // 곡률 계산
    
    double R = (H / 2.0) + (pow(W, 2) / (8.0 * H));
    steering_angle = tan(wheelbase/R);
    // if (steering_angle <=0.00001) {
    //        steering_angle =0;
    // }  
}
return steering_angle;
}
Mat RoadLaneDetector::img_filter (Mat transformed_frame) {



namedWindow("Trackbars");

   
    // 트랙바 값 읽기

        Mat hsv_transformed_frame;
		
        cvtColor(transformed_frame, hsv_transformed_frame, COLOR_BGR2HSV);
    int l_h = getTrackbarPos("L - H", "Trackbars");
    int l_s = getTrackbarPos("L - S", "Trackbars");
    int l_v = getTrackbarPos("L - V", "Trackbars");
    int u_h = getTrackbarPos("U - H", "Trackbars");
    int u_s = getTrackbarPos("U - S", "Trackbars");
    int u_v = getTrackbarPos("U - V", "Trackbars");

    Mat hls_transformed_frame;
    cvtColor(transformed_frame,hls_transformed_frame,COLOR_BGR2HLS);


    int l_h_hls = getTrackbarPos("L - H1", "Trackbars");
    int l_l = getTrackbarPos("L - L1", "Trackbars");
    int l_s_hls = getTrackbarPos("L - S1", "Trackbars");
    int u_h_hls = getTrackbarPos("U - H1", "Trackbars");
    int u_l = getTrackbarPos("U - L1", "Trackbars");
    int u_s_hls = getTrackbarPos("U - S1", "Trackbars");

    Mat mask1;

    Scalar lower1(l_h_hls , l_l , l_s_hls);
    Scalar upper1(u_h_hls,u_l,u_s_hls);
    cv::inRange(hls_transformed_frame , lower1 , upper1,mask1);

        // Create mask using the trackbar values
        Mat mask;
        Scalar lower(l_h, l_s, l_v);
        Scalar upper(u_h, u_s, u_v);
        cv::inRange(hsv_transformed_frame,lower, upper,mask);
Mat combined;
    cv::bitwise_or(mask, mask1, combined);


return combined;
}




#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include "RoadLaneDetector.h"
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/Twist.h>
    geometry_msgs::Twist cmd_vel;

int global_width = 1280;  // 기본 값 설정
int global_height = 720; // 기본 값 설정
double steering =0;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
        ROS_INFO_ONCE("4");
    try
    {
        Mat img_filter, img_edges, img_mask, img_lines, img_result;
        	vector<Vec4i> lines;
	    vector<vector<Vec4i>> separated_lines;
	    vector<Point> lane;
        string dir;

        RoadLaneDetector roadLaneDetector;
        ROS_INFO_ONCE("Subscribe Success");
        // 메시지를 OpenCV 이미지로 변환
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        // 이미지 처리 로직 추가
        //cv::namedWindow("image", cv::WINDOW_NORMAL); // 윈도우 크기 조절 가능
        cv::resize(image, image, cv::Size(global_width, global_height));
                //cv::imshow("image",image);

            img_filter = roadLaneDetector.filter_colors(image);
            //cv::imshow("img_filter",img_filter); //차선 검출 잘 되는지 확인 
            cvtColor(img_filter, img_filter, COLOR_BGR2GRAY);
       
            Canny(img_filter, img_edges, 50, 150); //확인완료
            cv::imshow("canny",img_edges);

            img_mask = roadLaneDetector.limit_region(img_edges);
            //cv::imshow("img_mask",img_mask);
		// 6. Hough 변환으로 에지에서의 직선 성분을 추출
		lines = roadLaneDetector.houghLines(img_mask);
    if (lines.size() > 0)
		{
			// 7. 추출한 직선성분으로 좌우 차선에 있을 가능성이 있는 직선들만 따로 뽑아서 좌우 각각 직선을 계산한다.
			// 선형 회귀를 하여 가장 적합한 선을 찾는다.
			separated_lines = roadLaneDetector.separateLine(img_mask, lines);
			lane = roadLaneDetector.regression(separated_lines, image);

			// 8. 진행 방향 예측
			dir = roadLaneDetector.predictDir();
            steering=roadLaneDetector.controlSteering(roadLaneDetector.right_m, roadLaneDetector.left_m);
            ROS_INFO("steering = %f",steering);
			// 9. 영상에 최종 차선을 선으로 그리고 내부 다각형을 색으로 채운다. 예측 진행 방향 텍스트를 영상에 출력
			img_result = roadLaneDetector.drawLine(image, lane, dir);
            
            cmd_vel.angular.z = steering;
		}

        cv::imshow("image",image);
        ROS_INFO("R_M = %f",roadLaneDetector.right_m);
        ROS_INFO("L_M = %f",roadLaneDetector.left_m);
        ROS_INFO("RIGHT_ANGLE = %f" , roadLaneDetector.right_angle);
        ROS_INFO("LEFT_ANGLE = %f",roadLaneDetector.left_angle);
        if (roadLaneDetector.left_detect) {
            ROS_INFO("왼쪽 탐지");
        }
        else if (roadLaneDetector.right_detect) {
            ROS_INFO("오른쪽탐지");
        }

            cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}
void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    global_width = msg->width;  // 전역 변수에 값 저장
    global_height = msg->height; // 전역 변수에 값 저장
    ROS_INFO_ONCE("Received camera info:");
    ROS_INFO_ONCE("Width: %d", global_width);
    ROS_INFO_ONCE("Height: %d", global_width);
    ROS_INFO_ONCE("Distortion model: %s", msg->distortion_model.c_str());
    //    ROS_INFO_ONCE("ROI:");
    // ROS_INFO_ONCE("  x_offset: %d", msg->roi.x_offset);
    // ROS_INFO_ONCE("  y_offset: %d", msg->roi.y_offset);
    // ROS_INFO_ONCE("  height: %d", msg->roi.height);
    // ROS_INFO_ONCE("  width: %d", msg->roi.width);
    // ROS_INFO_ONCE("  do_rectify: %d", msg->roi.do_rectify);
    // 추가적인 정보를 출력할 수 있습니다.
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_subscriber_node");
    ros::NodeHandle nh;
    ROS_INFO_ONCE("1");
    // image_transport 객체 생성
    image_transport::ImageTransport it(nh);
        ROS_INFO_ONCE("2");
    // 이미지 구독
    image_transport::Subscriber sub = it.subscribe("stereo/left/image_rect", 1, imageCallback);
        ROS_INFO_ONCE("3");
    ros::Subscriber sub_camera_info = nh.subscribe("stereo/left/camera_info", 1, cameraInfoCallback);
    ros::Publisher publisher_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    ros::spin();
    return 0;
}

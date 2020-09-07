#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>  
#include <string>
#include <opencv2/opencv_modules.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <ros/ros.h>  
using namespace cv;
 using namespace std;

void drawText(Mat & image, Point centerpoint)
{
    string text = to_string(centerpoint.x) + " " +to_string(centerpoint.y);
    putText(image, text,
            Point(centerpoint.x, centerpoint.y),
            FONT_HERSHEY_COMPLEX, 0.5, // font face and scale
            Scalar(0, 0, 255), // red (bgr) 
            1, LINE_AA); // line thickness and type
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ring_detection_node");
    ros::NodeHandle nh;//创建句柄
    ros::Rate loop_rate(30);
    
    Mat srcImage;
	// srcImage = imread("/home/linux/work/catkin_ws/src/ring_detection/src//test.png");  

    cv::VideoCapture cap1;
    cv::startWindowThread();
    cap1.open(1);
    while (ros::ok())
    {
        if(cap1.isOpened())
        {
            cap1 >> srcImage;  
        
            Mat midImage;
            
            cvtColor(srcImage, midImage, CV_BGR2GRAY);//转化边缘检测后的图为灰度图
            /*
            GaussianBlur(src,dst,ksize,sigmaX,sigmaY)
                src，输入图像
                dst，即目标图像
                ksize，高斯内核的大小。
                sigmaX，表示高斯核函数在X方向的的标准偏差。
                sigmaY，表示高斯核函数在Y方向的的标准偏差
            */
            GaussianBlur(midImage, midImage, Size(5, 5), 2, 2);
            imshow("midImage", midImage);
            vector<Vec3f> circles;
            HoughCircles(midImage, circles, CV_HOUGH_GRADIENT, 1, midImage.rows/20, 100, 100, 0, 0);
            //依次在图中绘制出圆
            for (size_t i = 0; i < circles.size(); i++)
            {
                Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                int radius = cvRound(circles[i][2]);
                //绘制圆心
                circle(srcImage, center, 3, Scalar(0, 255, 0), -1, 8, 0);
                //绘制圆轮廓
                circle(srcImage, center, radius, Scalar(155, 50, 255), 3, 8, 0);
                drawText(srcImage,center);
            }
        
            imshow("detectionImage", srcImage);
            waitKey(5);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
	return 0;
}
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../include/common.hpp"
#include "../src/preprocess.cpp"
#include "../src/mapping.cpp"
#include "../include/capture.hpp"

#include "../include/stop_watch.hpp"

#include "./recognition/tracking.cpp"
#include "../src/controlcenter.cpp"



using namespace std;
using namespace cv;

CaptureInterface captureInterface("/dev/video0");

int main(int argc, char *argv[])
{
    Tracking track;                     // 赛道识别
	Preprocess imagePreprocess;            // 图像预处理类
    ControlCenter controlCenterCal;          // 控制中心计算
    StopWatch watch;
    
    

    captureInterface.Start();

    watch.tic();
    while (1)
    {
        double frame_time = watch.toc();
        if(frame_time < 15)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            continue;
        }
        else
        {
            float detFPS = (float)1000.f / frame_time;
            std::cout << "run frame time: " << std::fixed << std::setprecision(3) << frame_time << "ms  ";
            std::cout << "FPS: " << (int)detFPS << std::endl;
            watch.tic();
        }

        Mat frame;
        frame = captureInterface.get_frame();

        Mat imgaeCorrect = frame;                                          // RGB
        Mat imageBinary = imagePreprocess.binaryzation(imgaeCorrect); // Gray
        track.trackRecognition(imageBinary); // 赛道线识别
        controlCenterCal.fitting(track);

        Mat remapImg = Mat::zeros(Size(COLSIMAGEIPM, ROWSIMAGEIPM), CV_8UC3); // 初始化图像
        POINT pointTemp(90, 160);
        // ipm.homography(frame, remapImg);
        //得到俯视域左、右边缘
        std::vector<POINT> perspectivePointsLeft;
        std::vector<POINT> perspectivePointsRight;
        std::vector<POINT> perspectivePointsCenter;
        Point2d point_perspective = ipm.homography(Point2d(pointTemp.y, pointTemp.x)); // 透视变换
        pointTemp = POINT(point_perspective.y, point_perspective.x);
        for(int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            Point2d point2d = ipm.homography(Point2d(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x)); // 透视变换
            perspectivePointsLeft.push_back(POINT(point2d.y, point2d.x));
        }
        for(int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            Point2d point2d = ipm.homography(Point2d(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x)); // 透视变换
            perspectivePointsRight.push_back(POINT(point2d.y, point2d.x));
        }
        for(int i = 0; i < controlCenterCal.centerEdge.size(); i++)
        {
            Point2d point2d = ipm.homography(Point2d(controlCenterCal.centerEdge[i].y, controlCenterCal.centerEdge[i].x)); // 透视变换
            perspectivePointsCenter.push_back(POINT(point2d.y, point2d.x));
        }

        //逆透视单边巡线
        perspectivePointsCenter = track.perspectiveMidFromRight(3);
        controlCenterCal.centerEdge = track.line_perspectiveInv(perspectivePointsCenter);

        // 绘制4象限分割线
        line(remapImg, Point(0, pointTemp.x), Point(remapImg.cols, pointTemp.x), Scalar(255, 255, 255), 1);
        line(remapImg, Point(pointTemp.y, 0), Point(pointTemp.y, remapImg.rows - 1), Scalar(255, 255, 255), 1);
        for (int i = 0; i < perspectivePointsLeft.size(); i++)
        {
            if(perspectivePointsLeft[i].x < 0 || perspectivePointsLeft[i].x > ROWSIMAGEIPM - 1 ||
                perspectivePointsLeft[i].y < 0 || perspectivePointsLeft[i].y > COLSIMAGEIPM - 1)
                continue;
            circle(remapImg, Point(perspectivePointsLeft[i].y, perspectivePointsLeft[i].x), 1,
                   Scalar(0, 255, 0), -1); // 绿色点
        }
        for (int i = 0; i < perspectivePointsRight.size(); i++)
        {
            if(perspectivePointsRight[i].x < 0 || perspectivePointsRight[i].x > ROWSIMAGEIPM - 1 ||
                perspectivePointsRight[i].y < 0 || perspectivePointsRight[i].y > COLSIMAGEIPM - 1)
                continue;
            circle(remapImg, Point(perspectivePointsRight[i].y, perspectivePointsRight[i].x), 1,
                   Scalar(0, 255, 255), -1); // 黄色点
        }

        //霍夫变换检测圆
         Mat gray;
         cvtColor(remapImg, gray, COLOR_BGR2GRAY);
         medianBlur(gray, gray, 5);

         vector<Vec3f> circles;
         HoughCircles(gray, circles, HOUGH_GRADIENT, 1, gray.rows / 8, 200, 20, 0, 1000);

         for (size_t i = 0; i < circles.size(); i++)
         {
             Vec3i c = circles[i];
             Point center = Point(c[0], c[1]);
             int radius = c[2];
             circle(remapImg, center, radius, Scalar(0, 0, 255), 3);
         }
        
        // 绘制田字格：基准线
        uint16_t rows = ROWSIMAGE / 30; // 16
        uint16_t cols = COLSIMAGE / 32; // 20

        for (size_t i = 1; i < rows; i++)
        {
            line(frame, Point(0, 30 * i), Point(frame.cols - 1, 30 * i), Scalar(211, 211, 211), 1);
        }
        for (size_t i = 1; i < cols; i++)
        {
            if (i == cols / 2)
                line(frame, Point(32 * i, 0), Point(32 * i, frame.rows - 1), Scalar(0, 0, 255), 1);
            else
                line(frame, Point(32 * i, 0), Point(32 * i, frame.rows - 1), Scalar(211, 211, 211), 1);
        }

        // 绘制中心点集edgeboard
        
        for (int i = 0; i < perspectivePointsCenter.size(); i++)
        {
            if(perspectivePointsCenter[i].x < 0 || perspectivePointsCenter[i].x > ROWSIMAGEIPM - 1 ||
                perspectivePointsCenter[i].y < 0 || perspectivePointsCenter[i].y > COLSIMAGEIPM - 1)
                continue;
            circle(remapImg, Point(perspectivePointsCenter[i].y, perspectivePointsCenter[i].x), 1, 
                    Scalar(0, 0, 255), -1);
        }

        track.drawImage(frame); // 图像显示赛道线识别结果
        for (int i = 0; i < controlCenterCal.centerEdge.size(); i++)
        {
            circle(frame, Point(controlCenterCal.centerEdge[i].y, controlCenterCal.centerEdge[i].x), 1, Scalar(0, 0, 255), -1);
        }

        imshow("frame", frame);
        imshow("remap", remapImg);
        
        waitKey(2);
    }
    captureInterface.Stop();
    return 0;
}

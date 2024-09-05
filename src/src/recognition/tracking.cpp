#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习,请勿商用).
 *            The code ADAPTS the corresponding hardware circuit board(代码适配百度Edgeboard-智能汽车赛事版),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file tracking.cpp
 * @author your name (you@domain.com)
 * @brief 赛道线识别：提取赛道左右边缘数据（包括岔路信息等）
 * @version 0.1
 * @date 2022-02-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"

using namespace cv;
using namespace std;

class Tracking
{
public:
    vector<POINT> pointsEdgeLeft;     // 赛道左边缘点集
    vector<POINT> pointsEdgeRight;    // 赛道右边缘点集
    vector<POINT> widthBlock;         // 色块宽度=终-起（每行）
    vector<POINT> spurroad;           // 保存岔路信息
    double stdevLeft;                 // 边缘斜率方差（左）
    double stdevRight;                // 边缘斜率方差（右）
    int validRowsLeft = 0;            // 边缘有效行数（左）
    int validRowsRight = 0;           // 边缘有效行数（右）
    POINT garageEnable = POINT(0, 0); // 车库识别标志：（x=1/0，y=row)
    uint16_t rowCutUp = 10;           // 图像顶部切行
    uint16_t rowCutBottom = 10;       // 图像底部切行
    int RingStatus = 0;               // 环岛状态：0-无环岛，1-左环岛,2-右环岛
    /**
     * @brief 赛道线识别
     *
     * @param isResearch 是否重复搜索
     * @param rowStart 边缘搜索起始行
     */
    void trackRecognition(bool isResearch, uint16_t rowStart)
    {
        bool flagStartBlock = true;                    // 搜索到色块起始行的标志（行）
        int counterSearchRows = pointsEdgeLeft.size(); // 搜索行计数
        int startBlock[30];                            // 色块起点（行）
        int endBlock[30];                              // 色块终点（行）
        int counterBlock = 0;                          // 色块计数器（行）
        POINT pointSpurroad;                           // 岔路坐标
        int counterSpurroad = 0;                       // 岔路识别标志
        bool spurroadEnable = false;

        if (rowCutUp > ROWSIMAGE / 4)
            rowCutUp = ROWSIMAGE / 4;
        if (rowCutBottom > ROWSIMAGE / 4)
            rowCutBottom = ROWSIMAGE / 4;

        if (!isResearch)
        {
            pointsEdgeLeft.clear();              // 初始化边缘结果
            pointsEdgeRight.clear();             // 初始化边缘结果
            widthBlock.clear();                  // 初始化色块数据
            spurroad.clear();                    // 岔路信息
            validRowsLeft = 0;                   // 边缘有效行数（左）
            validRowsRight = 0;                  // 边缘有效行数（右）
            flagStartBlock = true;               // 搜索到色块起始行的标志（行）
            garageEnable = POINT(0, 0);          // 车库识别标志初始化
            rowStart = ROWSIMAGE - rowCutBottom; // 默认底部起始行
        }
        else
        {
            if (pointsEdgeLeft.size() > rowStart)
                pointsEdgeLeft.resize(rowStart);
            if (pointsEdgeRight.size() > rowStart)
                pointsEdgeRight.resize(rowStart);
            if (widthBlock.size() > rowStart)
            {
                widthBlock.resize(rowStart);
                if (rowStart > 1)
                    rowStart = widthBlock[rowStart - 1].x - 2;
            }

            flagStartBlock = false; // 搜索到色块起始行的标志（行）
        }

        //  开始识别赛道左右边缘
        for (int row = rowStart; row > rowCutUp; row--) // 有效行：10~220
        {
            counterBlock = 0; // 色块计数器清空
            // 搜索色（block）块信息
            if (imageType == ImageType::Rgb) // 输入RGB图像
            {
                if (imagePath.at<Vec3b>(row, 1)[2] > 0)
                {
                    startBlock[counterBlock] = 0;
                }
                for (int col = 1; col < COLSIMAGE; col++) // 搜索出每行的所有色块
                {
                    if (imagePath.at<Vec3b>(row, col)[2] > 0 &&
                        imagePath.at<Vec3b>(row, col - 1)[2] == 0)
                    {
                        startBlock[counterBlock] = col;
                    }
                    else
                    {
                        if (imagePath.at<Vec3b>(row, col)[2] == 0 &&
                            imagePath.at<Vec3b>(row, col - 1)[2] > 0)
                        {
                            endBlock[counterBlock++] = col;
                            if (counterBlock >= end(endBlock) - begin(endBlock))
                                break;
                        }
                    }
                }
                if (imagePath.at<Vec3b>(row, COLSIMAGE - 1)[2] > 0)
                {
                    if (counterBlock < end(endBlock) - begin(endBlock) - 1)
                        endBlock[counterBlock++] = COLSIMAGE - 1;
                }
            }
            if (imageType == ImageType::Binary) // 输入二值化图像
            {
                if (imagePath.at<uchar>(row, 1) > 127)
                {
                    startBlock[counterBlock] = 0;
                }
                for (int col = 1; col < COLSIMAGE; col++) // 搜索出每行的所有色块
                {
                    if (imagePath.at<uchar>(row, col) > 127 &&
                        imagePath.at<uchar>(row, col - 1) <= 127)
                    {
                        startBlock[counterBlock] = col;
                    }
                    else
                    {
                        if (imagePath.at<uchar>(row, col) <= 127 &&
                            imagePath.at<uchar>(row, col - 1) > 127)
                        {
                            endBlock[counterBlock++] = col;
                            if (counterBlock >= end(endBlock) - begin(endBlock))
                                break;
                        }
                    }
                }
                if (imagePath.at<uchar>(row, COLSIMAGE - 1) > 127)
                {
                    if (counterBlock < end(endBlock) - begin(endBlock) - 1)
                        endBlock[counterBlock++] = COLSIMAGE - 1;
                }
            }

            int widthBlocks = endBlock[0] - startBlock[0]; // 色块宽度临时变量
            int indexWidestBlock = 0;                      // 最宽色块的序号
            if (flagStartBlock)                            // 起始行做特殊处理
            {
                if (row < ROWSIMAGE / 3)
                    return;
                if (counterBlock == 0)
                {
                    continue;
                }
                for (int i = 1; i < counterBlock; i++) // 搜索最宽色块
                {
                    int tmp_width = endBlock[i] - startBlock[i];
                    if (tmp_width > widthBlocks)
                    {
                        widthBlocks = tmp_width;
                        indexWidestBlock = i;
                    }
                }

                int limitWidthBlock = COLSIMAGE * 0.8; // 首行色块宽度限制（不能太小）
                if (row < ROWSIMAGE * 0.6)
                {
                    limitWidthBlock = COLSIMAGE * 0.4;
                }
                if (widthBlocks > limitWidthBlock) // 满足首行宽度要求
                {
                    flagStartBlock = false;
                    POINT pointTmp(row, startBlock[indexWidestBlock]);
                    pointsEdgeLeft.push_back(pointTmp);
                    pointTmp.y = endBlock[indexWidestBlock];
                    pointsEdgeRight.push_back(pointTmp);
                    widthBlock.emplace_back(row, endBlock[indexWidestBlock] - startBlock[indexWidestBlock]);
                    counterSearchRows++;
                }
                spurroadEnable = false;
            }
            else // 其它行色块坐标处理
            {
                if (counterBlock == 0)
                {
                    break;
                }

                //-------------------------------------------------<车库标识识别>-------------------------------------------------------------
                if (counterBlock > 5 && !garageEnable.x)
                {
                    int widthThis = 0;        // 色块的宽度
                    int widthVer = 0;         // 当前行色块的平均值
                    vector<int> widthGarage;  // 当前行色块宽度集合
                    vector<int> centerGarage; // 当前行色块质心集合
                    vector<int> indexGarage;  // 当前行有效色块的序号

                    for (int i = 0; i < counterBlock; i++)
                    {
                        widthThis = endBlock[i] - startBlock[i];        // 色块的宽度
                        int center = (endBlock[i] + startBlock[i]) / 2; // 色块的质心
                        if (widthThis > 5 && widthThis < 50)            // 过滤无效色块区域：噪点
                        {
                            centerGarage.push_back(center);
                            widthGarage.push_back(widthThis);
                        }
                    }

                    int widthMiddle = getMiddleValue(widthGarage); // 斑马线色块宽度中值

                    for (int i = 0; i < widthGarage.size(); i++)
                    {
                        if (abs(widthGarage[i] - widthMiddle) < widthMiddle / 3)
                        {
                            indexGarage.push_back(i);
                        }
                    }
                    if (indexGarage.size() >= 4) // 验证有效斑马线色块个数
                    {
                        vector<int> distance;
                        for (int i = 1; i < indexGarage.size(); i++) // 质心间距的方差校验
                        {
                            distance.push_back(widthGarage[indexGarage[i]] - widthGarage[indexGarage[i - 1]]);
                        }
                        double var = sigma(distance);
                        if (var < 5.0) // 经验参数
                        {
                            garageEnable.x = 1;                      // 车库标志使能
                            garageEnable.y = pointsEdgeRight.size(); // 斑马线行序号
                        }
                    }
                }
                //------------------------------------------------------------------------------------------------------------------------

                vector<int> indexBlocks;               // 色块序号（行）
                for (int i = 0; i < counterBlock; i++) // 上下行色块的连通性判断
                {
                    int g_cover = min(endBlock[i], pointsEdgeRight[pointsEdgeRight.size() - 1].y) -
                                  max(startBlock[i], pointsEdgeLeft[pointsEdgeLeft.size() - 1].y);
                    if (g_cover >= 0)
                    {
                        indexBlocks.push_back(i);
                    }
                }

                if (indexBlocks.size() == 0) // 如果没有发现联通色块，则图像搜索完成，结束任务
                {
                    break;
                }
                else if (indexBlocks.size() == 1) // 只存在单个色块，正常情况，提取边缘信息
                {
                    if (endBlock[indexBlocks[0]] - startBlock[indexBlocks[0]] < COLSIMAGE / 10)
                    {
                        continue;
                    }
                    pointsEdgeLeft.emplace_back(row, startBlock[indexBlocks[0]]);
                    pointsEdgeRight.emplace_back(row, endBlock[indexBlocks[0]]);
                    slopeCal(pointsEdgeLeft, pointsEdgeLeft.size() - 1); // 边缘斜率计算
                    slopeCal(pointsEdgeRight, pointsEdgeRight.size() - 1);
                    widthBlock.emplace_back(row, endBlock[indexBlocks[0]] - startBlock[indexBlocks[0]]);
                    spurroadEnable = false;
                }
                else if (indexBlocks.size() > 1) // 存在多个色块，则需要择优处理：选取与上一行最近的色块
                {
                    int centerLast = COLSIMAGE / 2;
                    if (pointsEdgeRight.size() > 0 && pointsEdgeLeft.size() > 0)
                        centerLast = (pointsEdgeRight[pointsEdgeRight.size() - 1].y + pointsEdgeLeft[pointsEdgeLeft.size() - 1].y) / 2; // 上一行色块的中心点横坐标
                    int centerThis = (startBlock[indexBlocks[0]] + endBlock[indexBlocks[0]]) / 2;                                       // 当前行色块的中心点横坐标
                    int differBlocks = abs(centerThis - centerLast);                                                                    // 上下行色块的中心距离
                    int indexGoalBlock = 0;                                                                                             // 目标色块的编号
                    int startBlockNear = startBlock[indexBlocks[0]];                                                                    // 搜索与上一行最近的色块起点
                    int endBlockNear = endBlock[indexBlocks[0]];                                                                        // 搜索与上一行最近的色块终点

                    for (int i = 1; i < indexBlocks.size(); i++) // 搜索与上一行最近的色块编号
                    {
                        centerThis = (startBlock[indexBlocks[i]] + endBlock[indexBlocks[i]]) / 2;
                        if (abs(centerThis - centerLast) < differBlocks)
                        {
                            differBlocks = abs(centerThis - centerLast);
                            indexGoalBlock = i;
                        }
                        // 搜索与上一行最近的边缘起点和终点
                        if (abs(pointsEdgeLeft[pointsEdgeLeft.size() - 1].y - startBlock[indexBlocks[i]]) <
                            abs(pointsEdgeLeft[pointsEdgeLeft.size() - 1].y - startBlockNear))
                        {
                            startBlockNear = startBlock[indexBlocks[i]];
                        }
                        if (abs(pointsEdgeRight[pointsEdgeRight.size() - 1].y - endBlock[indexBlocks[i]]) <
                            abs(pointsEdgeRight[pointsEdgeRight.size() - 1].y - endBlockNear))
                        {
                            endBlockNear = endBlock[indexBlocks[i]];
                        }
                    }

                    // 检索最佳的起点与终点
                    if (abs(pointsEdgeLeft[pointsEdgeLeft.size() - 1].y - startBlock[indexBlocks[indexGoalBlock]]) <
                        abs(pointsEdgeLeft[pointsEdgeLeft.size() - 1].y - startBlockNear))
                    {
                        startBlockNear = startBlock[indexBlocks[indexGoalBlock]];
                    }
                    if (abs(pointsEdgeRight[pointsEdgeRight.size() - 1].y - endBlock[indexBlocks[indexGoalBlock]]) <
                        abs(pointsEdgeRight[pointsEdgeRight.size() - 1].y - endBlockNear))
                    {
                        endBlockNear = endBlock[indexBlocks[indexGoalBlock]];
                    }

                    if (endBlockNear - startBlockNear < COLSIMAGE / 10)
                    {
                        continue;
                    }
                    POINT tmp_point(row, startBlockNear);
                    pointsEdgeLeft.push_back(tmp_point);
                    tmp_point.y = endBlockNear;
                    pointsEdgeRight.push_back(tmp_point);
                    widthBlock.emplace_back(row, endBlockNear - startBlockNear);
                    slopeCal(pointsEdgeLeft, pointsEdgeLeft.size() - 1);
                    slopeCal(pointsEdgeRight, pointsEdgeRight.size() - 1);
                    counterSearchRows++;

                    //-------------------------------<岔路信息提取>----------------------------------------
                    pointSpurroad.x = row;
                    pointSpurroad.y = endBlock[indexBlocks[0]];
                    if (!spurroadEnable)
                    {
                        spurroad.push_back(pointSpurroad);
                        spurroadEnable = true;
                    }
                    //------------------------------------------------------------------------------------
                }

                stdevLeft = stdevEdgeCal(pointsEdgeLeft, ROWSIMAGE); // 计算边缘方差
                stdevRight = stdevEdgeCal(pointsEdgeRight, ROWSIMAGE);

                validRowsCal(); // 有效行计算
            }
        }
    }

    /**
     * @brief 赛道线识别
     *
     * @param imageBinary 赛道识别基准图像
     */
    void trackRecognition(Mat &imageBinary)
    {
        imagePath = imageBinary;
        trackRecognition(false, 0);
    }

    /**
     * @brief 显示赛道线识别结果
     *
     * @param trackImage 需要叠加显示的图像
     */
    void drawImage(Mat &trackImage)
    {
        for (int i = 0; i < pointsEdgeLeft.size(); i++)
        {
            circle(trackImage, Point(pointsEdgeLeft[i].y, pointsEdgeLeft[i].x), 1,
                   Scalar(0, 255, 0), -1); // 绿色点
        }
        for (int i = 0; i < pointsEdgeRight.size(); i++)
        {
            circle(trackImage, Point(pointsEdgeRight[i].y, pointsEdgeRight[i].x), 1,
                   Scalar(0, 255, 255), -1); // 黄色点
        }

        for (int i = 0; i < spurroad.size(); i++)
        {
            circle(trackImage, Point(spurroad[i].y, spurroad[i].x), 3,
                   Scalar(0, 0, 255), -1); // 红色点
        }

        putText(trackImage, to_string(validRowsRight) + " " + to_string(stdevRight), Point(COLSIMAGE - 100, ROWSIMAGE - 50),
                FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 1, CV_AA);
        putText(trackImage, to_string(validRowsLeft) + " " + to_string(stdevLeft), Point(20, ROWSIMAGE - 50),
                FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 1, CV_AA);
    }

    /**
     * @brief 边缘斜率计算
     *
     * @param v_edge
     * @param img_height
     * @return double
     */
    double stdevEdgeCal(vector<POINT> &v_edge, int img_height)
    {
        if (v_edge.size() < img_height / 4)
        {
            return 1000;
        }
        vector<int> v_slope;
        int step = 10; // v_edge.size()/10;
        for (int i = step; i < v_edge.size(); i += step)
        {
            if (v_edge[i].x - v_edge[i - step].x)
                v_slope.push_back((v_edge[i].y - v_edge[i - step].y) * 100 / (v_edge[i].x - v_edge[i - step].x));
        }
        if (v_slope.size() > 1)
        {
            double sum = accumulate(begin(v_slope), end(v_slope), 0.0);
            double mean = sum / v_slope.size(); // 均值
            double accum = 0.0;
            for_each(begin(v_slope), end(v_slope), [&](const double d)
                     { accum += (d - mean) * (d - mean); });

            return sqrt(accum / (v_slope.size() - 1)); // 方差
        }
        else
            return 0;
    }
   


    /**
     * @brief 最小二乘法 一元线性回归
     *
     * @param line 边缘点集
     * @param k    斜率，注意此处的坐标系与习惯不同
     * @param b
     */
    void LeastSquare(vector<POINT> line, double &k, double &b)
    {
        if(line.size() < 5)
        {
            k = 0;
            b = 0;
            return;
        }
        double t1 = 0, t2 = 0, t3 = 0, t4 = 0;
        uint16_t num = line.size();
        for (int i = 0; i < num; i++)
        {
            t1 += line[i].x * line[i].x;
            t2 += line[i].x;
            t3 += line[i].y * line[i].x;
            t4 += line[i].y;
        }
        k = (t3 * num - t2 * t4) / (t1 * num - t2 * t2);
        b = (t1 * t4 - t2 * t3) / (t1 * num - t2 * t2);

        return;
    }
    double LeastSquare(vector<POINT> line)
    {
        double k = 0, b = 0;
        LeastSquare(line, k, b);
        return k;
    }

    // 计算目标段的最小二乘斜率
    double LeastSquare(vector<POINT> line, uint16_t start, uint16_t end)
    {
        double t1 = 0, t2 = 0, t3 = 0, t4 = 0;
        uint16_t num = line.size();
        if (start < 0 || start > num - 1 || end < 0 || end > num)
        {
            //td::cout << "LeastSqure err !" << std::endl;
            return 0;
        }

        for (int i = start; i < end; i++)
        {
            t1 += line[i].x * line[i].x;
            t2 += line[i].x;
            t3 += line[i].y * line[i].x;
            t4 += line[i].y;
        }
        double k = (t3 * num - t2 * t4) / (t1 * num - t2 * t2);
        // b = (t1 * t4 - t2 * t3) / (t1 * num - t2 * t2);

        return k;
    }

    // 计算目标点处切线的斜率
    double LeastSquare(vector<POINT> line, uint16_t index)
    {
        double t1 = 0, t2 = 0, t3 = 0, t4 = 0;
        uint16_t num = line.size();
        if (num < 5)
        {
            return 0;
        }
        int start = index - 2;
        int end = index + 2;
        if (start < 0)
            start = 0;
        if (end > num)
            end = num;

        for (int i = start; i < end; i++)
        {
            t1 += line[i].x * line[i].x;
            t2 += line[i].x;
            t3 += line[i].y * line[i].x;
            t4 += line[i].y;
        }
        double k = (t3 * num - t2 * t4) / (t1 * num - t2 * t2);
        // b = (t1 * t4 - t2 * t3) / (t1 * num - t2 * t2);

        return k;
    }

    /**
     * @brief 得到俯视域下的中线
     * @param edge 摄像机坐标系下的点集
     * 
     * @return 俯视域的点集
     */
    std::vector<POINT> line_perspective(std::vector<POINT> pointsEdgeline)
    {
        std::vector<POINT> perspectivePoints;
        for (int i = 0; i < pointsEdgeline.size(); i++)
        {
            if(pointsEdgeline[i].y == 0 || pointsEdgeline[i].y == COLSIMAGE - 1)
                continue;
                
            cv::Point2d point2d = ipm.homography(Point2d(pointsEdgeline[i].y, pointsEdgeline[i].x)); // 透视变换
            perspectivePoints.push_back(POINT(point2d.y, point2d.x));
        }

        return perspectivePoints;
    }

    /**
     * @brief 摄像机坐标系下的中线
     * @param edge 俯视域下的点集
     * 
     * @return 摄像机坐标系下的点集
     */
    std::vector<POINT> line_perspectiveInv(std::vector<POINT> perspectivePoints)
    {
        std::vector<POINT> pointsEdgeline;
        for (int i = 0; i < perspectivePoints.size(); i++)
        {
            cv::Point2d point2d = ipm.homographyInv(Point2d(perspectivePoints[i].y, perspectivePoints[i].x)); // 透视变换
            POINT edgePoint = POINT(point2d.y, point2d.x);
            // if (edgePoint.x >= ROWSIMAGE)
            //     edgePoint.x = ROWSIMAGE - 1;

            // else if (edgePoint.x < 0)
            //     edgePoint.x = 0;
            if(edgePoint.x >= ROWSIMAGE || edgePoint.x < 0)
                continue;
            else if (edgePoint.y >= COLSIMAGE)
                edgePoint.y = COLSIMAGE - 1;
            else if (edgePoint.y < 0)
                edgePoint.y = 0;

            pointsEdgeline.push_back(edgePoint);
        }

        return pointsEdgeline;
    }

	/**
	 * @brief 在俯视域由左边缘预测右边缘
	 *
	 * @param pointsEdgeLeft
	 * @return vector<POINT>
	 */
	vector<POINT> predictEdgeRight(vector<POINT> pointsEdgeLeft, bool bezier = true, int offset = 120)
	{
		// int offset = 120; // 右边缘平移尺度
		vector<POINT> pointsEdgeRight;
		if (pointsEdgeLeft.size() < 3)
			return pointsEdgeRight;

        if(!bezier)
        {
            for(int i = 0; i < pointsEdgeLeft.size(); i++)
            {
                Point2d edgeIpm = ipm.homography(
                    Point2d(pointsEdgeLeft[i].y, pointsEdgeLeft[i].x)); // 透视变换
                Point2d prefictRight = Point2d(edgeIpm.x + offset, edgeIpm.y);
                Point2d edgeIipm = ipm.homographyInv(prefictRight); // 反透视变换
                POINT edgePoint = POINT(edgeIipm.y, edgeIipm.x);
                if (edgePoint.x >= ROWSIMAGE)
                    edgePoint.x = ROWSIMAGE - 1;

                else if (edgePoint.x < 0)
                    edgePoint.x = 0;

                else if (edgePoint.y >= COLSIMAGE)
                    edgePoint.y = COLSIMAGE - 1;
                else if (edgePoint.y < 0)
                    edgePoint.y = 0;

                pointsEdgeRight.push_back(edgePoint);
            }
            return pointsEdgeRight;
        }

		// Start
		Point2d startIpm = ipm.homography(
			Point2d(pointsEdgeLeft[0].y, pointsEdgeLeft[0].x)); // 透视变换
		Point2d prefictRight = Point2d(startIpm.x + offset, startIpm.y);
		Point2d startIipm = ipm.homographyInv(prefictRight); // 反透视变换
		POINT startPoint = POINT(startIipm.y, startIipm.x);

		// Middle
		Point2d middleIpm = ipm.homography(
			Point2d(pointsEdgeLeft[pointsEdgeLeft.size() / 2].y,
					pointsEdgeLeft[pointsEdgeLeft.size() / 2].x)); // 透视变换
		prefictRight = Point2d(middleIpm.x + offset, middleIpm.y);
		Point2d middleIipm = ipm.homographyInv(prefictRight); // 反透视变换
		POINT midPoint = POINT(middleIipm.y, middleIipm.x);	  // 补线中点

		// End
		Point2d endIpm = ipm.homography(
			Point2d(pointsEdgeLeft[pointsEdgeLeft.size() - 1].y,
					pointsEdgeLeft[pointsEdgeLeft.size() - 1].x)); // 透视变换
		prefictRight = Point2d(endIpm.x + offset, endIpm.y);
		Point2d endtIipm = ipm.homographyInv(prefictRight); // 反透视变换
		POINT endPoint = POINT(endtIipm.y, endtIipm.x);

		// 补线
		vector<POINT> input = {startPoint, midPoint, endPoint};
		vector<POINT> repair = Bezier(0.05, input);

		for (int i = 0; i < repair.size(); i++)
		{
			if (repair[i].x >= ROWSIMAGE)
				repair[i].x = ROWSIMAGE - 1;

			else if (repair[i].x < 0)
				repair[i].x = 0;

			else if (repair[i].y >= COLSIMAGE)
				repair[i].y = COLSIMAGE - 1;
			else if (repair[i].y < 0)
				repair[i].y = 0;

			pointsEdgeRight.push_back(repair[i]);
		}

		return pointsEdgeRight;
	}

	/**
	 * @brief 在俯视域由右边缘预测左边缘
	 *
	 * @param pointsEdgeRight
	 * @return vector<POINT>
	 */
	vector<POINT> predictEdgeLeft(vector<POINT> pointsEdgeRight, bool bezier = true, int offset = 120)
	{
		// int offset = 120; // 右边缘平移尺度
		vector<POINT> pointsEdgeLeft;
		if (pointsEdgeRight.size() < 3)
			return pointsEdgeLeft;

        if(!bezier)
        {
            for(int i = 0; i < pointsEdgeRight.size(); i++)
            {
                Point2d edgeIpm = ipm.homography(
                    Point2d(pointsEdgeRight[i].y, pointsEdgeRight[i].x)); // 透视变换
                Point2d prefictRight = Point2d(edgeIpm.x - offset, edgeIpm.y);
                Point2d edgeIipm = ipm.homographyInv(prefictRight); // 反透视变换
                POINT edgePoint = POINT(edgeIipm.y, edgeIipm.x);
                if (edgePoint.x >= ROWSIMAGE)
                    edgePoint.x = ROWSIMAGE - 1;

                else if (edgePoint.x < 0)
                    edgePoint.x = 0;

                else if (edgePoint.y >= COLSIMAGE)
                    edgePoint.y = COLSIMAGE - 1;
                else if (edgePoint.y < 0)
                    edgePoint.y = 0;

                pointsEdgeLeft.push_back(edgePoint);
            }
            return pointsEdgeLeft;
        }

		// Start
		Point2d startIpm = ipm.homography(
			Point2d(pointsEdgeRight[0].y, pointsEdgeRight[0].x)); // 透视变换
		Point2d prefictLeft = Point2d(startIpm.x - offset, startIpm.y);
		Point2d startIipm = ipm.homographyInv(prefictLeft); // 反透视变换
		POINT startPoint = POINT(startIipm.y, startIipm.x);

		// Middle
		Point2d middleIpm = ipm.homography(
			Point2d(pointsEdgeRight[pointsEdgeRight.size() / 2].y,
					pointsEdgeRight[pointsEdgeRight.size() / 2].x)); // 透视变换
		prefictLeft = Point2d(middleIpm.x - offset, middleIpm.y);
		Point2d middleIipm = ipm.homographyInv(prefictLeft); // 反透视变换
		POINT midPoint = POINT(middleIipm.y, middleIipm.x);	 // 补线中点

		// End
		Point2d endIpm = ipm.homography(
			Point2d(pointsEdgeRight[pointsEdgeRight.size() - 1].y,
					pointsEdgeRight[pointsEdgeRight.size() - 1].x)); // 透视变换
		prefictLeft = Point2d(endIpm.x - offset, endIpm.y);
		Point2d endtIipm = ipm.homographyInv(prefictLeft); // 反透视变换
		POINT endPoint = POINT(endtIipm.y, endtIipm.x);

		// 补线

		vector<POINT> input = {startPoint, midPoint, endPoint};
		vector<POINT> repair = Bezier(0.05, input);

		for (int i = 0; i < repair.size(); i++)
		{
			if (repair[i].x >= ROWSIMAGE)
				repair[i].x = ROWSIMAGE - 1;

			else if (repair[i].x < 0)
				repair[i].x = 0;

			else if (repair[i].y >= COLSIMAGE)
				repair[i].y = COLSIMAGE - 1;
			else if (repair[i].y < 0)
				repair[i].y = 0;

			pointsEdgeLeft.push_back(repair[i]);
		}

		return pointsEdgeLeft;
	}

	/**
	 * @brief 在俯视域由左边缘预测中线
	 *
	 * @param size 计算切点曲率时开窗大小
	 * @return vector<POINT>
	 */
    std::vector<POINT> perspectiveMidFromLeft(int size)
    {
        int offset = 60;
        std::vector<POINT> centerEdge;
        std::vector<POINT> perspectiveLeft = line_perspective(this->pointsEdgeLeft);
        for(int i = size; i < perspectiveLeft.size() - size; i++)
        {
            // 计算 dx
            float dx = 0.0f;
            if (i + size < perspectiveLeft.size() && i - size >= 0) {
                dx = perspectiveLeft[i + size].x - perspectiveLeft[i - size].x;
            }

            // 计算 dy
            float dy = 0.0f;
            if (i + size < perspectiveLeft.size() && i - size >= 0) {
                dy = perspectiveLeft[i + size].y - perspectiveLeft[i - size].y;
            }
            float dn = std::sqrt(dx * dx + dy * dy);
            dx /= dn;
            dy /= dn;

            centerEdge.push_back(POINT(perspectiveLeft[i].x + offset * dy, perspectiveLeft[i].y - offset * dx));
        }

        return centerEdge;
    }

	/**
	 * @brief 在俯视域由右边缘预测中线
	 *
	 * @param size 计算切点曲率时开窗大小
	 * @return vector<POINT>
	 */
    std::vector<POINT> perspectiveMidFromRight(int size)
    {
        int offset = 60;
        std::vector<POINT> centerEdge;
        std::vector<POINT> perspectiveRight = line_perspective(this->pointsEdgeRight);
        for(int i = size; i < perspectiveRight.size() - size; i++)
        {
           // 计算 dx
            float dx = 0.0f;
            if (i + size < perspectiveRight.size() && i - size >= 0) {
                dx = perspectiveRight[i + size].x - perspectiveRight[i - size].x;
            }

            // 计算 dy
            float dy = 0.0f;
            if (i + size < perspectiveRight.size() && i - size >= 0) {
                dy = perspectiveRight[i + size].y - perspectiveRight[i - size].y;
            }
            float dn = std::sqrt(dx * dx + dy * dy);
            dx /= dn;
            dy /= dn;

            centerEdge.push_back(POINT(perspectiveRight[i].x - offset * dy, perspectiveRight[i].y + offset * dx));
        }

        return centerEdge;
    }







private:
    Mat imagePath; // 赛道搜索图像
    /**
     * @brief 赛道识别输入图像类型
     *
     */
    enum ImageType
    {
        Binary = 0, // 二值化
        Rgb,        // RGB
    };

    ImageType imageType = ImageType::Binary; // 赛道识别输入图像类型：二值化图像
    /**
     * @brief 边缘斜率计算
     *
     * @param edge
     * @param index
     */
    void slopeCal(vector<POINT> &edge, int index)
    {
        if (index <= 4)
        {
            return;
        }
        float temp_slop1 = 0.0, temp_slop2 = 0.0;
        if (edge[index].x - edge[index - 2].x != 0)
        {
            temp_slop1 = (float)(edge[index].y - edge[index - 2].y) * 1.0f /
                         ((edge[index].x - edge[index - 2].x) * 1.0f);
        }
        else
        {
            temp_slop1 = edge[index].y > edge[index - 2].y ? 255 : -255;
        }
        if (edge[index].x - edge[index - 4].x != 0)
        {
            temp_slop2 = (float)(edge[index].y - edge[index - 4].y) * 1.0f /
                         ((edge[index].x - edge[index - 4].x) * 1.0f);
        }
        else
        {
            edge[index].slope = edge[index].y > edge[index - 4].y ? 255 : -255;
        }
        if (abs(temp_slop1) != 255 && abs(temp_slop2) != 255)
        {
            edge[index].slope = (temp_slop1 + temp_slop2) * 1.0 / 2;
        }
        else if (abs(temp_slop1) != 255)
        {
            edge[index].slope = temp_slop1;
        }
        else
        {
            edge[index].slope = temp_slop2;
        }
    }

    /**
     * @brief 边缘有效行计算：左/右
     *
     */
    void validRowsCal(void)
    {
        // 左边有效行
        validRowsLeft = 0;
        if (pointsEdgeLeft.size() > 1)
        {
            for (int i = pointsEdgeLeft.size() - 1; i >= 1; i--)
            {
                if (pointsEdgeLeft[i].y > 2 && pointsEdgeLeft[i - 1].y >= 2)
                {
                    validRowsLeft = i + 1;
                    break;
                }
                if (pointsEdgeLeft[i].y < 2 && pointsEdgeLeft[i - 1].y >= 2)
                {
                    validRowsLeft = i + 1;
                    break;
                }
            }
        }

        // 右边有效行
        validRowsRight = 0;
        if (pointsEdgeRight.size() > 1)
        {
            for (int i = pointsEdgeRight.size() - 1; i >= 1; i--)
            {
                if (pointsEdgeRight[i].y <= COLSIMAGE - 2 && pointsEdgeRight[i - 1].y <= COLSIMAGE - 2)
                {
                    validRowsRight = i + 1;
                    break;
                }
                if (pointsEdgeRight[i].y >= COLSIMAGE - 2 && pointsEdgeRight[i - 1].y < COLSIMAGE - 2)
                {
                    validRowsRight = i + 1;
                    break;
                }
            }
        }
    }

    /**
     * @brief 冒泡法求取集合中值
     *
     * @param vec 输入集合
     * @return int 中值
     */
    int getMiddleValue(vector<int> vec)
    {
        if (vec.size() < 1)
            return -1;
        if (vec.size() == 1)
            return vec[0];

        int len = vec.size();
        while (len > 0)
        {
            bool sort = true; // 是否进行排序操作标志
            for (int i = 0; i < len - 1; ++i)
            {
                if (vec[i] > vec[i + 1])
                {
                    swap(vec[i], vec[i + 1]);
                    sort = false;
                }
            }
            if (sort) // 排序完成
                break;

            --len;
        }

        return vec[(int)vec.size() / 2];
    }
};

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
 * @file controlcenter.cpp
 * @author Leo
 * @brief 智能车控制中心计算
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
#include "../include/common.hpp"
#include "recognition/tracking.cpp"
using namespace cv;
using namespace std;

class ControlCenter
{
public:
    int controlCenter;
    vector<POINT> centerEdge;
    vector<POINT> centerEdge_yh;
    POINT intersectionLeft;
    POINT intersectionRight;
    uint16_t validRowsLeft = 0;
    uint16_t validRowsRight = 0;
    double sigmaCenter = 0;
    string style = "";
     /**
     * @brief 车辆冲出赛道检测（保护车辆）
     *
     * @param track
     * @return true
     * @return false
     */
    bool derailmentCheck(Tracking track)
    {
        if (track.pointsEdgeLeft.size() < 30 && track.pointsEdgeRight.size() < 30) // 防止车辆冲出赛道
        {
            countOutlineA++;
            countOutlineB = 0;
            if (countOutlineA > 20)
                return true;
        }
        else
        {
            countOutlineB++;
            if (countOutlineB > 50)
            {
                countOutlineA = 0;
                countOutlineB = 50;
            }
        }
        return false;
    }
    /**
     * @brief 控制中心计算
     * @param pointsEdgeLeft 赛道左边缘点集
     * @param pointsEdgeRight 赛道右边缘点集
     */
    void fitting(Tracking &track)
    {
        // 未裁线操作，点数小于一定数量，退出函数
        if(track.pointsEdgeLeft.size() <= 3 || track.pointsEdgeRight.size() <= 3)
            return;
        
        sigmaCenter = 0;
        controlCenter = COLSIMAGE / 2;
        centerEdge.clear();
        vector<POINT> v_center(4);
        style = "STRIGHT";

        double miu = ROWSIMAGE / 2;
        double singema = miu / 3 *2;

        // 边缘斜率标准差 重计算（边缘修正之后）
        track.stdevLeft = track.stdevEdgeCal(track.pointsEdgeLeft, ROWSIMAGE);
        track.stdevRight = track.stdevEdgeCal(track.pointsEdgeRight, ROWSIMAGE);
        std::cout<<"centerEdge.size()="<<std::endl;
        //std::cout<<"stdevLeft="<<track.stdevLeft<<" stdevRight="<<track.stdevRight<<std::endl;
        //std::cout<<"stdcvRight="<<track.stdevRight<<" stdcvLeft="<<track.stdevLeft<<std::endl;
        // 边线交点搜寻
        intersectionLeft = searchLeftIntersection(track.pointsEdgeLeft);
        intersectionRight = searchRightIntersection(track.pointsEdgeRight);

        // 边缘有效行优化，左边方差大右边方差小；或者左边小右边大，就是转弯。将边缘没用的边线优化
        if ((track.stdevLeft < 60 && track.stdevRight > 60) || (track.stdevLeft > 60 && track.stdevRight < 60))
        {
            validRowsCal(track.pointsEdgeLeft, track.pointsEdgeRight); // 边缘有效行计算
            track.pointsEdgeLeft.resize(validRowsLeft);
            track.pointsEdgeRight.resize(validRowsRight);
        }
        else if(track.pointsEdgeLeft.size() < 185 && track.pointsEdgeRight.size() < 185 && 
                (track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].y > COLSIMAGE / 2 + 30 || track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].y < COLSIMAGE / 2 - 30))
        {
            validRowsCal(track.pointsEdgeLeft, track.pointsEdgeRight); // 边缘有效行计算
            track.pointsEdgeLeft.resize(validRowsLeft);
            track.pointsEdgeRight.resize(validRowsRight);
        }

        /****补丁****/
        if((track.stdevLeft <= 5 && track.stdevRight > 50) /*&& track.pointsEdgeLeft.size() < ROWSIMAGE/2*/
            &&abs(track.pointsEdgeLeft[0].y - track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].y)<=5)
        {
            track.pointsEdgeLeft.resize(0);
            
            
        }
        else if((track.stdevRight <= 5 && track.stdevLeft > 50) /*&& track.pointsEdgeRight.size() < ROWSIMAGE/2*/
            && abs(track.pointsEdgeRight[0].y - track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].y)<=5)
        {
            track.pointsEdgeRight.resize(0);
        }
       // 通过双边缘有效点的差来判断赛道类型，使用双段三阶贝塞尔拟合中线
        if (track.pointsEdgeLeft.size() > 45 && track.pointsEdgeRight.size() > 45) 
        {
            centerEdge = bezier_curve_fitting(track.pointsEdgeLeft, track.pointsEdgeRight);
            style = "STRIGHT";
        }
        else if (track.pointsEdgeLeft.size() == 0 && track.pointsEdgeRight.size() > 4) // 右单边
        {
            for(int i = 0;i < 7;i++)
                track.pointsEdgeLeft.push_back(POINT(ROWSIMAGE - track.rowCutBottom - i, 0));
            centerEdge = bezier_curve_fitting(track.pointsEdgeLeft, track.pointsEdgeRight);
            style = "LEFT_D";
            //std::cout<<"Statue 2=LEFT_D"<<std::endl;
        }
        else if (track.pointsEdgeLeft.size() > 4 && track.pointsEdgeRight.size() == 0)
        {

            for(int i = 0;i < 7;i++)
                track.pointsEdgeRight.push_back(POINT(ROWSIMAGE - track.rowCutBottom - i, COLSIMAGE - 1));
            centerEdge = bezier_curve_fitting(track.pointsEdgeLeft, track.pointsEdgeRight);
            style = "RIGHT_D";
            //std::cout<<"Statue 3=RIGHT_D"<<std::endl;
        }
        else if ((track.pointsEdgeLeft.size() > 0 && track.pointsEdgeRight.size() <= 45) ||
                 (track.pointsEdgeLeft.size() > 0 && track.pointsEdgeRight.size() > 0 && track.pointsEdgeLeft[0].x - track.pointsEdgeRight[0].x > ROWSIMAGE / 2))
        {
            centerEdge = bezier_curve_fitting(track.pointsEdgeLeft, track.pointsEdgeRight);
            style = "RIGHT";
            //std::cout<<"Statue 4=RIGHT"<<std::endl;
        }
        else if ((track.pointsEdgeRight.size() > 0 && track.pointsEdgeLeft.size() <= 45) ||
                 (track.pointsEdgeRight.size() > 0 && track.pointsEdgeLeft.size() > 0 && track.pointsEdgeRight[0].x - track.pointsEdgeLeft[0].x > ROWSIMAGE / 2))
        {
            centerEdge = bezier_curve_fitting(track.pointsEdgeLeft, track.pointsEdgeRight);
            style = "LEFT";
            //std::cout<<"Statue 5=LEFT"<<std::endl;
        }

        centerEdge_yh = centerEdge;

        // 加权控制中心计算
        double controlNum = 0;
        double controlCenter_Calculate = 0.0;
        for (auto p : centerEdge)
        {
            if (p.x < ROWSIMAGE / 5)
            {
                double temp = normal_pdf(curve_fitting_output(ROWSIMAGE - p.x), miu, singema);
                controlNum += temp * 0.35;
                controlCenter_Calculate += p.y * temp * 0.35;
            }
            else if(p.x < ROWSIMAGE * 2 / 5 && p.x >= ROWSIMAGE / 5)
            {
                double temp = normal_pdf(curve_fitting_output(ROWSIMAGE - p.x), miu, singema);
                controlNum += temp * 0.96;
                controlCenter_Calculate += p.y * temp *0.96;
            }
            else if(p.x < ROWSIMAGE * 3 / 5 && p.x >= ROWSIMAGE * 2 / 5)
            {
                double temp = normal_pdf(curve_fitting_output(ROWSIMAGE - p.x), miu, singema);
                controlNum += temp * 1.21;
                controlCenter_Calculate += p.y * temp * 1.21;
            }
            else if(p.x < ROWSIMAGE * 4 / 5 && p.x >= ROWSIMAGE * 3 / 5)
            {
                double temp = normal_pdf(curve_fitting_output(ROWSIMAGE - p.x), miu, singema);
                controlNum += temp * 2.8;
                controlCenter_Calculate += p.y * temp * 2.8;
            }
            else if(p.x < ROWSIMAGE && p.x >= ROWSIMAGE * 4 / 5)
            {
                double temp = normal_pdf(curve_fitting_output(ROWSIMAGE - p.x), miu, singema);
                controlNum += temp * 0.45;
                controlCenter_Calculate += p.y * temp * 0.45;
            }
        }
        if (controlNum > 0)
        {
            controlCenter = (int)(controlCenter_Calculate / controlNum);
        }

        //限制幅值
        if (controlCenter > COLSIMAGE)
            controlCenter = COLSIMAGE;
        else if (controlCenter < 0)
            controlCenter = 0;
        
        sigmaCenter = track.stdevEdgeCal(centerEdge, 4);
        std::cout<<"sigmaCenter"<<sigmaCenter<<std::endl;
    }

    /**
     * @brief 显示赛道线识别结果
     * @param centerImage 需要叠加显示的图像
     */
    void drawImage(Tracking track, Mat &centerImage)
    {
        // 赛道边缘绘制
        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(centerImage, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 1, Scalar(0, 255, 0), -1); // 绿色点
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(centerImage, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1, Scalar(0, 255, 255), -1); // 黄色点
        }

        for (int i = 0; i < centerEdge.size(); i++)
        {
            circle(centerImage, Point(centerEdge[i].y, centerEdge[i].x), 1, Scalar(0, 0, 255), -1);
        }

        Rect rect(controlCenter, ROWSIMAGE - 20, 10, 20);
        rectangle(centerImage, rect, Scalar(0, 0, 255), CV_FILLED);

        int dis = 20;
        string str;
        putText(centerImage, style, Point(COLSIMAGE - 60, dis), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);

        str = "Edge: " + formatDoble2String(track.stdevLeft, 1) + " | " + formatDoble2String(track.stdevRight, 1);
        putText(centerImage, str, Point(COLSIMAGE - 150, 2 * dis), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);

        str = "Center: " + formatDoble2String(sigmaCenter, 2);
        putText(centerImage, str, Point(COLSIMAGE - 120, 3 * dis), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);

        putText(centerImage, to_string(controlCenter), Point(COLSIMAGE / 2 - 10, ROWSIMAGE - 40), FONT_HERSHEY_PLAIN, 1.2, Scalar(0, 0, 255), 1);
    }

private:
    /**
     * @brief  控制函数曲线拟合
     * @param  _highly_control_point 最高权限控制点
     * @return double curve_fitting_putout 拟合参数值
     */
    double curve_fitting(int _highly_control_point)
    {
        //定义曲线拟合参数输出值
        double curve_fitting_putout = 1;
        int x = _highly_control_point;
        curve_fitting_putout = (log((double)(ROWSIMAGE / 2)) -log((double)ROWSIMAGE)) / (log((double)x) -log((double)ROWSIMAGE));
        return curve_fitting_putout;
    }

    /**
     * @brief  得到拟合参数，采用次方函数拟合
     * @param  _highly_control_point 最高权限控制点
     * @return double curve_fitting_output_value 拟合曲线输出值
     */
    double curve_fitting_output(int point_x)
    {
        double curve_fitting_output_value = 0;
        double m = curve_fitting(highly_control_point);
        curve_fitting_output_value = pow((double)ROWSIMAGE, (1 - m)) * pow((double)point_x, m);
        return curve_fitting_output_value;
    }

    /**
     * @brief  正太分布函数
     * @param  x 函数x的值
     * @param  func_miu 系数μ
     * @param  func_singema 系数singema
     * @return 高斯分布函数输出值
     */
    double normal_pdf(double x, double func_miu, double func_singema)
    {
        static const float inv_sqrt_2pi = 0.3989422804014327;
        double a = (x / func_miu) / func_singema;
        return inv_sqrt_2pi / func_singema * std::exp(-0.5 * a * a) * AMPLIFICATION_FACTOR;
    }

    /**
     * @brief 搜索边线起点（左下）
     * @param pointsEdgeLeft
     * @return uint16_t
     */
    uint16_t searchBreakLeftDown(vector<POINT> pointsEdgeLeft)
    {
        uint16_t counter = 0;

        for (int i = 0; i < pointsEdgeLeft.size() - 10; i++)
        {
            if (pointsEdgeLeft[i].y >= 3)
            {
                counter++;
                if (counter > 3)
                {
                    return i - 2;
                }
            }
            else
                counter = 0;
        }
        return 0;
    }

    /**
     * @brief 搜索边线起点（右下）
     * @param pointsEdgeRight
     * @return uint16_t
     */
    uint16_t searchBreakRightDown(vector<POINT> pointsEdgeRight)
    {
        uint16_t counter = 0;

        for (int i = 0; i < pointsEdgeRight.size() - 10; i++) 
        {
            if (pointsEdgeRight[i].y < COLSIMAGE - 2)
            {
                counter++;
                if (counter > 3)
                {
                    return i - 2;
                }
            }
            else
                counter = 0;
        }
        return 0;
    }

    /**
     * @brief 搜索边线突变点（右下）
     * @param pointsEdgeRight
     * @return uint16_t
     */
    uint16_t search_Mutation_point_right(vector<POINT> pointsEdgeRight)
    {
        bool start = false;
        uint16_t rowBreakRight = 0;
        uint16_t counter = 0;

        for (int i = 0; i < pointsEdgeRight.size(); i++) // 寻找右边跳变点
        {
            if (pointsEdgeRight[i].y < COLSIMAGE - 3 || i >= 30)
                start = true;
            if(start)
            {
                if (pointsEdgeRight[i].y < pointsEdgeRight[rowBreakRight].y)
                {
                    rowBreakRight = i;
                    counter = 0;
                }
                else if (pointsEdgeRight[i].y >= pointsEdgeRight[rowBreakRight].y) // 突变点计数
                {
                    counter++;
                    if (counter > 5)
                        return rowBreakRight;
                }
            }
        }
        return rowBreakRight;
    }

    /**
     * @brief 搜索边线突变点（左下）
     * @param pointsEdgeLeft
     * @return uint16_t
     */
    uint16_t search_Mutation_point_left(vector<POINT> pointsEdgeLeft)
    {
        bool start = false;
        uint16_t rowBreakLeft = 0;
        uint16_t counter = 0;

        for (int i = 0; i < pointsEdgeLeft.size(); i++) // 寻找左边跳变点
        {
            if (pointsEdgeLeft[i].y > 3 || i >= 30)
                start = true;
            if(start)
            {
                if (pointsEdgeLeft[i].y > pointsEdgeLeft[rowBreakLeft].y)
                {
                    rowBreakLeft = i;
                    counter = 0;
                }
                else if (pointsEdgeLeft[i].y <= pointsEdgeLeft[rowBreakLeft].y) // 突变点计数
                {
                    counter++;
                    if (counter > 5)
                        return rowBreakLeft;
                }
            }
        }
        return rowBreakLeft;
    }

    /**
     * @brief 双段三阶贝塞尔拟合中线
     * @param pointsEdgeLeft 赛道左边缘点集
     * @param pointsEdgeRight 赛道右边缘点集
     * @return uint16_t
     */
    vector<POINT> bezier_curve_fitting(vector<POINT> pointsEdgeLeft, vector<POINT> pointsEdgeRight)
    {
        vector<POINT> su_centerEdge;        // 双阶贝塞尔曲中点计算点集
        vector<POINT> centerEdge_func;      // 赛道中心点集
        vector<POINT> v_center(4);          // 三阶贝塞尔曲线
        vector<POINT> center_point(2);      // 中点寻找容器
        POINT v_midpoint;                   // 分段贝塞尔的中点

        //清空点集
        su_centerEdge.clear();
        centerEdge_func.clear();

        //寻找拟合曲线的点集
        v_center[0] = {(pointsEdgeLeft[0].x + pointsEdgeRight[0].x) / 2, (pointsEdgeLeft[0].y + pointsEdgeRight[0].y) / 2};

        v_center[1] = {(pointsEdgeLeft[pointsEdgeLeft.size() / 7].x + pointsEdgeRight[pointsEdgeRight.size() / 7].x) / 2,
                        (pointsEdgeLeft[pointsEdgeLeft.size() / 7].y + pointsEdgeRight[pointsEdgeRight.size() / 7].y) / 2};

        v_center[2] = {(pointsEdgeLeft[pointsEdgeLeft.size() * 2 / 7].x + pointsEdgeRight[pointsEdgeRight.size() * 2 / 7].x) / 2,
                        (pointsEdgeLeft[pointsEdgeLeft.size() * 2 / 7].y + pointsEdgeRight[pointsEdgeRight.size() * 2 / 7].y) / 2};

        center_point[0] = {(pointsEdgeLeft[pointsEdgeLeft.size() * 3 / 7].x + pointsEdgeRight[pointsEdgeRight.size() * 3 / 7].x) / 2,
                            (pointsEdgeLeft[pointsEdgeLeft.size() * 3 / 7].y + pointsEdgeRight[pointsEdgeRight.size() * 3 / 7].y) / 2};
        center_point[1] = {(pointsEdgeLeft[pointsEdgeLeft.size() * 4 / 7].x + pointsEdgeRight[pointsEdgeRight.size() * 4 / 7].x) / 2,
                            (pointsEdgeLeft[pointsEdgeLeft.size() * 4 / 7].y + pointsEdgeRight[pointsEdgeRight.size() * 4 / 7].y) / 2};

        v_center[3] = {(center_point[0].x + center_point[1].x) / 2,
                        (center_point[0].y + center_point[1].y) / 2};

        centerEdge_func = Bezier(0.04, v_center);

        v_center[0] = {(center_point[0].x + center_point[1].x) / 2,
                        (center_point[0].y + center_point[1].y) / 2};

        v_center[1] = {(pointsEdgeLeft[pointsEdgeLeft.size() * 5 / 7].x + pointsEdgeRight[pointsEdgeRight.size() * 5 / 7].x) / 2,
                        (pointsEdgeLeft[pointsEdgeLeft.size() * 5 / 7].y + pointsEdgeRight[pointsEdgeRight.size() * 5 / 7].y) / 2};

        v_center[2] = {(pointsEdgeLeft[pointsEdgeLeft.size() * 6 / 7].x + pointsEdgeRight[pointsEdgeRight.size() * 6 / 7].x) / 2,
                        (pointsEdgeLeft[pointsEdgeLeft.size() * 6 / 7].y + pointsEdgeRight[pointsEdgeRight.size() * 6 / 7].y) / 2};

        v_center[3] = {(pointsEdgeLeft[pointsEdgeLeft.size() - 1].x + pointsEdgeRight[pointsEdgeRight.size() - 1].x) / 2,
                        (pointsEdgeLeft[pointsEdgeLeft.size() - 1].y + pointsEdgeRight[pointsEdgeRight.size() - 1].y) / 2};

        su_centerEdge = Bezier(0.04, v_center);

        for(int i = 0; i < su_centerEdge.size(); i++)
        {
            centerEdge_func.push_back(su_centerEdge[i]);
        }

        return centerEdge_func;
    }

    /**
     * @brief 双段三阶贝塞尔拟合中线
     * @param pointsEdgeMid 赛道中线点集
     * @return uint16_t
     */
    vector<POINT> bezier_curve_fitting(vector<POINT> pointsEdgeMid)
    {
        vector<POINT> su_centerEdge;        // 双阶贝塞尔曲中点计算点集
        vector<POINT> centerEdge_func;      // 赛道中心点集

        vector<POINT> v_center(4);          // 三阶贝塞尔曲线
        vector<POINT> center_point(2);      // 中点寻找容器
        POINT v_midpoint;                   // 分段贝塞尔的中点

        //清空点集
        su_centerEdge.clear();
        centerEdge_func.clear();

        //寻找拟合曲线的点集
        v_center[0] = {pointsEdgeMid[0].x, pointsEdgeMid[0].y};
        v_center[1] = {pointsEdgeMid[pointsEdgeMid.size() / 7].x, pointsEdgeMid[pointsEdgeMid.size() / 7].y};
        v_center[2] = {pointsEdgeMid[pointsEdgeMid.size() * 2 / 7].x, pointsEdgeMid[pointsEdgeMid.size() * 2 / 7].y};

        center_point[0] = {pointsEdgeMid[pointsEdgeMid.size() * 3 / 7].x, pointsEdgeMid[pointsEdgeMid.size() * 3 / 7].y};
        center_point[1] = {pointsEdgeMid[pointsEdgeMid.size() * 4 / 7].x, pointsEdgeMid[pointsEdgeMid.size() * 4 / 7].y};

        v_center[3] = {(center_point[0].x + center_point[1].x), (center_point[0].y + center_point[1].y)};

        centerEdge_func = Bezier(0.03, v_center);

        v_center[0] = {(center_point[0].x + center_point[1].x), (center_point[0].y + center_point[1].y)};

        v_center[1] = {pointsEdgeMid[pointsEdgeMid.size() * 5 / 7].x, pointsEdgeMid[pointsEdgeMid.size() * 5 / 7].y};

        v_center[2] = {pointsEdgeMid[pointsEdgeMid.size() * 6 / 7].x, pointsEdgeMid[pointsEdgeMid.size() * 6 / 7].y};

        v_center[3] = {pointsEdgeMid[pointsEdgeMid.size() - 1].x, pointsEdgeMid[pointsEdgeMid.size() - 1].y};

        su_centerEdge = Bezier(0.03, v_center);

        for(int i = 0; i < su_centerEdge.size(); i++)
        {
            centerEdge_func.push_back(su_centerEdge[i]);
        }

        //返回中心点集
        return centerEdge_func;
    }


    /**
     * @brief 边缘有效行计算：左/右
     * @param pointsEdgeLeft
     * @param pointsEdgeRight
     */
    void validRowsCal(vector<POINT> pointsEdgeLeft, vector<POINT> pointsEdgeRight)
    {
        int counter = 0;
        if (pointsEdgeRight.size() > 10 && pointsEdgeLeft.size() > 10)
        {
            uint16_t rowBreakLeft = searchBreakLeftDown(pointsEdgeLeft);                                           // 左边缘上升拐点->起始点
            uint16_t rowBreakRight = searchBreakRightDown(pointsEdgeRight);                                        // 右边缘上升拐点->起始点

            //截断多余的左边缘贴图片的边缘线
            if (pointsEdgeRight[pointsEdgeRight.size() - 1].y < COLSIMAGE / 2 && rowBreakRight - rowBreakLeft > 5) // 左弯道
            {
                if (pointsEdgeLeft.size() > rowBreakRight) // 左边缘有效行重新搜索
                {
                    for (int i = rowBreakRight; i < pointsEdgeLeft.size(); i++)
                    {
                        if (pointsEdgeLeft[i].y < 5)
                        {
                            counter++;
                            if (counter >= 3)
                            {
                                pointsEdgeLeft.resize(i);
                            }
                        }
                        else
                            counter = 0;
                    }
                }
            }

            else if (pointsEdgeLeft[pointsEdgeLeft.size() - 1].y > COLSIMAGE / 2 && rowBreakLeft - rowBreakRight > 5)
            {
                if (pointsEdgeRight.size() > rowBreakLeft)
                {
                    for (int i = rowBreakLeft; i < pointsEdgeRight.size(); i++)
                    {
                        if (pointsEdgeRight[i].y > COLSIMAGE - 3)
                        {
                            counter++;
                            if (counter >= 3)
                            {
                                pointsEdgeRight.resize(i);
                            }
                        }
                        else
                            counter = 0;
                    }
                }
            }
        }

        validRowsLeft = 0;
        if (pointsEdgeLeft.size() > 1)
        {
            for (int i = pointsEdgeLeft.size() - 1; i >= 1; i--)
            {
                if (pointsEdgeLeft[i].y > 5 && pointsEdgeLeft[i - 1].y >= 5)
                {
                    validRowsLeft = i + 1;
                    break;
                }
                if (pointsEdgeLeft[i].y < 5 && pointsEdgeLeft[i - 1].y >= 5)
                {
                    validRowsLeft = i + 1;
                    break;
                }
            }
        }

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
     * @brief 连续转弯边缘有效行计算：右转
     * @param pointsEdgeLeft
     * @param pointsEdgeRight
     */
    void coiled_validRowsCal_right(vector<POINT> &pointsEdgeLeft, vector<POINT> &pointsEdgeRight)
    {
        if (pointsEdgeRight.size() > 10 && pointsEdgeLeft.size() > 10)
        {
            uint16_t rowBreakLeft = search_Mutation_point_left(pointsEdgeLeft);
            pointsEdgeLeft.resize(rowBreakLeft);
            pointsEdgeRight.resize(pointsEdgeLeft.size());
            style = "RIGHT_CC";
        }
    }

    /**
     * @brief 连续转弯边缘有效行计算：左转
     * @param pointsEdgeLeft
     * @param pointsEdgeRight
     */
    void coiled_validRowsCal_left(vector<POINT> &pointsEdgeLeft, vector<POINT> &pointsEdgeRight)
    {
        if (pointsEdgeRight.size() > 10 && pointsEdgeLeft.size() > 10)
        {
            uint16_t rowBreakRight = search_Mutation_point_right(pointsEdgeRight);
            pointsEdgeRight.resize(rowBreakRight);
            pointsEdgeLeft.resize(pointsEdgeRight.size());
            style = "LEFT_CC";
        }
    }

    /**
     * @brief 搜寻左边线与中线的交叉点
     * @param pointsEdgeLeft
     */
    POINT searchLeftIntersection(std::vector<POINT> pointsEdgeLeft)
    {
        POINT Intersection = POINT(0, 0);
        for(int i = 0; pointsEdgeLeft[i].x > ROWSIMAGE - COLSIMAGE / 2; i++)
        {
            if(pointsEdgeLeft[i].y >= COLSIMAGE / 2)
            {
                Intersection = pointsEdgeLeft[i];
                break;
            }
        }

        return Intersection;
    }

    /**
     * @brief 搜寻右边线与中线的交叉点
     * @param pointsEdgeRight
     */
    POINT searchRightIntersection(std::vector<POINT> pointsEdgeRight)
    {
        POINT Intersection = POINT(0, 0);
        for(int i = 0; pointsEdgeRight[i].x > ROWSIMAGE - COLSIMAGE / 2; i++)
        {
            if(pointsEdgeRight[i].y <= COLSIMAGE / 2)
            {
                Intersection = pointsEdgeRight[i];
                break;
            }
        }

        return Intersection;
    }


    /**
     * @brief 搜索左连续弯道极值点
     * @param pointsEdgeRight 中线
     * @return uint16_t
     */
    uint16_t left_search_value(vector<POINT> pointsEdgeRight)
    {
        uint16_t rowBreakRight = 0;
        uint16_t counter = 0;

        if(pointsEdgeRight.size() == 0)
        {
            return 0;
        }

        // 寻找右边最大点
        for (int i = 0; i < pointsEdgeRight.size() - 1; i++) 
        {
            if (pointsEdgeRight[i].y <= pointsEdgeRight[rowBreakRight].y)
            {
                rowBreakRight = i;
                counter = 0;
            }
            // 突变点计数
            else
            {
                counter++;
                if (counter > 5)
                    return rowBreakRight;
            }
        }
        // 没有搜寻到，返回0
        return rowBreakRight;
    }


    /**
     * @brief 搜索右连续弯道极值点
     * @param pointsEdgeLeft 中线
     * @return uint16_t
     */
    uint16_t right_search_value(vector<POINT> pointsEdgeLeft)
    {
        uint16_t rowBreakLeft = 0;
        uint16_t counter = 0;

        if(pointsEdgeLeft.size() == 0)
        {
            return 0;
        }
        
        // 寻找左边最大点
        for (int i = 0; i < pointsEdgeLeft.size() - 1; i++)
        {
            if (pointsEdgeLeft[i].y >= pointsEdgeLeft[rowBreakLeft].y)
            {
                rowBreakLeft = i;
                counter = 0;
            }
            // 突变点计数
            else
            {
                counter++;
                if (counter > 5)
                    return rowBreakLeft;
            }
        }

        // 没有搜寻到
        return rowBreakLeft;
    }


private:
    int countOutlineA = 0; // 车辆脱轨检测计数器
    int countOutlineB = 0; // 车辆脱轨检测计数器
    /**
	 * @brief 在俯视域计算预瞄点的切线斜率
     * @param line 图像域下的中线
	 * @param index 预瞄点的下标号
	 * @param size 开窗大小
	 * @return 斜率
	 */
    double perspectiveTangentSlope(std::vector<POINT> line, uint16_t index, int size)
    {
        if(line.size() < 5)
            return 0;
            
		// End
		Point2d endIpm = ipm.homography(
			Point2d(line[inRange(line, index-size)].y, line[inRange(line, index-size)].x)); // 透视变换
		POINT p1 = POINT(endIpm.y, endIpm.x);

		// Start
		Point2d startIpm = ipm.homography(
			Point2d(line[inRange(line, index+size)].y, line[inRange(line, index+size)].x)); // 透视变换
		POINT p0 = POINT(startIpm.y, startIpm.x);

        float dx = p1.x - p0.x;
        float dy = p1.y - p0.y;
        if(dx == 0)
            return 10.0;
        else
            return dy / dx;
    }
};
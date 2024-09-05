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
 * @file ring.cpp
 * @author Leo
 * @brief 环岛识别（基于track赛道识别后）
 * @version 0.1
 * @date 2022-02-28
 *
 * @copyright Copyright (c) 2022
 *
 * @note  环岛识别步骤（ringStep）：
 *          1：环岛识别（初始化）
 *          2：入环处理
 *          3：环中处理
 *          4：出环处理
 *          5：出环结束
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "tracking.cpp"

using namespace cv;
using namespace std;

class Ring
{
public:
    Ring(){
        reset();
    }
    /**
     * @brief 环岛识别初始化|复位
     *
     */
    void reset(void)
    {
        ringType = RingType::RingNone; // 环岛类型
        ringStep = RingStep::None;     // 环岛处理阶段
        ring_cnt = 0;
        counterSpurroad = 0;
        counterShield = 0;
    }
    void print(void)
    {
        cout << "RingStep: " << ringStep << endl;
    }

    /**
     * @brief 环岛识别与行径规划
     *
     * @param track 基础赛道识别结果
     * @param imagePath 赛道路径图像
     */
    bool process(Tracking &track, Mat img_bin)
    {
        if (track.pointsEdgeRight.size() < ROWSIMAGE / 4 || track.pointsEdgeLeft.size() < ROWSIMAGE / 4) //环岛有效行限制
        {
            return ringType;
        }

        
        if (counterShield < 8)
        {
            counterShield++;
            return false;
        }
       


        pointBreakD = POINT(0, 0);
        pointBreakU = POINT(0, 0);

        //[1]左右环岛判断
        /*
        if(ringType == RingType::RingNone && ringStep == RingStep::None)
        {
            //std::cout<<"ringType: "<<1<<std::endl;
            uint16_t rowBreakRightDown = searchBreakRightDown(track.pointsEdgeRight, 0, COLSIMAGE / 2 - 20);
            uint16_t rowBreakLeftDown = searchBreakLeftDown(track.pointsEdgeLeft, 0, COLSIMAGE / 2 - 20);
            std::cout<<"rowBreakRightDown: "<<rowBreakRightDown<<std::endl;
            std::cout<<"rowBreakLeftDown: "<<rowBreakLeftDown<<std::endl;
            std::cout<<"track.stdevLeft: "<<track.stdevLeft<<std::endl;
            std::cout<<"track.stdevRight: "<<track.stdevRight<<std::endl;
            std::cout<<"track.widthBlock[rowBreakRightDown + 10].y: "<<track.widthBlock[rowBreakRightDown + 10].y<<std::endl;
            std::cout<<"track.widthBlock[rowBreakLeftDown + 10].y: "<<track.widthBlock[rowBreakLeftDown + 10].y<<std::endl;
            std::cout<<"track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].y: "<<track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].y<<std::endl;
            std::cout<<"track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].y: "<<track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].y<<std::endl;

            if(rowBreakLeftDown != 0 && rowBreakRightDown == 0
                && ((track.stdevLeft > 120 && track.stdevRight < 60) || (track.stdevLeft > 200 && track.stdevRight < 80) || (track.stdevLeft > 100 && track.stdevRight < 40))
                && abs(track.pointsEdgeRight[0].y - track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].y) > 10
                && track.widthBlock[rowBreakLeftDown + 10].y > COLSIMAGE / 2 && track.pointsEdgeLeft[rowBreakLeftDown + 10].y < 5
                && track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].y > COLSIMAGE / 2 - 30)
            {
                std::cout<<"ringType: "<<21<<std::endl;
                // for(int i = rowBreakLeftDown; i < rowBreakLeftDown + 50; i++)
                // {
                //     uint16_t counter = 0;
                //     if(track.pointsEdgeLeft[i].y < 5)
                //         counter++;
                //     if(counter> 30)
                //     {
                //         ring_cnt++;
                //         break;
                //     }
                // }
                ring_cnt++;
                if(ring_cnt > 1)
                {
                    // for(int i = 0; i < track.pointsEdgeLeft.size() / 2; i++)
                    // {
                    //     if(track.pointsEdgeLeft[i].y < 5)
                    //         counterSpurroad++;
                    // }
                    // if(counterSpurroad > 30)
                    //     ringStep = RingStep::Entering;

                    counterSpurroad = 0;
                    ringType = RingType::RingLeft;
                }
            }
            else if(rowBreakLeftDown == 0 && rowBreakRightDown != 0 
                && ((track.stdevLeft < 60 && track.stdevRight > 120) || (track.stdevLeft < 80 && track.stdevRight > 200) || (track.stdevLeft < 40 && track.stdevRight > 100))
                && abs(track.pointsEdgeLeft[0].y - track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].y) > 10
                && track.widthBlock[rowBreakRightDown + 10].y > COLSIMAGE / 2 && track.pointsEdgeRight[rowBreakRightDown + 10].y > COLSIMAGE - 5
                && track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].y < COLSIMAGE / 2 + 30)
            {
                // for(int i = rowBreakRightDown; i < rowBreakRightDown + 50; i++)
                // {
                //     uint16_t counter = 0;
                //     if(track.pointsEdgeRight[i].y > COLSIMAGE - 5)
                //         counter++;
                //     if(counter> 30)
                //     {
                //         ring_cnt++;
                //         break;
                //     }
                // }
                std::cout<<"ringType: "<<22<<std::endl;
                ring_cnt++;
                if(ring_cnt > 1)
                {
                    // for(int i = 0; i < track.pointsEdgeRight.size() / 2; i++)
                    // {
                    //     if(track.pointsEdgeRight[i].y > COLSIMAGE - 5)
                    //         counterSpurroad++;
                    // }
                    // if(counterSpurroad > 30)
                    //     ringStep = RingStep::Entering;

                    counterSpurroad = 0;
                    ringType = RingType::RingRight;
                    std::cout<<"ringType: "<<"22"<<std::endl;
                }
            }
            else
            {
                ring_cnt = 0;
            }
        }
        */
        // /*环岛识别方法2*/
        std::cout<<"the begining of ring"<<std::endl;
        std::cout<<"track.stdevLeft: "<<track.stdevLeft<<std::endl;
        std::cout<<"track.stdevRight: "<<track.stdevRight<<std::endl;
        if(ringType == RingType::RingNone && track.stdevLeft > 80 && track.stdevRight < 50 && abs(track.pointsEdgeRight[0].y - track.pointsEdgeRight[ROWSIMAGE / 2].y) > 5)
        {
           // track.RingStatus = 0;
            std::cout<<"ring STEP 1-1"<<std::endl;
            uint16_t rowBreakLeftDown = searchBreakLeftDown(track.pointsEdgeLeft, 0, ROWSIMAGE / 2);
            std::cout<<"rowBreakLeftDown: "<<rowBreakLeftDown<<std::endl;
            uint16_t counter = 0, step = 2;
            int counterExit=0;
            for(int i = rowBreakLeftDown; i < track.pointsEdgeLeft.size(); i++)
            {
                if(track.pointsEdgeLeft[i].y < 5)
                {
                    //if(step % 2 == 0)
                        counter++;
                        counterExit++;
                    //else if(step % 2 == 1)
                    //  counter = 0;
                }
                else
                {
                    if(step % 2 == 0)
                        counter = 0;
                    else if(step % 2 == 1)
                        counter++;
                }
                if(counter > 3)
                {
                    step += 1;
                    counter = 0;
                    if(step >= 3 && rowBreakLeftDown != 0)
                    {
                        ringType = RingType::RingLeft;
                        track.RingStatus = 1;
                        std::cout<<"ringleft"<<std::endl;
                    }
                }
            }
        }
        else if(ringType == RingType::RingNone && track.stdevLeft < 50 && track.stdevRight > 80 && abs(track.pointsEdgeLeft[0].y - track.pointsEdgeLeft[ROWSIMAGE / 2].y) > 5)
        {
                std::cout<<"ring STEP 1-1"<<std::endl;
                //std::cout<<"ring STEP 1-2"<<std::endl;
                uint16_t rowBreakRightDown = searchBreakRightDown(track.pointsEdgeRight, 0, ROWSIMAGE / 2);
                std::cout<<"rowBreakRightDown: "<<rowBreakRightDown<<std::endl;
                uint16_t counter = 0, step = 2;
                int counterExit = 0;
                 for(int i = rowBreakRightDown; i < track.pointsEdgeRight.size(); i++)
                 {
                     
                     if(track.pointsEdgeRight[i].y > COLSIMAGE - 15)
                     {
                        
                         //if(step % 2 == 0)
                             counter++;
                         //else if(step % 2 == 1)
                           //  counter = 0;
                        counterExit++;
                        //std::cout<<"counterExit: "<<counterExit<<std::endl;
                     }
                     else
                     {
                         if(step % 2 == 0)
                             counter = 0;
                         else if(step % 2 == 1)
                             counter++;
                     }
                     if(counter > 3)
                     {
                        
                         step += 1;
                         counter = 0;
                         if(step == 4 && rowBreakRightDown != 0)
                         {
                             ringType = RingType::RingRight;
                             track.RingStatus = 2;
                             ringStep = RingStep::None;
                         }
                     }
                }
                // std::cout<<"ringType: "<<ringType<<std::endl;
                // std::cout<<"ringStep: "<<ringStep<<std::endl;
                
        }

        if(ringType != RingType::RingNone && ringStep == RingStep::None)
        {
          
			// counterExit++;
			// if (counterExit > 40) {
			//   reset();
			//   return false;
			// }
            //std::cout<<"ring STEP 2"<<std::endl;
            if(ringType == RingType::RingLeft)
            {
                uint16_t rowBreakLeftD = searchBreakLeftDown(track.pointsEdgeLeft, 0, ROWSIMAGE / 2);
                uint16_t rowBreakLeftU = searchBreakLeftDown(track.pointsEdgeLeft, rowBreakLeftD + 30, track.pointsEdgeLeft.size());
                
                pointBreakD = track.pointsEdgeLeft[rowBreakLeftD];
                if(rowBreakLeftD && rowBreakLeftU && rowBreakLeftU > rowBreakLeftD)///////////////////////////////
                {
                    counterSpurroad++;
                    pointBreakU = track.pointsEdgeLeft[rowBreakLeftU];
                    line(track.pointsEdgeLeft, rowBreakLeftD, rowBreakLeftU);
                }
                else if(rowBreakLeftD && !rowBreakLeftU)
                {
                    counterSpurroad++;
                    line(track.pointsEdgeLeft, rowBreakLeftD, track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1]);
                }
                else if(!rowBreakLeftD && rowBreakLeftU <180 && track.pointsEdgeLeft[rowBreakLeftU].x>200 &&
                   track.pointsEdgeLeft[rowBreakLeftU].y <20)
                {
                    counterSpurroad = 0;
                    ringStep = RingStep::Entering;
                    //std::cout<<"ringStep: "<<"Entering"<<std::endl;
                    
                }
                else 
                {
                    counterSpurroad++;
                    track.pointsEdgeLeft = track.predictEdgeLeft(track.pointsEdgeRight);
                }


                {
                    uint16_t edgecounter = 0;
                    for(int i = 0; i < track.pointsEdgeLeft.size(); i++)
                    {
                        if(track.pointsEdgeLeft[i].y < 5)
                            edgecounter++;
                        else
                            edgecounter = 0;
                    }
                    if(edgecounter > track.pointsEdgeLeft.size() / 2)
                    {
                        //ringStep = RingStep::Entering;
                        counterSpurroad = 0;
                    }
                }             
            }
            else if(ringType == RingType::RingRight)
            {
                uint16_t rowBreakRightD = searchBreakRightDown(track.pointsEdgeRight, 0, ROWSIMAGE / 2);
                uint16_t rowBreakRightU = searchBreakRightDown(track.pointsEdgeRight, rowBreakRightD + 30, track.pointsEdgeRight.size());

                pointBreakD = track.pointsEdgeRight[rowBreakRightD];

                if(rowBreakRightD && rowBreakRightU && rowBreakRightU > rowBreakRightD)//////////////////////
                {
                    counterSpurroad++;
                    pointBreakU = track.pointsEdgeRight[rowBreakRightU];
                    line(track.pointsEdgeRight, rowBreakRightD, rowBreakRightU);
                }
                else if(rowBreakRightD && !rowBreakRightU)
                {
                    counterSpurroad++;
                    line(track.pointsEdgeRight, rowBreakRightD, track.pointsEdgeRight[track.pointsEdgeRight.size() - 1]);
                }
                else if(!rowBreakRightD && rowBreakRightU <180 && track.pointsEdgeRight[rowBreakRightU].x>200 &&
                   track.pointsEdgeRight[rowBreakRightU].y >300)
                {
                    counterSpurroad = 0;
                    ringStep = RingStep::Entering;
                    //std::cout<<"ringStep: "<<"Entering"<<std::endl;
                }
                else 
                {
                    counterSpurroad++;
                    track.pointsEdgeRight = track.predictEdgeRight(track.pointsEdgeLeft);
                }

                {
                    uint16_t edgecounter = 0;
                    for(int i = 0; i < track.pointsEdgeRight.size(); i++)
                    {
                        if(track.pointsEdgeRight[i].y > COLSIMAGE - 5)
                            edgecounter++;
                        else
                            edgecounter = 0;
                    }
                    if(edgecounter > track.pointsEdgeRight.size() / 2)
                    {
                        ringStep = RingStep::Entering;
                        counterSpurroad = 0;
                    }
                }

            }

        }
        else if(ringStep == RingStep::Entering)
        {
            std::cout<<"ring of Entering"<<std::endl;
            //bool repaired = false; 
            _corner = POINT(0, 0);
            static bool flag_Inside = false;
            static bool repaired_flag = false;
            //std::cout<<"Status you have intered the Entring of the ring "<<std::endl;
            if(ringType == RingType::RingLeft)
            {
                //std::cout<<"Status you have intered the Entring of the ring CHECKED"<<std::endl;
                
                uint16_t rowBreakLeftD = 0;
                uint16_t spurroad_item = 0;
                bool repaired = false;
                
                //std::cout<<"spurroadsize: "<<track.spurroad.size()<<std::endl;
                if(track.spurroad.size() == 0)
                {
                    counterSpurroad++;
                }
                else if(track.spurroad.size() == 1)
                {
                    //寻找岔路行对应边线下标
                    if(track.spurroad[0].y > 70 && track.spurroad[0].x < ROWSIMAGE - 40)
                    {
                        _corner = track.spurroad[0];
                        spurroad_item = abs(_corner.x - track.pointsEdgeLeft[0].x);
                        rowBreakLeftD = searchBreakLeftDown(track.pointsEdgeLeft, 0, spurroad_item);
                    }
                    else
                    {
                        rowBreakLeftD = searchBreakLeftDown(track.pointsEdgeLeft, 0, track.pointsEdgeLeft.size());
                        uint16_t rowBreakLeftU = searchBreakLeftUp(track.pointsEdgeLeft);
                        if(rowBreakLeftU > ROWSIMAGE / 2 && rowBreakLeftU > rowBreakLeftD)
                        {
                            spurroad_item = rowBreakLeftU;
                            _corner = track.pointsEdgeLeft[rowBreakLeftU];
                        }
                    }
                }
                else
                {
                    for(int i = 0; i < track.spurroad.size(); i++)
                    {
                        if(track.spurroad[i].x > _corner.x && track.spurroad[i].y > 70 && track.spurroad[i].x < COLSIMAGE / 2)
                        {
                            _corner = track.spurroad[i];
                        }
                        spurroad_item = abs(_corner.x - track.pointsEdgeLeft[0].x);
                        rowBreakLeftD = searchBreakLeftDown(track.pointsEdgeLeft, 0, spurroad_item);
                    }
                }

                pointBreakD = track.pointsEdgeLeft[track.pointsEdgeLeft.size()/2];
                pointBreakU = track.pointsEdgeLeft[rowBreakLeftD];

                if(rowBreakLeftD)
                {
                    std::cout<<"line 0"<<std::endl;
                    line(track.pointsEdgeLeft, 0, rowBreakLeftD);
                    repaired = true;
                }
                else if(_corner.x == 0)
                {
                    std::cout<<"line 1"<<std::endl;
                    line(track.pointsEdgeLeft, 0, track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1]);

                }
                if(_corner.x< ROWSIMAGE/3  &&  track.spurroad.size()!=0)
                {
                    
                    line(track.pointsEdgeRight, rowBreakLeftD, _corner);
                    {
                        uint16_t rowBreakRightD = rowBreakLeftD;
                        if(rowBreakRightD > 50)
                            rowBreakRightD -= 50;
                        else
                            rowBreakRightD = 0;
                        if(rowBreakRightD < 3)
                        {
                            float rowRate = _corner.x;
                            if(rowRate < 50)
                                rowRate = 0;
                            else
                                rowRate -= 50;
                            track.pointsEdgeRight[rowBreakRightD].y -= (rowRate / ROWSIMAGE) * (COLSIMAGE - _corner.y);
                        }
                        POINT startPoint = track.pointsEdgeRight[rowBreakRightD];
                        POINT endPoint = _corner;
                        POINT midPoint = POINT((0.3*startPoint.x + 0.7*endPoint.x), (0.3*startPoint.y + 0.7*endPoint.y));
                        midPoint.y += abs(startPoint.y - endPoint.y) / 4;
                        uint16_t rowBreakMid = abs(midPoint.x - startPoint.x) + rowBreakRightD - 1;
                        
                        line(track.pointsEdgeRight, rowBreakRightD, midPoint);
                        line(track.pointsEdgeRight, rowBreakMid, endPoint);
                        //line(track.pointsEdgeRight, rowBreakRightD, endPoint);
                    }
                    track.pointsEdgeLeft.resize(spurroad_item);
                    track.pointsEdgeRight.resize(spurroad_item);

                    const uint16_t width_thresh = track.pointsEdgeRight[spurroad_item - 1].y - track.pointsEdgeLeft[spurroad_item - 1].y;
                    uint16_t mid = (track.pointsEdgeLeft[spurroad_item - 1].y + track.pointsEdgeRight[spurroad_item - 1].y) / 2;
                    POINT left(0, 0);
                    POINT right(0, 0);
                    for(int i = _corner.x - 1; i > ROWSIMAGE / 4; i--)
                    {
                        int j = mid;
                        for(j = mid; j < COLSIMAGE - 5; j++)
                        {
                            if(img_bin.at<uchar>(i, j) == 255 && img_bin.at<uchar>(i, j + 1) == 0 
                                && img_bin.at<uchar>(i, j + 2) == 0 && img_bin.at<uchar>(i, j + 3) == 0)
                            {
                                right = POINT(i, j);
                                break;
                            }
                        }
                        if(j == COLSIMAGE - 5)
                        {
                            right = POINT(i, COLSIMAGE - 1);
                        }

                        for(j = mid; j > 5; j--)
                        {
                            if(img_bin.at<uchar>(i, j) == 255 && img_bin.at<uchar>(i, j - 1) == 0 
                                && img_bin.at<uchar>(i, j - 2) == 0 && img_bin.at<uchar>(i, j - 3) == 0)
                            {
                                left = POINT(i, j);
                                break;
                            }
                        }
                        if(j == 5)
                        {
                            left = POINT(i, 0);
                        }
                        uint16_t width = abs(right.y - left.y);
                        if(width < COLSIMAGE / 10 || width > width_thresh)
                        {
                            break;
                        }
                        if(left.x && right.x)
                        {
                            track.pointsEdgeLeft.push_back(left);
                            track.pointsEdgeRight.push_back(right);
                            track.widthBlock.push_back(POINT(i, width));
                        }

                        mid = (right.y + left.y) / 2;
                    }
                }
                else if((track.spurroad.size() == 0 || _corner.x==0 || counterSpurroad>2)|| (repaired_flag&&flag_Inside))
                {
                    counterSpurroad = 0;
                    repaired = false;
                    flag_Inside = false;
                    repaired_flag = false;
                    ringStep = RingStep::Inside;
                }
                else if((repaired || _corner.x>ROWSIMAGE/3))//右边线优化
                {
                    repaired_flag = true;
                    if(_corner.x>ROWSIMAGE*2/3)
                    {
                        flag_Inside = true;
                    }
                    //std::cout<<"line 2"<<std::endl;
                    if(track.pointsEdgeLeft.size()>20)
                    {
                        vector<POINT> newPointsLeft;
                        int i;
                        bool flag = false;
                        for ( i = 0; i < track.pointsEdgeLeft.size(); )
                        {
                            // 确保 i+1 在容器的范围内
                            if (i+1 < track.pointsEdgeLeft.size()) {
                                // 检查 y 坐标之间的差值是否大于 ROWSIMAGE/3
                                if (track.pointsEdgeLeft[i+1].y - track.pointsEdgeLeft[i].y > ROWSIMAGE / 3 || flag) {
                                    // 删除满足条件的点，但不改变 i 的值
                                    //std::cout<<"i: "<<i<<" track.pointedgeleft.x::::"<<track.pointsEdgeLeft[i].x<<" "<<track.pointsEdgeLeft[i].y<<std::endl;
                                    track.pointsEdgeLeft.erase(track.pointsEdgeLeft.begin() + i);
                                    flag=true;
                                } else {
                                    // 将点添加到 newPointsLeft 中，并移动到下一个点
                                    newPointsLeft.push_back(track.pointsEdgeLeft[i]);
                                    ++i;
                                }
                            } else {
                                // 如果 i+1 超出范围，将剩下的点添加到 newPointsLeft 中
                                newPointsLeft.push_back(track.pointsEdgeLeft[i]);
                                break;
                            }
                        }
                        if (!track.pointsEdgeLeft.empty()) 
                        {
                            track.pointsEdgeLeft.pop_back();
                        }

                        std::vector<POINT> Point_arr;
                        POINT P(0,0);
                        Point_arr.push_back(track.pointsEdgeRight[0]);
                        Point_arr.push_back(P);
                        //line(track.pointsEdgeRight, 0, track.pointsEdgeLeft[i-2]);
                        track.pointsEdgeRight = Bezier(0.05,Point_arr);

                        if(track.pointsEdgeLeft[track.pointsEdgeLeft.size()-1].y > COLSIMAGE/4)
                        {
                            track.pointsEdgeLeft.clear();
                            track.pointsEdgeLeft = track.predictEdgeLeft(track.pointsEdgeRight);
                        }

                        for(int j =0;j<track.pointsEdgeRight.size();j++)
                        {
                            
                            if(track.pointsEdgeRight[j].x > track.pointsEdgeLeft[i-2].x)
                            {
                                track.pointsEdgeRight.erase(track.pointsEdgeRight.begin() + j);
                            }
                            
                        }
                        //std::cout<<"flag"<<std::endl;
                        i=0;
                        flag = false;
                        newPointsLeft.clear();
                    }
                    else{
                        //std::cout<<"points is to less"<<std::endl;
                    }
                }
                
                    
            }
            else if(ringType == RingType::RingRight)
            {
                static bool repaired = false;
                //static bool flag_Inside = false;
                uint16_t spurroad_item = 0;
                uint16_t rowBreakRightD = 0;
                
                if(track.spurroad.size() == 0)
                {
                    counterSpurroad++;
                }
                else if(track.spurroad.size() == 1)
                {
                    //std::cout<<"Status 2"<<std::endl;
                    if(track.spurroad[0].y < COLSIMAGE - 70 && track.spurroad[0].x < ROWSIMAGE - 40)
                    {
                        //寻找岔路行对应边线下标
                        _corner = track.spurroad[0];
                        spurroad_item = abs(_corner.x - track.pointsEdgeRight[0].x);
                        rowBreakRightD = searchBreakRightDown(track.pointsEdgeRight, 0, spurroad_item);
                    }
                    else
                    {
                        rowBreakRightD = searchBreakRightDown(track.pointsEdgeRight, 0, track.pointsEdgeRight.size());
                        uint16_t rowBreakRightU = searchBreakRightUp(track.pointsEdgeRight);
                        if(rowBreakRightU > ROWSIMAGE / 2 && rowBreakRightU > rowBreakRightD)
                        {
                            _corner = track.pointsEdgeRight[rowBreakRightU];
                        }
                    }
                }
                else 
                {
                    for(int i = 0; i < track.spurroad.size(); i++)
                    {
                        
                        if(track.spurroad[i].x > _corner.x && track.spurroad[i].y < COLSIMAGE - 70 )//&& track.spurroad[i].x < COLSIMAGE / 2)
                        {
                            _corner = track.spurroad[i];
                        }
                        spurroad_item = abs(_corner.x - track.pointsEdgeRight[0].x);
                        rowBreakRightD = searchBreakRightDown(track.pointsEdgeRight, 0, spurroad_item);
                    }
                    
                }
                pointBreakD = track.pointsEdgeRight[track.pointsEdgeRight.size()/2];//
                pointBreakU = track.pointsEdgeRight[rowBreakRightD];

                

                if(rowBreakRightD)
                {
                    repaired = true;
                    line(track.pointsEdgeRight, 0, rowBreakRightD);
                    std::cout<<"line 0"<<std::endl;
                }
                else if(_corner.x == 0)
                {
                    line(track.pointsEdgeRight, 0, track.pointsEdgeRight[track.pointsEdgeRight.size() - 1]);
                    std::cout<<"line 1"<<std::endl;
                }

                if(_corner.x < ROWSIMAGE / 3 && track.spurroad.size()!=0)
                {
                    std::cout<<"line 2"<<std::endl;
                    //std::cout<<"the step of 1"<<std::endl;
                    line(track.pointsEdgeLeft, rowBreakRightD, _corner);
                    {
                        uint16_t rowBreakLeftD = rowBreakRightD;
                        if(rowBreakLeftD > 50)
                            rowBreakLeftD -= 50;
                        else 
                            rowBreakLeftD = 0;
                        if(rowBreakLeftD < 3)
                        {
                            float rowRate = _corner.x;
                            if(rowRate < 50)
                                rowRate = 0;
                            else
                                rowRate -= 50;
                            track.pointsEdgeLeft[rowBreakLeftD].y += (rowRate / ROWSIMAGE) * (_corner.y);
                        }
                        POINT startPoint = track.pointsEdgeLeft[rowBreakLeftD];
                        POINT endPoint = _corner;
                        POINT midPoint = POINT((0.3*startPoint.x + 0.7*endPoint.x), (0.3*startPoint.y + 0.7*endPoint.y));
                        midPoint.y -= abs(startPoint.y - endPoint.y) / 4;
                        uint16_t rowBreakMid = abs(midPoint.x - startPoint.x) + rowBreakLeftD - 1;
                        line(track.pointsEdgeLeft, rowBreakLeftD, midPoint);
                        line(track.pointsEdgeLeft,rowBreakMid,endPoint);
                        
                    }
                    track.pointsEdgeLeft.resize(spurroad_item);
                    track.pointsEdgeRight.resize(spurroad_item);

                    const uint16_t width_thresh = track.pointsEdgeRight[spurroad_item - 1].y - track.pointsEdgeLeft[spurroad_item - 1].y;
                    uint16_t mid = (track.pointsEdgeLeft[spurroad_item - 1].y + track.pointsEdgeRight[spurroad_item - 1].y) / 2;
                    POINT left(0, 0);
                    POINT right(0, 0);
                    for(int i = _corner.x - 1; i > ROWSIMAGE / 4; i--)
                    {
                        int j = mid;
                        for(j = mid; j < COLSIMAGE - 5; j++)
                        {
                            if(img_bin.at<uchar>(i, j) == 255 && img_bin.at<uchar>(i, j + 1) == 0 
                                && img_bin.at<uchar>(i, j + 2) == 0 && img_bin.at<uchar>(i, j + 3) == 0)
                            {
                                right = POINT(i, j);
                                break;
                            }
                        }
                        if(j == COLSIMAGE - 5)
                        {
                            right = POINT(i, COLSIMAGE - 1);
                        }

                        for(j = mid; j > 5; j--)
                        {
                            if(img_bin.at<uchar>(i, j) == 255 && img_bin.at<uchar>(i, j - 1) == 0 
                                && img_bin.at<uchar>(i, j - 2) == 0 && img_bin.at<uchar>(i, j - 3) == 0)
                            {
                                left = POINT(i, j);
                                break;
                            }
                        }
                        if(j == 5)
                        {
                            left = POINT(i, 0);
                        }
                        uint16_t width = abs(right.y - left.y);
                        if(width < COLSIMAGE / 10 || width > width_thresh)
                        {
                            break;
                        }
                        if(left.x && right.x)
                        {
                            track.pointsEdgeLeft.push_back(left);
                            track.pointsEdgeRight.push_back(right);
                            track.widthBlock.push_back(POINT(i, width));
                        }

                        mid = (right.y + left.y) / 2;
                    }
                    
                }
                else if((_corner.x == 0 || track.spurroad.size()==0||counterSpurroad>2) || (repaired_flag&&flag_Inside))
                {
                    std::cout<<"-----------------------------------"<<std::endl;
                    counterSpurroad = 0;
                    flag_Inside = false;
                    repaired = false;
                    repaired_flag = false;
                    ringStep = RingStep::Inside;
                }
                else if(repaired || _corner.x > ROWSIMAGE / 3)//左边线优化
                {
                    repaired_flag = true;
                    if(_corner.x>ROWSIMAGE*2/3)
                    {
                        std::cout<<"#########################"<<std::endl;
                        flag_Inside = true;
                    }
                    
                    if(track.pointsEdgeRight.size()>20)
                    {
                        vector<POINT> newPointsRight;
                        int i;
                        
                        bool flag = false;
                        for ( i = 0; i < track.pointsEdgeRight.size(); )
                        {
                            // 确保 i+1 在容器的范围内
                            if (i+1 < track.pointsEdgeRight.size()) {
                                // 检查 y 坐标之间的差值是否大于 ROWSIMAGE/3
                                if (track.pointsEdgeRight[i].y - track.pointsEdgeRight[i+1].y > ROWSIMAGE / 3 || flag) {
                                    // 删除满足条件的点，但不改变 i 的值
                                    track.pointsEdgeRight.erase(track.pointsEdgeRight.begin() + i);
                                    flag=true;
                                } else {
                                    // 将点添加到 newPointsRight 中，并移动到下一个点
                                    newPointsRight.push_back(track.pointsEdgeRight[i]);
                                    ++i;
                                }
                            } else {
                                // 如果 i+1 超出范围，将剩下的点添加到 newPointsRight 中
                                newPointsRight.push_back(track.pointsEdgeRight[i]);
                                break;
                            }
                        }
                        if (!track.pointsEdgeRight.empty()) 
                        {
                            track.pointsEdgeRight.pop_back();
                        }
                        std::vector<POINT> Point_arr;
                        POINT P(0,320);
                        Point_arr.push_back(track.pointsEdgeLeft[0]);
                        Point_arr.push_back(P);
                        //line(track.pointsEdgeLeft, 0, track.pointsEdgeRight[i-2]);
                        track.pointsEdgeLeft = Bezier(0.08,Point_arr);

                        if(track.pointsEdgeRight[track.pointsEdgeRight.size()-1].y <COLSIMAGE*3/4)
                        {
                            track.pointsEdgeRight.clear();
                            track.pointsEdgeRight = track.predictEdgeRight(track.pointsEdgeLeft);
                        }
                        
                        for(int j =0;j<track.pointsEdgeLeft.size();j++)
                        {
                            
                            if(track.pointsEdgeLeft[j].x < track.pointsEdgeRight[i-2].x)
                            {
                                track.pointsEdgeLeft.erase(track.pointsEdgeLeft.begin() + j);
                            }
                            
                        }
                        //std::cout<<"flag"<<std::endl;
                        i=0;
                        flag = false;
                        newPointsRight.clear();
                    }
                    else{
                        //std::cout<<"points is to less"<<std::endl;
                    }
                }
                
                
                //std::cout<<"track.pointsEdgeRight.size()"<<track.pointsEdgeRight.size()<<std::endl;
            }
        }
        else if(ringStep == RingStep::Inside)
        {
            std::cout<<"ring ----------------------of---------------------------- Inside"<<std::endl;
            if(ringType == RingType::RingLeft)
            {
                uint16_t rowBreakRight = searchBreakRightDown(track.pointsEdgeRight, 0, track.pointsEdgeRight.size() - 10);
                //寻找左边跳变点
                if(rowBreakRight) {
                    track.pointsEdgeRight.erase(
                        track.pointsEdgeRight.begin() + rowBreakRight, 
                        track.pointsEdgeRight.end()
                    );
                }
                uint16_t rowBreakLeft = 0;
                uint16_t counter = 0;

                if(track.pointsEdgeLeft.size() > 31)
                {
                    for(int i =track.pointsEdgeLeft.size(); i > 30; i--)
                    {
                        // 检查当前点是否满足删除条件
                            if (track.pointsEdgeLeft[i].y > track.pointsEdgeLeft[i - 1].y) 
                            {
                                // 使用erase方法删除当前点
                                track.pointsEdgeLeft.erase(track.pointsEdgeLeft.begin() + i);
                                // 删除后不需要递减i，因为我们已经处理了当前索引
                                // 但要确保循环条件仍然有效，即不小于minPointsToKeep
                                if (track.pointsEdgeLeft.size() <= 30) {
                                    break; // 退出循环，因为我们已经达到了最小保留点数
                                }
                            }
                    }
                }
                if(track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].y < COLSIMAGE / 2)
                    {
                        ringStep = RingStep::Exiting;
                        counterSpurroad = 0;
                    }
                
                if(track.pointsEdgeRight[rowBreakRight].x<ROWSIMAGE/2)
                {
                    counterSpurroad++;
                    if(counterSpurroad>2)
                    {
                        ringStep = RingStep::Exiting;
                        counterSpurroad = 0;
                    }
                }
                
            }
            else if(ringType == RingType::RingRight)
            {
                uint16_t rowBreakLeft = searchBreakLeftDown(track.pointsEdgeLeft, 0, track.pointsEdgeLeft.size() - 10);
                //寻找右边跳变点
                if (rowBreakLeft) {
                    track.pointsEdgeLeft.erase(
                        track.pointsEdgeLeft.begin() + rowBreakLeft, 
                        track.pointsEdgeLeft.end()
                    );
                }
                uint16_t rowBreakRight = 0;
                uint16_t counter = 0;
                
                if(track.pointsEdgeRight.size() > 31)
                {
                    for(int i = track.pointsEdgeRight.size(); i > 30; i--)
                    {
                        // 检查当前点是否满足删除条件
                            if (track.pointsEdgeRight[i].y < track.pointsEdgeRight[i - 1].y) 
                            {
                                // 使用erase方法删除当前点
                                track.pointsEdgeRight.erase(track.pointsEdgeRight.begin() + i);
                                // 删除后不需要递减i，因为我们已经处理了当前索引
                                // 但要确保循环条件仍然有效，即不小于minPointsToKeep
                                if (track.pointsEdgeRight.size() <= 30) {
                                    break; // 退出循环，因为我们已经达到了最小保留点数
                                }
                            }
                    }
                }
                
                        
                
                 std::cout<<"rowBreakLeft:--------------------------------> "<<rowBreakLeft<<std::endl;
                 std::cout<<"rowBreakRight:------------------------------> "<<rowBreakRight<<std::endl;
                if(track.pointsEdgeLeft[rowBreakLeft].x<ROWSIMAGE/2)
                {
                    counterSpurroad++;
                    if(counterSpurroad>2)
                    {
                        std::cout<<"ringStep:--------------------------------------> "<<"Exiting"<<std::endl;
                        ringStep = RingStep::Exiting;
                        counterSpurroad = 0;
                    }
                }
                
            }
        }


        else if(ringStep == RingStep::Exiting)
        {
            std::cout<<"ring of Exiting"<<std::endl;
            
            static vector<POINT> PointsLeft;
            static vector<POINT> PointsRight;
            
            uint16_t rowBreakLeft = 0;
            uint16_t rowBreakRight = 0;
            uint16_t counter = 0;
            //寻找左边跳变点
            
            if(!track.pointsEdgeLeft.empty())
            {
                for(int i = track.pointsEdgeLeft.size() ; i > 0; i--)
                {
                    if(i-1>0)
                    {
                        if(track.pointsEdgeLeft[i-1].y - track.pointsEdgeLeft[i].y<0)
                        {
                            track.pointsEdgeLeft.pop_back();
                            
                            std::cout<<"rowBreakLeft: "<<rowBreakLeft<<std::endl;
                
                        }
                        else{
                            break;
                        }
                    }
                }
            }
            else{
                rowBreakLeft = 0;
            }
            
            
            //寻找右边跳变点
            if(!track.pointsEdgeRight.empty())
            {
                for(int i = track.pointsEdgeRight.size()-1; i>0 ; i--)
                {
                    if(i-1>0)
                    {
                        if(track.pointsEdgeRight[i-1].y - track.pointsEdgeRight[i].y>0)
                        {
                            
                            track.pointsEdgeRight.pop_back();
                            rowBreakRight = i;
                            //std::cout<<"rowBreakRight: "<<rowBreakRight<<std::endl;
                        }
                        else{
                            break;
                        }
                    }
                }
            }
            else{
                rowBreakRight = 0;
            }
            
            rowBreakLeft = searchBreakLeftDown(track.pointsEdgeLeft, 0, track.pointsEdgeLeft.size() - 10);
            //rowBreakRight = searchBreakRightDown(track.pointsEdgeRight, 0, track.pointsEdgeRight.size() - 10);
            //rowBreakRight = searchBreakRightDown(track.pointsEdgeRight, 0, track.pointsEdgeRight.size() - 10);
            if(ringType == RingType::RingLeft)
            {
                uint16_t rowBreakLeftD = searchBreakLeftDown(track.pointsEdgeLeft,0,ROWSIMAGE/2);
                uint16_t rowBreakLeftU = searchBreakLeftDown(track.pointsEdgeLeft,rowBreakLeftD + 30, track.pointsEdgeLeft.size());
                //rowBreakLeft = searchBreakLeftUp(track.pointsEdgeLeft);
                ///std::cout<<"rowBreakLeftU: "<<track.pointsEdgeLeft[rowBreakLeftU].x<<"|"<<track.pointsEdgeLeft[rowBreakLeftU].y<<std::endl;
                ///std::cout<<"rowBreakLeft---------->: "<<track.pointsEdgeLeft[rowBreakLeft].x <<"|"<< track.pointsEdgeLeft[rowBreakLeft].y<<std::endl;
                
                // if (track.pointsEdgeLeft.size() > 3) { // 确保向量至少有4个点
                //     // 从向量的最后一个元素开始向前遍历
                //     for (int i = track.pointsEdgeLeft.size() - 1; i >= 0; i--) {
                //         // 检查是否有足够的元素来进行比较
                //         if (i > 2) {
                //             // 检查第i个点的y值是否大于5
                //             if (track.pointsEdgeLeft[i].y > 5) {
                //                 // 删除满足条件的点
                //                 track.pointsEdgeLeft.erase(track.pointsEdgeLeft.begin() + i);
                //                 // 打印删除点后的索引
                //                 // std::cout << "Deleted point at index: " << i << std::endl;
                //             }
                //         } else {
                //             break;
                //         }
                //     }
                // }
                
                if(track.pointsEdgeLeft[rowBreakLeftU].x < ROWSIMAGE/2)
                {
                    std::cout<<"The next part of the ring"<<std::endl;
                    ringStep = RingStep::Finish;
                }
                else
                {
                    pointBreakU = track.pointsEdgeLeft[rowBreakLeft-5];
                    pointBreakD = track.pointsEdgeRight[0];
                    std::vector<POINT> Ring_Right;
                    //line(track.pointsEdgeRight, 0, pointBreakU);
                    if(rowBreakRight&&track.pointsEdgeRight[rowBreakRight].x<ROWSIMAGE/2)
                    {
                        Ring_Right.push_back(track.pointsEdgeRight[0]);
                        Ring_Right.push_back(track.pointsEdgeRight[rowBreakRight]);
                        Ring_Right.push_back(pointBreakU);
                    }else{
                        Ring_Right.push_back(track.pointsEdgeRight[0]);
                        Ring_Right.push_back(track.pointsEdgeLeft[track.pointsEdgeLeft.size()-1]);
                    }
                    track.pointsEdgeRight = Bezier(0.05, Ring_Right);
                }
            }
           else if(ringType == RingType::RingRight)
            {
                uint16_t rowBreakRightD = searchBreakRightDown(track.pointsEdgeRight, 0, ROWSIMAGE/2);
                uint16_t rowBreakRightU = searchBreakRightDown(track.pointsEdgeRight, rowBreakRightD + 30, track.pointsEdgeRight.size());
                //rowBreakRight = searchBreakRightDown(track.pointsEdgeRight, 0, track.pointsEdgeRight.size() - 10);
                std::cout<<"rowBreakRightU: "<<rowBreakRightD<<std::endl;
                std::cout<<"rowBreakRight---------->: "<<track.pointsEdgeRight[rowBreakRightU].x <<"|"<< track.pointsEdgeRight[rowBreakRightU].y<<std::endl;
                

                // if(track.pointsEdgeRight.size()>3){
                //      for (int i = track.pointsEdgeLeft.size() - 1; i >= 0; i--) {
                //      // 检查是否有足够的元素来进行比较
                //         if (i > 2) {
                //             // 检查第i个点的y值是否大于5
                //             if (track.pointsEdgeRight[i].y <COLSIMAGE - 5) {
                //                 // 删除满足条件的点
                //                 track.pointsEdgeRight.erase(track.pointsEdgeRight.begin() + i);
                //                 // 打印删除点后的索引
                //                 // std::cout << "Deleted point at index: " << i << std::endl;
                //             }
                //         } else {
                //             break;
                //         }
                //      }
                // }

                if(track.pointsEdgeRight[rowBreakRightU].x<ROWSIMAGE/2 )
                {

                    ringStep = RingStep::Finish;
                }
                else
                {
                    pointBreakU = track.pointsEdgeRight[rowBreakRight-5];
                    pointBreakD = track.pointsEdgeLeft[0];
                    std::vector<POINT> Ring_left;
                    //line(track.pointsEdgeLeft, 0, pointBreakU);
                    if(rowBreakLeft&&track.pointsEdgeLeft[rowBreakLeft].x<ROWSIMAGE/4)
                    {
                    
                    Ring_left.push_back(track.pointsEdgeLeft[0]);
                    Ring_left.push_back(track.pointsEdgeLeft[rowBreakLeft]);
                    Ring_left.push_back(pointBreakU);
                    }
                    else{
                        Ring_left.push_back(track.pointsEdgeLeft[0]);
                        Ring_left.push_back(track.pointsEdgeRight[track.pointsEdgeRight.size()-1]);
                    }
                    track.pointsEdgeLeft = Bezier(0.05, Ring_left);
                }
            }
                
        
            
        }
        else if(ringStep == RingStep::Finish)
        {
            track.RingStatus = 0;
            std::cout<<"ring of Finish"<<std::endl;
            if(ringType == RingType::RingLeft)
            {
                uint16_t rowBreakLeft = searchBreakLeftUp(track.pointsEdgeLeft);
                pointBreakU = track.pointsEdgeLeft[rowBreakLeft];
                if(rowBreakLeft >= ROWSIMAGE / 2)
                {
                    counterSpurroad++;
                    pointBreakD = track.pointsEdgeLeft[0];
                    //line(track.pointsEdgeLeft, 0, pointBreakU);
                }
                else if(rowBreakLeft < ROWSIMAGE / 2 && rowBreakLeft > 0)
                {
                    if(counterSpurroad > 1)
                        reset();
                    else
                        counterSpurroad++;
                }
                else if(counterSpurroad && track.spurroad.size() > 0)
                {
                    track.pointsEdgeLeft = track.predictEdgeLeft(track.pointsEdgeRight);
                }
            }
            else if(ringType == RingType::RingRight)
            {
                // std::cout<<"welcome to finish"<<std::endl;
                // uint16_t rowBreakLeft = searchBreakLeftDown(track.pointsEdgeLeft, 0, track.pointsEdgeLeft.size() - 10);
                // uint16_t rowBreakRight = searchBreakRightDown(track.pointsEdgeRight,0,track.pointsEdgeRight.size() - 10);
                // uint16_t rowBreakLeftD = searchBreakLeftDown(track.pointsEdgeLeft,0,ROWSIMAGE/2);
                // uint16_t rowBreakLeftU = searchBreakLeftDown(track.pointsEdgeLeft,rowBreakLeftD + 30, track.pointsEdgeLeft.size());
                
                // uint16_t rowBreakRightD = searchBreakRightDown(track.pointsEdgeRight, 0, ROWSIMAGE/2);
                // uint16_t rowBreakRightU = searchBreakRightDown(track.pointsEdgeRight, rowBreakRightD + 30, track.pointsEdgeRight.size());
                
                // std::cout<<"rowBreakLeft: "<<rowBreakLeft<<std::endl;
                // std::cout<<"rowBreakRight: "<<rowBreakRight<<std::endl;
                // std::cout<<"rowBreakLeftD: "<<rowBreakLeftD<<std::endl;
                // std::cout<<"rowBreakLeftU: "<<rowBreakLeftU<<std::endl;
                // std::cout<<"rowBreakRightD: "<<rowBreakRightD<<std::endl;
                // std::cout<<"rowBreakRightU: "<<rowBreakRightU<<std::endl;

                // vector<POINT> Ring_left;
                // Ring_left.push_back(track.pointsEdgeLeft[0]);
                // Ring_left.push_back(track.pointsEdgeRight[track.pointsEdgeRight.size()-1]);
                // track.pointsEdgeLeft = Bezier(0.05, Ring_left);
                //track.pointsEdgeRight.clear();
                //track.predictEdgeRight(track.pointsEdgeLeft);
                
                uint16_t rowBreakRight = searchBreakRightUp(track.pointsEdgeRight);
                if(rowBreakRight)
                {
                    pointBreakU = track.pointsEdgeRight[rowBreakRight];
                    if(pointBreakU.x >= ROWSIMAGE /2)
                    {
                        reset();
                    }
                    else if(rowBreakRight < ROWSIMAGE / 2 && rowBreakRight > 0)
                    {
                         track.pointsEdgeRight = track.predictEdgeRight(track.pointsEdgeLeft); 
                        if(counterSpurroad >= 2)
                            reset();
                        else
                            counterSpurroad++;
                    }
                    
                }
                else{
                    track.pointsEdgeRight = track.predictEdgeRight(track.pointsEdgeLeft); 
                    reset();
                }
                
            }
        }

        if (ringType == RingType::RingNone)
            return false;
        else
            return true;
    }
    

    void drawImage(Tracking track, Mat &Image)
    {
        // 绘制边缘点
        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(Image, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 2,
                   Scalar(0, 255, 0), -1); // 绿色点
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(Image, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 2,
                   Scalar(0, 255, 255), -1); // 黄色点
        }

        // 绘制岔路点
        for (int i = 0; i < track.spurroad.size(); i++)
        {
            circle(Image, Point(track.spurroad[i].y, track.spurroad[i].x), 3, Scalar(0, 0, 255), -1); // 红色点
        }
        circle(Image, Point(_corner.y, _corner.x), 6, Scalar(0, 0, 255), -1); // 红色点

        // 绘制补线点
        {
            circle(Image, Point(pointBreakU.y, pointBreakU.x), 5, Scalar(255, 0, 255), -1); // 上补线点：粉色
            circle(Image, Point(pointBreakD.y, pointBreakD.x), 5, Scalar(226, 43, 138), -1); // 下补线点：紫色
        }
        
        putText(Image, to_string(ringStep), Point(COLSIMAGE / 2 - 5, ROWSIMAGE - 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 155), 1, CV_AA);
        if(ringType == RingType::RingLeft)
        {
            putText(Image, "Ring L", Point(COLSIMAGE / 2 - 5, 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 1, CV_AA); // 显示赛道识别类型
        }
        else if(ringType == RingType::RingRight)
        {
            putText(Image, "Ring R", Point(COLSIMAGE / 2 - 5, 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 1, CV_AA); // 显示赛道识别类型
        }
        putText(Image, to_string(track.validRowsRight) + " " + to_string(track.stdevRight), Point(COLSIMAGE - 100, ROWSIMAGE - 50),
                FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 1);
        putText(Image, to_string(track.validRowsLeft) + " " + to_string(track.stdevLeft), Point(20, ROWSIMAGE - 50),
                FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 1);
    }

    float get_speed(float motionSpeed)
    {
        if(ringStep == RingStep::Entering)
            return (motionSpeed * 0.8f);
        else
            return motionSpeed;
    }

public:

    /**
     * @brief 环岛类型
     *
     */
    enum RingType
    {
        RingNone = 0, // 未知类型
        RingLeft,     // 左入环岛
        RingRight     // 右入环岛
    };

    /**
     * @brief 环岛运行步骤/阶段
     *
     */
    enum RingStep
    {
        None = 0, // 未知类型
        Entering, // 入环
        Inside,   // 环中
        Exiting,  // 出环
        Finish    // 环任务结束
    };

    RingType ringType = RingType::RingNone; // 环岛类型
    RingStep ringStep = RingStep::None;     // 环岛处理阶段
    bool PointsEdgeboardLeft(Tracking &track){
        int counter=0;
        if(track.pointsEdgeLeft.size()>30){
            for(int i = track.pointsEdgeLeft.size()-1;i>5;i--){
                if(abs(track.pointsEdgeLeft[i].y - track.pointsEdgeLeft[i-2].y )>5){
                    counter++;
                    if(counter>2){
                        counter=0;
                        return true;
                    }
                }
            }
        }
        else{
            return false;
        }
    }
    bool PointsEdgeboardRight(Tracking &track){
        int counter=0;
        if(track.pointsEdgeRight.size()>30){
            for(int i = track.pointsEdgeRight.size()-1;i>5;i--){
                if(abs(track.pointsEdgeRight[i].y - track.pointsEdgeRight[i-2].y) >5){
                    counter++;
                    if(counter>2){
                        counter=0;
                        return true;
                    }
                }
            }
        }
        else{
            return false;
        }
    }
    /**
     * @brief 搜索环岛赛道突变行（左下）
     *
     * @param pointsEdgeLeft
     * @return uint16_t
     */
    uint16_t searchBreakLeftDown(vector<POINT> pointsEdgeLeft, uint16_t row_start, uint16_t row_end)
    {
        bool start = false;
        uint16_t rowBreakLeft = 1;
        uint16_t counter = 0;

        if(row_start == 0)
        {
            row_start++;
        }
        if(row_end > pointsEdgeLeft.size())
        {
            row_end = pointsEdgeLeft.size();
        }
        if(row_start > pointsEdgeLeft.size())
        {
            row_start = pointsEdgeLeft.size();
        }
        for (int i = row_start; i < row_end; i++) // 寻找左边跳变点
        {
            if(pointsEdgeLeft[i].y > pointsEdgeLeft[i - 1].y && start == false)
            {
                start = true;
            }
            if(start)
            {
                if (pointsEdgeLeft[i].y > pointsEdgeLeft[rowBreakLeft].y)
                {
                    rowBreakLeft = i;
                    counter = 0;
                }
                else if (pointsEdgeLeft[i].y <= pointsEdgeLeft[rowBreakLeft].y) // 突变点计数
                {
                    if(row_end > COLSIMAGE / 2 || abs(pointsEdgeLeft[i].y - pointsEdgeLeft[rowBreakLeft].y) > 10 || pointsEdgeLeft[i].y < 3)
                        counter++;
                    if (counter > 5)
                        return rowBreakLeft;
                }
            }
        }

        return 0;
    }

    /**
     * @brief 搜索环岛赛道突变行（右下）
     *
     * @param pointsEdgeRight
     * @return uint16_t
     */
    uint16_t searchBreakRightDown(vector<POINT> pointsEdgeRight, uint16_t row_start, uint16_t row_end)
    {
        bool start = false;
        uint16_t rowBreakRight = 1;
        uint16_t counter = 0;

        if(row_start == 0)
        {
            row_start++;
        }
        if(row_end > pointsEdgeRight.size())
        {
            row_end = pointsEdgeRight.size();
        }
        if(row_start > pointsEdgeRight.size())
        {
            row_start = pointsEdgeRight.size();
        }
        for (int i = row_start; i < row_end; i++) // 寻找右边跳变点
        {
            if(pointsEdgeRight[i].y < pointsEdgeRight[i - 1].y && start == false)
            {
                start = true;
            }
            if(start)
            {
                if (pointsEdgeRight[i].y < pointsEdgeRight[rowBreakRight].y)
                {
                    rowBreakRight = i;
                    counter = 0;
                }
                else if (pointsEdgeRight[i].y >= pointsEdgeRight[rowBreakRight].y) // 突变点计数
                {

                    if(row_end > COLSIMAGE / 2 || abs(pointsEdgeRight[i].y - pointsEdgeRight[rowBreakRight].y) > 10 || pointsEdgeRight[i].y > COLSIMAGE - 3)
                        counter++;
                    if (counter > 5)
                        return rowBreakRight;
                }
            }
        }

        return 0;
    }

    /**
     * @brief 搜索环岛赛道突变行（左上）
     *
     * @param pointsEdgeLeft
     * @return uint16_t
     */
    uint16_t searchBreakLeftUp(vector<POINT> pointsEdgeLeft)
    {
        uint16_t rowBreakLeftUp = pointsEdgeLeft.size() - 1;
        uint16_t counter = 0;
        uint16_t counterFilter = 0;
        for (int i = pointsEdgeLeft.size() - 5; i > 0; i--)
        {
            if (pointsEdgeLeft[i].y > 5 && abs(pointsEdgeLeft[i].y - pointsEdgeLeft[i + 1].y) < 5)
            {
                rowBreakLeftUp = i;
                counter = 0;
                counterFilter++;
            }
            else if (pointsEdgeLeft[i].y <= 5 && abs(pointsEdgeLeft[i].y - pointsEdgeLeft[rowBreakLeftUp].y) > 5 && counterFilter > 5)
            {
                counter++;
                if (counter > 3)
                    return rowBreakLeftUp;
            }
        }

        return 0;
    }

    /**
     * @brief 搜索环岛赛道突变行（右上）
     *
     * @param pointsEdgeRight
     * @return uint16_t
     */
    uint16_t searchBreakRightUp(vector<POINT> pointsEdgeRight)
    {
        uint16_t rowBreakRightUp = pointsEdgeRight.size() - 1;
        uint16_t counter = 0;
        uint16_t counterFilter = 0;
        for (int i = pointsEdgeRight.size() - 5; i > 0; i--)
        {
            if (pointsEdgeRight[i].y < COLSIMAGE - 2 && abs(pointsEdgeRight[i].y - pointsEdgeRight[i + 1].y) < 5)
            {
                rowBreakRightUp = i;
                counter = 0;
                counterFilter++;
            }
            else if (pointsEdgeRight[i].y >= COLSIMAGE - 2 && abs(pointsEdgeRight[i].y - pointsEdgeRight[rowBreakRightUp].y) > 5 && counterFilter > 5)
            {
                counter++;
                if (counter > 3)
                    return rowBreakRightUp;
            }
        }

        return 0;
    }
public:
    uint16_t counterShield = 0; // 环岛检测屏蔽计数器
public:
    POINT _corner;
    uint16_t counterSpurroad = 0; // 岔路计数器
    uint16_t ring_cnt = 0; // 环岛检测确认计数器
	uint16_t counterExit = 0;	  // 异常退出计数器

    POINT pointBreakU;
    POINT pointBreakD;
    vector<POINT> pointsEdgeLeftLast;  // 记录前一场左边缘点集
    vector<POINT> pointsEdgeRightLast; // 记录前一场右边缘点集

};
 
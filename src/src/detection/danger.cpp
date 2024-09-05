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
 * @file danger.cpp
 * @author Leo
 * @brief 危险区AI识别与路径规划
 * @version 0.1
 * @date 2024-01-09
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "../../include/detection.hpp" // Ai模型预测

using namespace std;
using namespace cv;


/**
 * @brief 危险区AI识别与路径规划类
 *
 */

class Danger{  
public:

Danger()
{
  string jsonPath = "../src/config/danger.json";
  std::ifstream config_is(jsonPath);
  if (!config_is.good())
  {
      std::cout << "Error: Params file path:[" << jsonPath
                << "] not find .\n";
      exit(-1);
  }

  nlohmann::json js_value;
  config_is >> js_value;

  try
  {
      params = js_value.get<Params>();
  }
  catch (const nlohmann::detail::exception &e)
  {
      std::cerr << "Json Params Parse failed :" << e.what() << '\n';
      exit(-1);
  }
}

    std::vector<cv::Rect> _danger;
    bool dangerEnable = false;     // 场景检测使能标志
/**
     * @brief 控制器核心参数
     */
    struct Params 
    {
		float k1 = 0.5;
        float k2 = 0.8;
		float k3 = 1;
		uint16_t minArea_Cone = 800;
        uint16_t miny = 0;
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Params, k1, k2, k3, minArea_Cone, miny); // 添加构造函数
    };

    void dangerCheck(vector<PredictResult> predict)
    {
        if(dangerEnable) 
        {
            if(counterExit > 80)
            {
                dangerEnable = false;
                counterExit = 0;
                count_sign = 0;
            }
            return;
        }
          
        for (int i = 0; i < predict.size(); i++) 
        {
          if (predict[i].type == LABEL_BOMB ) 
          {
            counterBomb++;
            break;
          }
        }
        if(counterBomb>0){
          std::cout << "there is a bomb!!!!!!!" << std::endl;
          dangerEnable = true;
          //执行完毕后，将计数器清零
            counterBomb = 0;
        /*使能位需要在执行结束后清零*/
        }
        else
            dangerEnable = false;
    }
    /**
     * @brief 危险区AI识别与路径规划处理
     *
     * @param track 赛道识别结果
     * @param predict AI检测结果
     * @return true
     * @return false
     */
    bool process(Tracking &track, cv::Mat img_rgb, vector<PredictResult> predict)
    {
        enable = false; // 场景检测使能标志
        if (track.pointsEdgeLeft.size() < ROWSIMAGE / 2 || track.pointsEdgeRight.size() < ROWSIMAGE / 2)
            return enable;

        _coneRects = detectCones(img_rgb);
        std::vector<cv::Rect> dangers;

        for (int i = 0; i < predict.size(); i++)
        {
            if (predict[i].type == LABEL_BLOCK) // AI标志
                {
                    cv::Rect rect(predict[i].x, predict[i].y, predict[i].width, predict[i].height);
                     _BlackRects.push_back(rect);
                }
        }

        // 将 _coneRects 的所有元素逐个添加到 dangers 的末尾
        for (const auto& rect : _coneRects) {
            dangers.push_back(rect);
        }

        // 将 _BlackRects 的所有元素逐个添加到 dangers 的末尾
        for (const auto& rect : _BlackRects) {
            dangers.push_back(rect);
        }

        if (dangers.size() <= 0)
        {
            counterExit++;
            if(count_sign  == 1)
            {
                counterExit = counterExit+10;
            }
                
            return enable;
        }
        else
        {
            counterExit = 0;
        }

        // 选取距离最近的障碍
        int index = -1;   // 目标序号
        int Max_y = params.miny;
        for (int i = 0; i < dangers.size(); i++)
        {
            if (dangers[i].y + dangers[i].height>= Max_y)
            {
                index = i;
                Max_y = dangers[i].y + dangers[i].height;
            }
            
        }
       
        enable = true; // 场景检测使能标志
    
        int row = track.pointsEdgeLeft.size() - (dangers[index].y + dangers[index].height/2 - track.rowCutUp);
        if (row < 0 || index == -1) // 无需规划路径
            return enable;

        int disLeft = dangers[index].x + dangers[index].width - track.pointsEdgeLeft[row].y;
        int disRight = track.pointsEdgeRight[row].y - dangers[index].x;
        if (disLeft > 0 && disRight > 0 && disLeft <= disRight) //[1] 障碍物靠左
        {
            if((index + 1 ==  dangers.size()) && _BlackRects.size()>0)
            {
                count_sign = 1;
                curtailTracking(track, false); // 缩减优化车道线（双车道→单车道）
            }
            else
            {
                vector<POINT> points(4); // 三阶贝塞尔曲线
                points[0] = {240, dangers[index].x + dangers[index].width/2 + params.k1 * (dangers[index].y + dangers[index].height)};
                points[1] = {dangers[index].y + dangers[index].height, dangers[index].x + dangers[index].width + params.k2 * (dangers[index].y + dangers[index].height)};
                points[2] = {(dangers[index].y + dangers[index].height + dangers[index].y) / 2, dangers[index].x + dangers[index].width + params.k3 * (dangers[index].y + dangers[index].height)};
                if (dangers[index].y > track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].x)
                    points[3] = track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1];
                else
                    points[3] = {dangers[index].y, dangers[index].x + dangers[index].width};

                track.pointsEdgeLeft.clear(); // 删除错误路线
                vector<POINT> repair = Bezier(0.01, points);  // 重新规划车道线
                for (int i = 0; i < repair.size(); i++)
                        track.pointsEdgeLeft.push_back(repair[i]);
            }
        }
        else if (disLeft > 0 && disRight > 0 && disLeft > disRight) //[2] 障碍物靠右
        {
            if((index + 1 ==  dangers.size()) && _BlackRects.size()>0)
            {
                count_sign = 1;
                curtailTracking(track, true); // 缩减优化车道线（双车道→单车道）
            }
            else
            {
                vector<POINT> points(4); // 三阶贝塞尔曲线
                points[0] = {240, dangers[index].x + dangers[index].width/2 - params.k1 * (dangers[index].y + dangers[index].height)};
                points[1] = {dangers[index].y + dangers[index].height, dangers[index].x - params.k2 * (dangers[index].y + dangers[index].height)};
                points[2] = {(dangers[index].y + dangers[index].height + dangers[index].y) / 2, dangers[index].x - params.k3 * (dangers[index].y + dangers[index].height)};
                if (dangers[index].y > track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x)
                    points[3] = track.pointsEdgeRight[track.pointsEdgeRight.size() - 1];
                else
                    points[3] = {dangers[index].y, dangers[index].x};

                track.pointsEdgeRight.clear(); // 删除错误路线
                vector<POINT> repair = Bezier(0.01, points);   // 重新规划车道线
                for (int i = 0; i < repair.size(); i++)
                        track.pointsEdgeRight.push_back(repair[i]);
            }
           
        }
        _coneRects.clear();
        _BlackRects.clear();

        // 障碍物方向判定（左/右）
      
        return enable;
    }

    /**
     * @brief 图像绘制禁行区识别结果
     *
     * @param img 需要叠加显示的图像
     */
    void drawImage(Mat &img)
    {
        if (enable)
        {
            putText(img, "[2] DANGER - ENABLE", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
        }
    }

private:
    bool enable = false;     // 场景检测使能标志
    uint16_t counterBomb=0; // 炸弹计数器
	std::vector<cv::Rect> _coneRects;       // 传统视觉识别锥桶的方框点集
    std::vector<cv::Rect> _BlackRects;       // 传统视觉识别黑块的方框点集
    uint16_t counterExit = 0;     // 标志结束计数器
    Params params;                   // 读取控制参数
    char count_sign = 0;

    //传统视觉识别锥桶
	std::vector<cv::Rect> detectCones(cv::Mat img_rgb)
	{
		std::vector<cv::Rect> coneRects;
		// 设置锥桶颜色的RGB范围（黄色）
		cv::Scalar lowerYellow(0, 100, 100);
		cv::Scalar upperYellow(100, 255, 255);

		// 在RGB图像中根据颜色范围提取锥桶区域
		cv::Mat mask;
		cv::inRange(img_rgb, lowerYellow, upperYellow, mask);

		// 进行形态学操作，去除噪声并提取锥桶区域的轮廓
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
		cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

		std::vector<std::vector<cv::Point>> contours;
		cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours)      // 绘制正方形框选中锥桶区域
        {
            double contourArea = cv::contourArea(contour);
            cv::Rect boundingRect = cv::boundingRect(contour);
            if (contourArea > params.minArea_Cone) // 自定义的面积阈值
            { 
                coneRects.push_back(boundingRect);
            }
        }
		return coneRects;
	}
  

    /**
     * @brief 缩减优化车道线（双车道→单车道）
     *
     * @param track
     * @param left
     */
    void curtailTracking(Tracking &track, bool left)
    {
        if (left) // 向左侧缩进
        {
            if (track.pointsEdgeRight.size() > track.pointsEdgeLeft.size())
                track.pointsEdgeRight.resize(track.pointsEdgeLeft.size());
            // for (int i = 0; i < track.pointsEdgeRight.size(); i++)
            // {
            //     track.pointsEdgeRight[i].x = dangers[index].x;
            // }

            for (int i = 0; i < track.pointsEdgeRight.size(); i++)
            {
                track.pointsEdgeRight[i].y = (track.pointsEdgeRight[i].y + track.pointsEdgeLeft[i].y) / 2;
            }
        }
        else // 向右侧缩进
        {
            if (track.pointsEdgeRight.size() < track.pointsEdgeLeft.size())
                track.pointsEdgeLeft.resize(track.pointsEdgeRight.size());

            for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
            {
                track.pointsEdgeLeft[i].y = (track.pointsEdgeRight[i].y + track.pointsEdgeLeft[i].y) / 2;
            }
        }
    }
};

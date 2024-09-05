/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo;
 *https://bjsstech.com 版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial
 *transactions(开源学习,请勿商用). The code ADAPTS the corresponding hardware
 *circuit board(代码适配百度Edgeboard-智能汽车赛事版), The specific details
 *consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file rescue.cpp
 * @author Leo
 * @brief 救援区检测
 * @version 0.1
 * @date 2024-01-11
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "../../include/common.hpp"
#include "../../include/detection.hpp"
#include "../mapping.cpp"
#include "../recognition/tracking.cpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../include/json.hpp"

using namespace cv;
using namespace std;

class Rescue {
public:

Rescue()
{
  string jsonPath = "../src/config/rescue.json";
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
  bool carStoping = false;  // 停车标志
  bool carExitting = false; // 出库标志
  bool rescueEnable = false;      // 使能标志
  enum Step {
    None = 0, // AI检测
    Enable,   // 使能（标志识别成功）
    Enter,    // 进站
    Cruise,   // 巡航
    Stop,     // 停车
    Exit      // 出站
  };


  /**
     * @brief 控制器核心参数
     */
    struct Params 
    {
		double DangerClose = 175;       // 智能车危险距离
		uint16_t ServoRow = 75;
    uint16_t ServoRow2 = 30;
    uint16_t ServoRow3 = 30;
		uint16_t ServoValue = 87;
		uint16_t DelayCnt = 3;
		uint16_t BrakeCnt = 1;
    uint16_t Backsize = 100;
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Params, DangerClose, ServoRow, ServoRow2, ServoRow3, ServoValue, DelayCnt, BrakeCnt,Backsize); // 添加构造函数
    };

  Step step = Step::None;
  bool entryLeft = true; // 左入库使能标志
  /**
   * @brief 检测初始化
   *
   */
  void reset(void) {
    carStoping = false;
    carExitting = false;
    step = Step::None;
    counterSession = 0;         // 图像场次计数器
    counterRec = 0;             // 标志检测计数器
    lastPointsEdgeLeft.clear(); // 记录上一场边缘点集（丢失边）
    lastPointsEdgeRight.clear();
    counterExit = 0;
    counterImmunity = 0;
  }

  void rescueCheck(vector<PredictResult> predict)
  {
    if(rescueEnable)
      return;
      
    if ((counterImmunity > 200 && again) ||
          (counterImmunity > 30 && !again)) {
        for (int i = 0; i < predict.size(); i++) {
          if (predict[i].type == LABEL_TUMBLE ||
              predict[i].type == LABEL_PATIENT) // 伤员平民标志检测
          {
            counterRec++;
            break;
          }
        }
        for (int i = 0; i < predict.size(); i++) {
          if (predict[i].type == LABEL_EVIL ||
              predict[i].type == LABEL_THIEF) // 劫匪标志检测
          {
            counterExit++;
            break;
          }
        }

        if (counterRec || counterExit) {
          counterSession++;
          if (counterRec > 2 && counterSession <= 8) {
            std::cout << "entryLeft" << std::endl;
            step = Step::Enable; // 使能
            entryLeft = true;
            counterRec = 0;
            counterExit = 0;
            counterSession = 0;
            this->rescueEnable = true; // 使能
          } else if (counterExit > 2 && counterSession <= 8) {
            std::cout<<"entryright"<<std::endl;
            step = Step::Enable; // 使能
            
            entryLeft = false;
            counterRec = 0;
            counterExit = 0;
            counterSession = 0;
            this->rescueEnable = true; // 使能
          } else if (counterSession > 8) {
            counterRec = 0;
            counterSession = 0;
          }

        }
      } else
        counterImmunity++;
      
  }
  /**
   * @brief 检测与路径规划
   *
   * @param track 赛道识别结果
   * @param detection AI检测结果
   */
  

bool process(Tracking &track, vector<PredictResult> predict,cv::Mat img_rgb) {
    _pointNearCone = POINT(0, 0);
    _distance = 0;
    pointEdgeDet.clear();
    _coneRects.clear();
    _bezier_input.clear();

    switch (step) {
    case Step::Enable: //[02] 使能
    {
      counterExit++;
      if (counterExit > 500) // 超时退出
      {
        reset();
        return false;
      }
      _coneRects = detectCones(img_rgb);
			searchCones(_coneRects, track.rowCutUp);//30
       if (entryLeft)        // 左入库
       {
          _pointNearCone = searchNearestCone(track.pointsEdgeLeft, pointEdgeDet);   // 搜索左下锥桶
       }
       else
       {
          _pointNearCone = searchNearestCone(track.pointsEdgeRight, pointEdgeDet);  // 搜索右下锥桶
       }
       
      switch(indexDebug)
      {
        case 0:
        if (_pointNearCone.x > params.ServoRow && _pointNearCone.x < params.ServoRow + ROWSIMAGE / 3
            && _pointNearCone.y != 0) // 当车辆开始靠近右边锥桶：准备入库
            {
              counterSession ++;
              if(counterSession > params.DelayCnt)
              {
                  indexDebug = 1;
                  counterSession = 0;
              }
            }
       break;    
        case 1:
        {
          if (_pointNearCone.x > params.ServoRow2 && _pointNearCone.x < params.ServoRow3 && _pointNearCone.y != 0) // 当车辆开始靠近右边锥桶：准备入库
          {
              counterSession ++;
              if(counterSession > params.DelayCnt)
              {
                  indexDebug = 2;
                  counterSession = 0;
              }
          }
          break;
        }
        case 2:
        {
          step = Step::Enter; //使能
          indexDebug = 0;
          break;
        }
      }
       break;
    }
    case Step::Enter: //[03] 入库使能
    {
      _coneRects = detectCones(img_rgb);
      searchCones(_coneRects, track.rowCutUp);
      _distance = 0;
			_pointNearCone = searchClosestCone(pointEdgeDet);
			if(_distance < params.DangerClose && _distance > 0)
			{
				counterRec++;
				if(counterRec > 1)
				{
					step = Step::Cruise; // 巡航使能
					counterRec = 0;
				}
			}
			if(entryLeft)
			{
				POINT start = POINT(ROWSIMAGE - 1, COLSIMAGE - 1);
				POINT end = POINT(ROWSIMAGE / 2 - params.ServoValue, 0);
				POINT middle = POINT((start.x + end.x) * 0.4, (start.y + end.y) * 0.6);
				vector<POINT> input = {start, middle, end};
				track.pointsEdgeRight = Bezier(0.05, input); // 补线
				track.pointsEdgeLeft = predictEdgeLeft(track.pointsEdgeRight); // 由右边缘补偿左边缘
    
			}
			else
			{
				POINT start = POINT(ROWSIMAGE - 1, 0);
				POINT end = POINT(ROWSIMAGE / 2 - params.ServoValue, COLSIMAGE - 1);
				POINT middle = POINT((start.x + end.x) * 0.4, (start.y + end.y) * 0.6);
				vector<POINT> input = {start, middle, end};
				track.pointsEdgeLeft = Bezier(0.05, input); // 补线
				track.pointsEdgeRight = predictEdgeRight(track.pointsEdgeLeft); // 由左边缘补偿右边缘
 			}
			pathsEdgeLeft.push_back(track.pointsEdgeLeft); // 记录进厂轨迹
			pathsEdgeRight.push_back(track.pointsEdgeRight);

			break;
    }
     case Step::Cruise: //[04] 巡航使能
    {
			step = Step::Stop; // 停车使能
			track.pointsEdgeLeft = pathsEdgeLeft[pathsEdgeLeft.size() - 1];//维持入库最后的打角
			track.pointsEdgeRight = pathsEdgeRight[pathsEdgeRight.size() - 1];
      break;

    }

    case Step::Stop: //[05] 停车使能
    {
      carStoping = true;
      counterRec++;
      if (counterRec > params.BrakeCnt) // 停车
      {
        carStoping = false;
        carExitting = true;
        step = Step::Exit; // 出站使能
        counterRec = 0;
      }
      track.pointsEdgeLeft = pathsEdgeLeft[pathsEdgeLeft.size() - 1];//维持入库最后的打角
			track.pointsEdgeRight = pathsEdgeRight[pathsEdgeRight.size() - 1];
      break;
    }

   case Step::Exit: //[06] 出站使能
    {
      carExitting = true;
      if (pathsEdgeLeft.size() < 2 || pathsEdgeRight.size() < 2) {
        {
          if(track.pointsEdgeLeft.size()> params.Backsize && track.pointsEdgeRight.size()> params.Backsize)
          {
            step = Step::None; // 出站完成
            carExitting = false;
            again = true; // 第二次进入救援区标志
            reset();
            rescueEnable=false;
          }
          else
          {
            track.pointsEdgeLeft = pathsEdgeLeft[0];
            track.pointsEdgeRight = pathsEdgeRight[0];
          }
        }
        
      } else {
        track.pointsEdgeLeft = pathsEdgeLeft[pathsEdgeLeft.size() - 1];
        track.pointsEdgeRight = pathsEdgeRight[pathsEdgeRight.size() - 1];
        pathsEdgeLeft.pop_back();
        pathsEdgeRight.pop_back();
      }
      break;
    }
    default : break;
  }

    if (step == Step::None) // 返回控制模式标志
      return false;
    else
      return true;
  }

////////////////////////////////////////////////////////////////



  /**
   * @brief 识别结果图像绘制
   *
   */
   
void drawImage(Tracking track, Mat &image) {
    // 赛道边缘
    for (int i = track.pointsEdgeLeft.size()*0.1; i < track.pointsEdgeLeft.size()*0.8; i++) {
      circle(image, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x),
             1, Scalar(0, 255, 0), -1); // 绿色点
    }
    for (int i = 0; i < track.pointsEdgeRight.size(); i++) {
      circle(image,
             Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1,
             Scalar(0, 255, 255), -1); // 黄色点
    }

    // 入库状态
    string state = "None";
    switch (step) {
    case Step::Enable:
      state = "Enable";
      break;
    case Step::Enter:
      state = "Enter";
      break;
    case Step::Cruise:
      state = "Cruise";
      break;
    case Step::Stop:
      state = "Stop";
      break;
    case Step::Exit:
      state = "Exit";
      break;
    }
    // 绘制锥桶坐标
		for (int i = 0; i < pointEdgeDet.size(); i++)
		{
            putText(image, to_string(i+1), Point(pointEdgeDet[i].y, pointEdgeDet[i].x), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1, CV_AA);
		}

    if (entryLeft) {
     
      putText(image, "[3] RESCUE - LEFT", Point(COLSIMAGE / 2 - 30, 10),
              cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    } else {
      putText(image, "[3] RESCUE - RIGHT", Point(COLSIMAGE / 2 - 30, 10),
              cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    }

    putText(image, state, Point(COLSIMAGE / 2 - 10, 30),
            cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);

    putText(image, to_string(_distance), Point(COLSIMAGE / 2 - 30, 40),
            cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
            CV_AA); // 显示锥桶距离
    if (_pointNearCone.y > 0)
    {
        putText(image, to_string(_pointNearCone.y), Point(COLSIMAGE / 2 - 30, 120),
        cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
        CV_AA); 
        putText(image, to_string(_pointNearCone.x), Point(COLSIMAGE / 2, 120),
        cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
        CV_AA); 
        circle(image, Point(_pointNearCone.y, _pointNearCone.x), 5,
        Scalar(200, 200, 200), -1);

    }
      
    putText(image, to_string(indexDebug),
            Point(COLSIMAGE / 2 - 10, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX,
            0.3, cv::Scalar(0, 0, 255), 1, CV_AA);
  }
 private:
  bool again = false; // 第二次进入救援区标志
  double _distance = 0;
  Params params;                   // 读取控制参数
  POINT _pointNearCone;
  std::vector<POINT> pointEdgeDet;        // 锥桶检测边缘点集
	std::vector<cv::Rect> _coneRects;       // 传统视觉识别锥桶的方框点集
  std::vector<POINT> lastPointsEdgeLeft;  // 记录上一场边缘点集（丢失边）
  std::vector<POINT> lastPointsEdgeRight; // 记录上一场边缘点集（丢失边）
  std::vector<POINT> _bezier_input;
  vector<vector<POINT>> pathsEdgeLeft; // 记录入库路径
  vector<vector<POINT>> pathsEdgeRight;
  uint16_t counterSession = 0;  // 图像场次计数器
  uint16_t counterRec = 0;      // 标志检测计数器
  uint16_t counterExit = 0;     // 标志结束计数器
  uint16_t counterImmunity = 0; // 屏蔽计数器
  Mapping ipm = Mapping(Size(COLSIMAGE, ROWSIMAGE), Size(COLSIMAGEIPM, ROWSIMAGEIPM));
  int indexDebug = 0;

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

		// 绘制正方形框选中锥桶区域
		for (const auto& contour : contours)
		{
			cv::Rect boundingRect = cv::boundingRect(contour);
			coneRects.push_back(boundingRect);
		}	
		return coneRects;
	}

  /**
	 * @brief 从视觉结果中检索锥桶坐标集合
	 *
	 * @param predict 检测结果
	 * @param rowCutUp 滤除掉图片最上方部分的色块
	 * @return vector<POINT>
	 */
	void searchCones(vector<Rect> predict, uint16_t rowCutUp = 0)
	{
		pointEdgeDet.clear();
		for (int i = 0; i < predict.size(); i++)
		{
			if(predict[i].y + predict[i].height / 2 > rowCutUp)
				pointEdgeDet.push_back(POINT(predict[i].y + predict[i].height / 2,
												predict[i].x + predict[i].width / 2));
		}
	}

  /**
	 * @brief 搜索距离赛道左边缘锥桶坐标(右下，非常规意义上的)
	 *
	 * @param pointsEdgeLeft 赛道边缘点集
	 * @param predict AI检测结果
	 * @return POINT
	 */
	POINT searchNearestCone(vector<POINT> pointsEdgeLeft,
							vector<POINT> pointsCone)
	{
		POINT point(0, 0);
		double disMin = 80; // 右边缘锥桶离赛道左边缘最小距离
    _distance = 80;
		if (pointsCone.size() <= 0 || pointsEdgeLeft.size() < 10)
			return point;

		POINT a = pointsEdgeLeft[pointsEdgeLeft.size() * 0.1];
		POINT b = pointsEdgeLeft[pointsEdgeLeft.size() * 0.8];

		for (int i = 0; i < pointsCone.size(); i++)
		{
			double dis = distanceForPoint2Line(a, b, pointsCone[i]);
			if (dis < disMin && dis < _distance)
			{
				point = pointsCone[i];
				_distance = dis;
			}
		}

		return point;
	}

  /**
	 * @brief 搜索距离车最近的锥桶
	 *
	 * @param pointsEdgeLeft 赛道边缘点集
	 * @param predict AI检测结果
	 * @return POINT
	 */
	POINT searchClosestCone(vector<POINT> pointsCone)
	{
		POINT closestCone(0, 0);
		if (pointsCone.size() <= 0)
			return closestCone;

		POINT carCenter(ROWSIMAGE - 1, COLSIMAGE / 2);
		double closestLen = 0;
		closestCone = pointsCone[0];
		closestLen = distance(closestCone, carCenter);

		for (int i = 1; i < pointsCone.size(); i++)
		{
			double len = distance(pointsCone[i], carCenter);
			if(len < closestLen)
			{
				closestLen = len;
				closestCone = pointsCone[i];
			}
		}
		_distance = closestLen;

		return closestCone;
	}



  /**
   * @brief 在俯视域由左边缘预测右边缘
   *
   * @param pointsEdgeLeft
   * @return vector<POINT>
   */
  vector<POINT> predictEdgeRight(vector<POINT> &pointsEdgeLeft) {
    int offset = 120; // 右边缘平移尺度
    vector<POINT> pointsEdgeRight;
    if (pointsEdgeLeft.size() < 3)
      return pointsEdgeRight;

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
    POINT midPoint = POINT(middleIipm.y, middleIipm.x);   // 补线中点

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

    for (int i = 0; i < repair.size(); i++) {
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
  vector<POINT> predictEdgeLeft(vector<POINT> &pointsEdgeRight) {
    int offset = 120; // 右边缘平移尺度
    vector<POINT> pointsEdgeLeft;
    if (pointsEdgeRight.size() < 3)
      return pointsEdgeLeft;

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
    POINT midPoint = POINT(middleIipm.y, middleIipm.x);  // 补线中点

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

    for (int i = 0; i < repair.size(); i++) {
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
  //两点之间的像素距离
	double distance(POINT x1, POINT x2)
	{
		double dx = x1.x - x2.x;
		double dy = x1.y - x2.y;
		double len = std::sqrt(dx * dx + dy * dy);
		return len;
	}
};



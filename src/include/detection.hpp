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
 * @file detection.hpp
 * @author Leo
 * @brief 基于Paddle飞桨，Edgeboard板卡部署的AI目标检测框架
 * @version 0.1
 * @date 2024-01-12
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "predictor_api.h"
#include <onnxruntime_cxx_api.h>
#include <sys/time.h>
#include <cstdio>
#include <functional>
#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <vector>
#include <chrono>
#include <unordered_map>
#include <cstdlib>
#include <memory>
#include <stdlib.h>
#include "predictor.hpp"
#include "common.hpp"


#include "capture.hpp"
#include "stop_watch.hpp"
#include <condition_variable>
#include <mutex>
#include <thread>
#include "detection/bridge.cpp"      //AI检测：坡道区
#include "detection/danger.cpp"      //AI检测：危险区
#include "detection/parking.cpp"     //AI检测：停车区
#include "detection/racing.cpp"      //AI检测：追逐区
#include "detection/rescue.cpp"      //AI检测：救援区


Bridge bridge;            // 坡道区检测类
Parking parking;          // 停车区检测类
Danger danger;            // 危险区检测类
Rescue rescue;            // 救援区检测类
Racing racing;            // 追逐区检测类
/**
 * @brief 目标检测结果
 *
 */

struct DetectionResult
{
    cv::Mat rgb_frame;
    std::vector<PredictResult> predictor_results;
};
class Detection
{
public:
    bool Startdetect = false;
public:
    std::vector<PredictResult> results; // AI推理结果
    float score = 0.5;                  // AI检测置信度


     Detection() {}
    ~Detection() {}

    int init(std::string model_config_path)
    {
        return _init(model_config_path);
    }

    void Start()
    {
        _loop = true;
        inference();
    }

    void Stop()
    {
        _loop = false;
        if(_thread->joinable())
            _thread->join();
        std::cout << "ai exit" << std::endl;
    }

    bool AI_Enable()
    {
        return AI_Captured;
    }
    /**
     * @brief Construct a new Detection object
     *
     * @param pathModel
     */
    //Detection()
    //{
        /*
        // 模型初始化
        this->predictor_nna_ = std::make_shared<PPNCPredictor>("../src/config/config_ppncnna.json");
        this->predictor_nms_ = std::make_shared<PPNCPredictor>("../src/config/config_ppncnms.json");
        this->onnx_env_ = Ort::Env(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING, "test");
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(8);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        std::string onnx_model = pathModel + "/post.onnx";
        this->predictor_onnx_ = std::make_shared<Ort::Session>(this->onnx_env_, onnx_model.c_str(), session_options);

        // ONNX模型加载
        this->onnx_input_names_.first.push_back("im_shape");
        this->onnx_input_names_.first.push_back("scale_factor");
        std::string io_paddle = pathModel + "/io_paddle.json";
        // read io_paddle
        std::ifstream ifs(io_paddle);
        nlohmann::json j;
        ifs >> j;
        ifs.close();
        for (size_t i = 0; i < j.size(); ++i)
        {
            if (j[i]["type"] == "OUTPUT")
            {
                assert(j[i]["shape"].size() == 4);
                this->onnx_input_names_.first.push_back(j[i]["name"]);
            }
            if (j[i]["type"] == "post_out")
            {
                this->onnx_out_names_.first.push_back(j[i]["name"]);
            }
        }

        for (auto &s : this->onnx_input_names_.first)
        {
            this->onnx_input_names_.second.push_back(s.c_str());
        }

        for (auto &s : this->onnx_out_names_.first)
        {
            this->onnx_out_names_.second.push_back(s.c_str());
        }
        buildNms(pathModel); // 编译生成.so文件

        this->predictor_nna_->load();
        this->predictor_nms_->load();

        // 模型标签加载
        std::string pathLabels = pathModel + "/label_list.txt";
        labels.clear();
        std::ifstream file(pathLabels);
        if (file.is_open())
        {
            std::string line;
            while (getline(file, line))
            {
                labels.push_back(line);
            }
            file.close();
        }
        else
        {
            std::cout << "Open Lable File failed: " << pathLabels << std::endl;
        }
        */
    //};

    /**
     * @brief AI模型推理
     *
     */
    void inference( )
    {
        
        _thread = std::make_unique<std::thread>([this]() {
            while (_loop)
            {
                std::shared_ptr<DetectionResult> result = std::make_shared<DetectionResult>();

                std::unique_lock<std::mutex> lock(_mutex);

                while (_frame == nullptr)
                {
                    // 设置超时时间
                    if (cond_.wait_for(lock, std::chrono::seconds(1)) == std::cv_status::timeout) {
                        //std::cout << "ai wait for frame time out" << std::endl;
                        break;
                    }
                }
                if(_frame != nullptr)
                {
                    result->rgb_frame = _frame->clone();
                    _frame = nullptr;
                    lock.unlock();
                }
                else 
                    continue;

                //ai推理
                if(Startdetect)
                {
                    auto feeds = _predictor->preprocess(result->rgb_frame, {320, 320});
                    _predictor->run(*feeds);
                    _predictor->render();
                    rescue.rescueCheck(_predictor->results);
                    danger.dangerCheck(_predictor->results);
                    parking.parkingCheck(_predictor->results);
                    /*
                    bridgeDetection.bridgeCheck(_predictor->results);
                    slowZoneDetection.slowZoneCheck(_predictor->results);
                    depotDetection.depotDetection(_predictor->results);
                    farmlandAvoidance.farmdlandCheck(_predictor->results);
                    granaryDetection.granaryCheck(_predictor->results);
                    */
                }
                else
                {
                    _Cnt = 0;
                    AI_Captured = false;
                }

                bool flag = false;
                for(int i = 0; i < _predictor->results.size(); i++)
                {
                    std::string label_name = _predictor->results[i].label;
                    if((label_name == "tumble" ||label_name == "thief" ||label_name == "spy" ||label_name == "prop" ||
                        label_name == "patient" ||label_name == "block" ||label_name == "evil" ||
                        label_name == "danger" ||label_name == "safety" ||label_name == "bridge" 
                    ||label_name == "bomb" || label_name == "cone" || label_name == "danger") && _predictor->results[i].score > 0.52
                        && _predictor->results[i].y + _predictor->results[i].height / 2 > 20)
                    {
                        flag = true;
                        break;
                    }
                }
                if(_Cnt == 0)
                    AI_Captured = flag;
                if(AI_Captured)
                {
                    _Cnt++;
                    if(_Cnt > 2)
                        _Cnt = 0;
                }
            
                //数据传递
                result->predictor_results = _predictor->results;
                std::unique_lock<std::mutex> lock2(_mutex);
                _lastResult = result;
                cond_.notify_all();
            }
        });
    }

    std::shared_ptr<DetectionResult> getLastFrame()
    {
        std::shared_ptr<DetectionResult> ret = nullptr;
        {
            std::unique_lock<std::mutex> lock(_mutex);

            while (_lastResult == nullptr)
            {
                cond_.wait(lock);
            }
            ret = _lastResult;
            _lastResult = nullptr;
        }
        return ret;
    }

    void setFrame(cv::Mat img)
    {
        std::unique_lock<std::mutex> lock(_mutex);
        _frame = std::make_shared<cv::Mat>(img.clone());
        //std::cout<<" detection set frame"<<std::endl;
        cond_.notify_all();
    }
    
    void build_nms(const std::string &model_dir) 
    {
        std::string model_file = model_dir + "/nms.tar";
        std::string untar_cmd = "tar -xf " + model_file + " -C . --no-same-owner";
        std::string final_file = model_file + ".so";
        std::string cc_cmd = "g++ -shared -fPIC -o " + final_file + " lib0.o devc.o";
        int sys_status = 0;

        sys_status = system(untar_cmd.c_str());
        if (sys_status) {
            std::cout << "Error: cannot untar file " << model_file << std::endl;
            exit(-1);
        }

        // create shared
        sys_status = system(cc_cmd.c_str());
        if (sys_status) {
            std::cout << "Error: compile for " << model_file << std::endl;
            exit(-1);
        }
        std::cout << "compile done." << std::endl;
    }

  
    void drawBox(Mat &img)
    {
        for (int i = 0; i < results.size(); i++)
        {
            PredictResult result = results[i];

            auto score = std::to_string(result.score);
            int pointY = result.y - 20;
            if (pointY < 0)
                pointY = 0;
            cv::Rect rectText(result.x, pointY, result.width, 20);
            cv::rectangle(img, rectText, this->getCvcolor(result.type), -1);
            std::string label_name = result.label + " [" + score.substr(0, score.find(".") + 3) + "]";
            cv::Rect rect(result.x, result.y, result.width, result.height);
            cv::rectangle(img, rect, this->getCvcolor(result.type), 1);
            cv::putText(img, label_name, Point(result.x, result.y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 254), 1);
        }
    }

    
    /**
     * @brief 获取Opencv颜色
     *
     * @param index 序号
     * @return cv::Scalar
     */
    cv::Scalar getCvcolor(int index)
    {
        switch (index)
        {
        case 0:
            return cv::Scalar(0, 255, 0); // 绿
            break;
        case 1:
            return cv::Scalar(255, 255, 0); // 天空蓝
            break;
        case 2:
            return cv::Scalar(0, 0, 255); // 大红
            break;
        case 3:
            return cv::Scalar(0, 250, 250); // 大黄
            break;
        case 4:
            return cv::Scalar(250, 0, 250); // 粉色
            break;
        case 5:
            return cv::Scalar(0, 102, 255); // 橙黄
            break;
        case 6:
            return cv::Scalar(255, 0, 0); // 深蓝
            break;
        case 7:
            return cv::Scalar(255, 255, 255); // 大白
            break;
        case 8:
            return cv::Scalar(247, 43, 113);
            break;
        case 9:
            return cv::Scalar(40, 241, 245);
            break;
        case 10:
            return cv::Scalar(237, 226, 19);
            break;
        case 11:
            return cv::Scalar(245, 117, 233);
            break;
        case 12:
            return cv::Scalar(55, 13, 19);
            break;
        case 13:
            return cv::Scalar(255, 255, 255);
            break;
        case 14:
            return cv::Scalar(237, 226, 19);
            break;
        case 15:
            return cv::Scalar(0, 255, 0);
            break;
        default:
            return cv::Scalar(255, 0, 0);
            break;
        }
    }

   
public:
    int _init(std::string model_config_path)
    {
        _predictor = std::make_shared<PPNCDetection>();
        if (_predictor == nullptr)
        {
            std::cout << "Predictor Create failed." << std::endl;
            return -1;
        }
        int ret = _predictor->init(model_config_path);
        if (ret != 0)
        {
            return -1;
        }
        return 0;
    }
    bool _loop = false;
    std::shared_ptr<PPNCDetection> _predictor;
    std::shared_ptr<DetectionResult> _lastResult;
    std::shared_ptr<cv::Mat> _frame;
    
    uint16_t _Cnt = 0;//计数器
    bool AI_Captured = false;

    std::mutex _mutex;
    std::condition_variable cond_;
    std::unique_ptr<std::thread> _thread;
};

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
 * @file motion.cpp
 * @author Leo
 * @brief 运动控制器：PD姿态控制||速度控制
 * @version 0.1
 * @date 2023-12-26
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include "../include/common.hpp"
#include "../include/json.hpp"
#include "../include/fruzzy_pid.hpp"
#include "fruzzy_pid.cpp"
#include "controlcenter.cpp"

using namespace std;
using namespace cv;

#define NB -3
#define NM -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3

//kp_m为动态的Kp最大值，kd_m为动态的最大Kd值

float KP_Fuzzy(float E,float EC);
float Kd_Fuzzy(float EC);

const float M_PI_f = 3.14159265358979323846f;
/**
 * @brief 运动控制器
 *
 */
class Motion
{
private:
    int countShift = 0; // 变速计数器

public:
    
    int Index[6]={1,4,8,12,16,18};
    
public:
    /**
     * @brief 初始化：加载配置文件
     *
     */
    Motion()
    {
        string jsonPath = "../src/config/config.json";
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

        speed = params.speedLow;
        cout << "--- speedLow:" << params.speedLow << "m/s  |  speedHigh:" << params.speedHigh << "m/s" << endl;
    };

    /**
     * @brief 控制器核心参数
     *
     */
    struct Params
    {
        float speedLow = 0.8;                              // 智能车最低速
        float speedHigh = 0.8;                             // 智能车最高速
        float speedBridge = 0.6;                           // 坡道速度
        float speedDown = 0.5;                             // 特殊区域降速速度
        float speedDanger = 0.1;
        float speedRing = 0.3;                      
        float runP1 = 0.9;                                 // 一阶比例系数：直线控制量
        float runP2 = 0.018;                               // 二阶比例系数：弯道控制量
        float runP3 = 0.0;                                 // 三阶比例系数：弯道控制量
        float turnP = 3.5;                                 // 一阶比例系数：转弯控制量
        float turnD1 = 3.5;                                 // 一阶微分系数：转弯控制量
        float turnD2 = 3.5;
        float turnD3 = 3.5;                                 // 一阶微分系数：转弯控制量
        float sigma_mid = 60;
        float sigma_k = 0.5;

        float KP0 = 2.0;
        float KP1 = 2.0;
        float KP2 = 2.0;
        float KP3 = 2.0;
        float KP4 = 2.0;
        float KP5 = 2.0;
        float KP6 = 2.0;
        
        float KD0 = 2.0;
        float KD1 = 2.0;
        float KD2 = 2.0;
        float KD3 = 2.0;
        float KD4 = 2.0;
        float KD5 = 2.0;
        float KD6 = 2.0;

        float kp_m = 3.2;
        float kd_m = 20.0;

        uint16_t angle = 35;
        bool debug = false;                                // 调试模式使能
        bool saveImg = false;                              // 存图使能
        uint16_t rowCutUp = 10;                            // 图像顶部切行
        uint16_t rowCutBottom = 10;                        // 图像顶部切行
        bool bridge = true;                                // 坡道区使能
        bool danger = true;                                // 危险区使能
        bool rescue = true;                                // 救援区使能
        bool racing = true;                                // 追逐区使能
        bool parking = true;                               // 停车区使能
        bool ring = true;                                  // 环岛使能
        bool cross = true;                                 // 十字道路使能
        float score = 0.5;                                 // AI检测置信度
        float e_max = 320;
        float de_max = 240;
        float kp_max = 3.0;
        float ki_max = 0.0001;
        float kd_max = 20;
        float Kp0 = 2.00;
        float Ki0 = 0.00001;
        float Kd0 = 10;

        float Gaussian = 0.0;
        bool lvbo = true;
        bool lvbo1 = true;
        bool lvbo2 = true;
        string model = "../res/model/yolov3_mobilenet_v1"; // 模型路径
        string video = "../res/samples/demo.mp4";          // 视频路径
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Params, speedLow, speedHigh, speedBridge, speedDown,speedDanger, speedRing,
                                        runP1, runP2, runP3,
                                       turnP, turnD1,turnD2,turnD3,sigma_mid,sigma_k,angle, debug, saveImg, rowCutUp, rowCutBottom, bridge, danger,
                                       rescue, racing, parking, ring, cross, score, model, video,e_max,
                                       de_max,kp_max,ki_max,kd_max,Kp0,Ki0,Kd0,kp_m,kd_m,KP0,
                                       KP1,KP2,KP3,KP4,KP5,KP6,KD0,KD1,KD2,KD3,KD4,KD5,KD6,Gaussian,lvbo,lvbo1,lvbo2); // 添加构造函数
    };

    Params params;                   // 读取控制参数
    uint16_t servoPwm = PWMSERVOMID; // 发送给舵机的PWM
    float speed = 0.3;               // 发送给电机的速度
    int i = 0;
    float lastactual = 0;
    
    
    
    /**
     * @brief 姿态PD控制器
     *
     * @param controlCenter 智能车控制中心
     */
    void poseCtrl(int controlCenter,ControlCenter control,Tracking &track ,Scene scene = Scene::NormalScene)
    {
        static unsigned int index = 0;
        static int count= 0;
        float E=0,EC=0;
        static float errorLast = 0;          // 记录前一次的偏差
        float sigmaValue = abs(control.sigmaCenter);
        float Kp,Kd,output;
        E  = controlCenter - COLSIMAGE / 2; // 图像控制中心转换偏差
        EC = E - errorLast;

        // /*********误差补偿*********** */

        // if (abs(E) > 40 && abs(E) < 100) {
        //     int temp;
        //     temp = E;
        //     while(abs(E) < Index[index]) {
        //         index--;
        //     }
        //     if (E > 0) {
        //         E -= Index[index];
        //         index++;
        //     } else if (E < 0) {
        //         E += Index[index];
        //         index++;
        //     }
            
        //     cout << "E is " << E << "orginal E is " << temp <<" Index"<<Index[index-1]<< endl;
        // } else {
        //     count++;
        //     index = 1;
        //     if (count >= 2) {
        //         index = 0;
        //     }
        // }
        // if(index>5) index = 5;

        if(scene == Scene::RingScene&&track.RingStatus== 2){
            std::cout<<"PID of RingRight"<<std::endl;

            output = (E * params.runP2) + (EC) * params.turnD2;
            int pwmDiff = (int)(output);
            errorLast = E;
            servoPwm = (uint16_t)(PWMSERVOMID + pwmDiff); // PWM转换
            return;
        }else if(scene == Scene::RingScene&&track.RingStatus== 1){
            std::cout<<"PID of RingLeft"<<std::endl;
            output = (E * params.runP3) + (EC) * params.turnD3;
            int pwmDiff = (int)(output);
            errorLast = E;
            servoPwm = (uint16_t)(PWMSERVOMID + pwmDiff); // PWM转换
            return;
        }
        Kp = KP_Fuzzy(E,EC);
        Kd = Kd_Fuzzy(EC);
        if(abs(E)<params.angle||sigmaValue <= params.sigma_mid)
        {
            output = (E * params.runP1) + (EC) * params.turnD1;
            cout << "直道!!!" << endl;
        }
            
        else
            output = Kp*E+Kd*(EC);
        
        int pwmDiff = (int)(output);
        errorLast = E;
        servoPwm = (uint16_t)(PWMSERVOMID + pwmDiff); // PWM转换
    }
    /**
     * @brief 变加速控制
     *
     * @param enable 加速使能
     * @param control
     */
    void speedCtrl(bool enable, bool slowDown, ControlCenter control,bool ring=false) {
    // 定义速度控制参数
    uint8_t controlLow = 0;   // 速度控制下限
    uint8_t controlMid = 5;   // 中间速度控制点
    uint8_t controlHigh = 10; // 速度控制上限

    // 定义速度参数
    float speedLow = params.speedLow;  // 最低速度
    float speedHigh = params.speedHigh; // 最高速度

    // 初始化速度和控制位移

    uint8_t countShift = controlLow;
    if(ring){
        speed = params.speedRing;
        return;
    }
    if (slowDown) {
        // 如果需要减速，直接设置为最低速度
        speed = speedLow;
    } else if (enable) {
        // 检查特定条件
        if (control.centerEdge.size() < 10 || 
            control.centerEdge[control.centerEdge.size() - 1].x > ROWSIMAGE / 2) {
            // 特定条件下，保持低速
            countShift = controlLow;
            std::cout << "this is the special scene" << std::endl;
        } else {
            // 根据sigma值调整速度
            float sigmaValue = abs(control.sigmaCenter);
            if (sigmaValue <= 60.0f) {
                // 当sigmaValue在0到90之间时，进行线性插值
                countShift = static_cast<uint8_t>(controlLow + 
                    (controlHigh - controlLow) * (1.0f - (sigmaValue / 60.0f)));
            } else {
                // 当sigmaValue超过50时，按最低速度跑
                countShift = controlLow;
            }

            // 确保countShift在控制范围内
            countShift = std::min(controlHigh, std::max(controlLow, countShift));
        }

        // 根据countShift计算速度
        speed = speedLow + (speedHigh - speedLow) * (countShift / static_cast<float>(controlHigh));
    } else {
        // 如果未使能，保持低速
        speed = speedLow;
        countShift = controlLow;
        std::cout<<"unable"<<std::endl;
    }
    std::cout<<"the speed is "<<speed<<std::endl;
        // 此处可以添加额外的代码，例如更新速度控制变量或通知其他系统组件
        // 例如：updateSpeed(speed); // 假设的函数，用于更新速度
    }

private:
    //使用的模糊PID程序，有几个特殊步骤，输入的参数E为err，EC为err的微分，即这次的err减去上次的err
    float KP_Fuzzy(float E,float EC)
    {
    
        int rule_p[7][7]=
        {
            { 6 , 5 , 4 , 4 , 3 , 0 , 0},//-36
            { 6 , 4 , 3 , 3 , 2 , 0 , 0},//-24
            { 4 , 3 , 2 , 1 , 0 , 1 , 2},//-12
            { 2 , 1 , 1 , 0 , 1 , 1 , 2},//0
            { 2 , 1 , 0 , 1 , 2 , 3 , 4},//12
            { 0 , 0 , 2 , 3 , 3 , 4 , 6},//24
            { 0 , 1 , 3 , 4 , 4 , 5 , 6},//36
        };//模糊规则表 P
    
        unsigned char i2;
        /*输入量P语言值特征点*/
        float EFF[7]={-110,-60,-40,0,40,60,110};
        /*输入量D语言值特征点*/
        float DFF[7]={-35,-18,-10,0,10,18,35};
        /*输出量U语言值特征点(根据赛道类型选择不同的输出值)*/
        float UFF[7] = {params.KP0,params.KP1,params.KP2,params.KP3,params.KP4,params.KP5,params.KP6};
    
        // for(i2=0;i2<7;i2++)
        //     UFF[i2]=params.kp_m/6*i2;
    
    
        float U=0;  /*偏差,偏差微分以及输出值的精确量*/
        float PF[2]={0},DF[2]={0},UF[4]={0};
        /*偏差,偏差微分以及输出值的隶属度*/
        int Pn=0,Dn=0,Un[4]={0};
        float t1=0,t2=0,t3=0,t4=0,temp1=0,temp2=0;
        /*隶属度的确定*/
        /*根据PD的指定语言值获得有效隶属度*/
        if(E>EFF[0] && E<EFF[6])
        {
            if(E<=EFF[1])
            {
                Pn=-2;
                PF[0]=(EFF[1]-E)/(EFF[1]-EFF[0]);
            }
            else if(E<=EFF[2])
            {
                Pn=-1;
                PF[0]=(EFF[2]-E)/(EFF[2]-EFF[1]);
            }
            else if(E<=EFF[3])
            {
                Pn=0;
                PF[0]=(EFF[3]-E)/(EFF[3]-EFF[2]);
            }
            else if(E<=EFF[4])
            {
                Pn=1;
                PF[0]=(EFF[4]-E)/(EFF[4]-EFF[3]);
            }
            else if(E<=EFF[5])
            {
                Pn=2;
                PF[0]=(EFF[5]-E)/(EFF[5]-EFF[4]);
            }
            else if(E<=EFF[6])
            {
                Pn=3;
                PF[0]=(EFF[6]-E)/(EFF[6]-EFF[5]);
            }
        }
    
        else if(E<=EFF[0])
        {
            Pn=-2;
            PF[0]=1;
        }
        else if(E>=EFF[6])
        {
            Pn=3;
            PF[0]=0;
        }
    
        PF[1]=1-PF[0];
    
    
        //判断D的隶属度
        if(EC>DFF[0]&&EC<DFF[6])
        {
            if(EC<=DFF[1])
            {
                Dn=-2;
                DF[0]=(DFF[1]-EC)/(DFF[1]-DFF[0]);
            }
            else if(EC<=DFF[2])
            {
                Dn=-1;
                DF[0]=(DFF[2]-EC)/(DFF[2]-DFF[1]);
            }
            else if(EC<=DFF[3])
            {
                Dn=0;
                DF[0]=(DFF[3]-EC)/(DFF[3]-DFF[2]);
            }
            else if(EC<=DFF[4])
            {
                Dn=1;
                DF[0]=(DFF[4]-EC)/(DFF[4]-DFF[3]);
            }
            else if(EC<=DFF[5])
            {
                Dn=2;
                DF[0]=(DFF[5]-EC)/(DFF[5]-DFF[4]);
            }
            else if(EC<=DFF[6])
            {
                Dn=3;
                DF[0]=(DFF[6]-EC)/(DFF[6]-DFF[5]);
            }
        }
        //不在给定的区间内
        else if (EC<=DFF[0])
        {
            Dn=-2;
            DF[0]=1;
        }
        else if(EC>=DFF[6])
        {
            Dn=3;
            DF[0]=0;
        }
    
        DF[1]=1-DF[0];
    
        /*使用误差范围优化后的规则表rule[7][7]*/
        /*输出值使用13个隶属函数,中心值由UFF[7]指定*/
        /*一般都是四个规则有效*/
        Un[0]=rule_p[Pn+2][Dn+2];
        Un[1]=rule_p[Pn+3][Dn+2];
        Un[2]=rule_p[Pn+2][Dn+3];
        Un[3]=rule_p[Pn+3][Dn+3];
    
        if(PF[0]<=DF[0])    //求小
            UF[0]=PF[0];
        else
            UF[0]=DF[0];
        if(PF[1]<=DF[0])
            UF[1]=PF[1];
        else
            UF[1]=DF[0];
        if(PF[0]<=DF[1])
            UF[2]=PF[0];
        else
            UF[2]=DF[1];
        if(PF[1]<=DF[1])
            UF[3]=PF[1];
        else
            UF[3]=DF[1];
        /*同隶属函数输出语言值求大*/
        if(Un[0]==Un[1])
        {
            if(UF[0]>UF[1])
                UF[1]=0;
            else
                UF[0]=0;
        }
        if(Un[0]==Un[2])
        {
            if(UF[0]>UF[2])
                UF[2]=0;
            else
                UF[0]=0;
        }
        if(Un[0]==Un[3])
        {
            if(UF[0]>UF[3])
                UF[3]=0;
            else
                UF[0]=0;
        }
        if(Un[1]==Un[2])
        {
            if(UF[1]>UF[2])
                UF[2]=0;
            else
                UF[1]=0;
        }
        if(Un[1]==Un[3])
        {
            if(UF[1]>UF[3])
                UF[3]=0;
            else
                UF[1]=0;
        }
        if(Un[2]==Un[3])
        {
            if(UF[2]>UF[3])
                UF[3]=0;
            else
                UF[2]=0;
        }
        t1=UF[0]*UFF[Un[0]];
        t2=UF[1]*UFF[Un[1]];
        t3=UF[2]*UFF[Un[2]];
        t4=UF[3]*UFF[Un[3]];
        temp1=t1+t2+t3+t4;
        temp2=UF[0]+UF[1]+UF[2]+UF[3];//模糊量输出
        if(temp2!=0)
            U=temp1/temp2;
        else {
            U=0;
        }
    //    temp1=PF[0]*UFF[Un[0]]+PF[1]*UFF[Un[1]]+PF[0]*UFF[Un[2]]+PF[1]*UFF[Un[3]]+DF[0]*UFF[Un[0]]+DF[0]*UFF[Un[1]]+DF[1]*UFF[Un[2]]+DF[0]*UFF[Un[3]];
    //    U=temp1;
        return U;
    }
    int rule_d[7] = { 6 , 5 , 3 , 2 , 3 , 5 , 6};//模糊规则表 D
    float Kd_Fuzzy(float EC)
    {
        float out=0;
        unsigned char i=0;
        float degree_left = 0,degree_right = 0;
        unsigned char degree_left_index = 0,degree_right_index = 0;
        float DFF[7]={-35,-18,-10,0,10,18,35};

        float UFF[7] = {params.KD0,params.KD1,params.KD2,params.KD3,params.KD4,params.KD5,params.KD6};
    
        // for(i=0;i<7;i++)
        //         UFF[i]=params.kd_m/6*i;
    
        if(EC<DFF[0])
        {
            degree_left = 1;
            degree_right = 0;
            degree_left_index = 0;
        }
        else if (EC>DFF[6]) {
            degree_left = 1;
            degree_right = 0;
            degree_left_index = 6;
        }
        else {
            for(i=0;i<6;i++)
            {
                if(EC>=DFF[i]&&EC<DFF[i+1])
                {
                    degree_left = (float)(DFF[i+1] - EC)/(DFF[i+1] - DFF[i]);
                    degree_right = 1 - degree_left;
                    degree_left_index = i;
                    degree_right_index = i+1;
                    break;
                }
            }
        }
    
        out = UFF[rule_d[degree_left_index]]*degree_left+UFF[rule_d[degree_right_index]]*degree_right;
    
        return out;
    }
};
/*
 * pid.h
 *功能：pid控制函数，各个参数版本供参考
 *  Created on: 2021年7月2日
 *      Author: 12249
 */

#ifndef PID_H_
#define PID_H_

#include <Arduino.h>

/**************************/


#define SPD_P_DATA 100.0f // 比例常数 Proportional Const
#define SPD_I_DATA 35.0f  // 积分常数  Integral Const    --动态响应
#define SPD_D_DATA 2.0f // 微分常数 Derivative Const    --预测作用，增大阻尼
#define TARGET_SPEED 0
#define LIMIT   800      //积分限幅

/***************************/

typedef struct
{
    double SumError; //误差累计
    double Proportion; //比例常数Proportional Const
    double Integral; //积分常数 IntegralConst
    double Derivative; //微分常数Derivative Const
    float SetPoint; //设定目标 DesiredValue
    float LastError; //Error[-1]
    float PrevError; //Error[-2]
    int limit;      //调节CCR限幅
}PIDs;

typedef struct   //PID参数保存，设置为变量形式，用于PID调参
{
  float PID_P;
  float PID_I;
  float PID_D;
  float PID_LIMIT;
}PID_Param;

void PID_ParamInit(PIDs *pid, PID_Param *param); //初始化pid参数,每次设置均清零误差
void PID_TargetSet(PIDs *pid, float targetPoint); //设置pid调节目标，第一次初始化后必须设置目标位置
float PID_Calculate(PIDs *pid,float NextPoint);    //速度闭环PID控制设计


/*
//整数运算版pid
//bios系数为1--平地行驶
#define SPD_P_DATA 100 // 比例常数 Proportional Const
#define SPD_I_DATA 35  // 积分常数  Integral Const    --动态响应
#define SPD_D_DATA 2 // 微分常数 Derivative Const    --预测作用，增大阻尼
#define TARGET_SPEED 0
#define LIMIT   800      //积分限幅

typedef struct
{
    long SumError; //误差累计
    uint32_t Proportion; //比例常数Proportional Const
    uint32_t Integral; //积分常数 IntegralConst
    uint32_t Derivative; //微分常数Derivative Const
    uint32_t SetPoint; //设定目标 DesiredValue
    int LastError; //Error[-1]
    int PrevError; //Error[-2]
    int lastCCR;
    int limit;      //调节CCR限幅

}PIDs;
*/


/*short PID_Pos_PosCalc(short NextPoint);

short PID_Ang_PosCalc(short NextPoint);
*/

#endif /* PID_H_ */

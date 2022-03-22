/*
 * pid.cpp
 *
 *  Created on: 2021年7月2日
 *      Author: 12249
 */

#include "pid.h"

/******************** PID 控制设计 ***************************/
/**
  * 函数功能: PID参数初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void PID_ParamInit(PIDs *pid, PID_Param *param) //初始化pid参数,每次设置均清零误差
{
    pid->LastError = 0.0;               // Error[-1]
    pid->PrevError = 0.0;               // Error[-2]
    pid->Proportion = param->PID_P; // 比例常数 Proportional Const
    pid->Integral = param->PID_I;   // 积分常数  Integral Const
    pid->Derivative = param->PID_D; // 微分常数 Derivative Const
    pid->limit = param->PID_LIMIT;  //设定积分限幅
}

void PID_TargetSet(PIDs *pid, float targetPoint) //设置pid调节目标，第一次初始化后必须设置目标位置
{
  pid->SetPoint = targetPoint;  
}


/**
  * 函数名称：速度闭环PID控制设计
  * 输入参数：当前控制量
  * 返 回 值：目标控制量
  * 说    明：无
  */
//浮点数运算
float PID_Calculate(PIDs *pid,float NextPoint)    //速度闭环PID控制设计--增量式pid
{
    register float iError,InteError,iIncpid;
    iError = (float)pid->SetPoint - NextPoint; //偏差

    if((iError<0.5f )&&(iError>-0.5f))           //偏差滤波
    iError = 0.0f;

    pid->SumError+=iError;
    InteError=pid->Integral * iError;
    if((pid->SumError>pid->limit)||(pid->SumError<-(pid->limit)))           //积分项限幅，减小超调
    {    InteError=0;
        pid->SumError-=iError;
    }

    //send_pidpoint("add 2,1,",(int)pid->SumError);

    //if(InteError>15.0) InteError=0.0;
    iIncpid = (pid->Proportion * (iError - pid->LastError) )                //E[k]项
              + (InteError)     //E[k-1]项
              + (pid->Derivative * (iError-2*pid->LastError+pid->PrevError));  //E[k-2]项

    pid->PrevError = pid->LastError;
    pid->LastError = iError;
    return iIncpid;
}


//整数运算版pid--加快处理速度
/*
long SpdPIDCalc(PIDs *pid,uint32_t NextPoint)    //速度闭环PID控制设计--增量式pid
{
    register uint32_t iError,InteError,iIncpid;
    iError = pid->SetPoint - NextPoint; //偏差

    if((iError<3 )&&(iError>-3))           //偏差滤波
    iError = 0;

    pid->SumError+=iError;
    InteError=pid->Integral * iError;
    if((pid->SumError>LIMIT)||(pid->SumError<-300))           //积分项限幅，减小超调
    {    InteError=0;
        pid->SumError-=iError;
    }

    //send_pidpoint("add 2,1,",(int)pid->SumError);

    //if(InteError>15.0) InteError=0.0;
    iIncpid = (pid->Proportion * (iError - pid->LastError) )                //E[k]项
              + (InteError)     //E[k-1]项
              + (pid->Derivative * (iError-2*pid->LastError+pid->PrevError));  //E[k-2]项

    pid->PrevError = pid->LastError;
    pid->LastError = iError;
    return iIncpid;
}
*/

/*
int32_t PosPIDCalc(PIDs *pid,float NextPoint)    //速度闭环PID控制设计--位置式pid
{
    register float iError,iIncpid;
    iError = (float)pid->SetPoint - NextPoint; //偏差

    if((iError<1.0f )&&(iError>-1.0f))           //偏差滤波
    iError = 0.0f;

    pid->SumError+=iError;
    //InteError=pid->Integral * iError;
    if((pid->SumError>1200)){           //积分项限幅，减小超调
        pid->SumError=1200;
    }
    if(pid->SumError<-1200){
        pid->SumError=-1200;
    }

    //if(InteError>15.0) InteError=0.0;
    iIncpid = (pid->Proportion * (iError) )                //E[k]项
              + (pid->Integral*pid->SumError)     //E[k-1]项
              + (pid->Derivative * (iError-pid->LastError));  //E[k-2]项

    pid->LastError = iError;
    return iIncpid;
}
*/

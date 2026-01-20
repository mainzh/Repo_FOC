#include "pid.h"

/**
  * 函    数：PID计算及结构体变量值更新
  * 参    数：PID_t * 指定结构体的地址
  * 返 回 值：无
  */
void PID_Update(PID_HandleTypeDef *p)
{
    /*获取本次误差和上次误差*/
    p->ErrorPrev = p->Error;             //获取上次误差
    p->Error = p->Target - p->Actual;    //获取本次误差，目标值减实际值，即为误差值
    
    /*外环误差积分（累加）*/
    /*如果Ki不为0，才进行误差积分，这样做的目的是便于调试*/
    /*因为在调试时，我们可能先把Ki设置为0，这时积分项无作用，误差消除不了，误差积分会积累到很大的值*/
    /*后续一旦Ki不为0，那么因为误差积分已经积累到很大的值了，这就导致积分项疯狂输出，不利于调试*/
    if (p->Ki != 0)                 //如果Ki不为0
    {
        p->ErrorInt += p->Error;    //进行误差积分
    }
    else                            //否则
    {
        p->ErrorInt = 0;            //误差积分直接归0
    }
    
    /*PID计算*/
    /*使用位置式PID公式，计算得到输出值*/
    p->Out = p->Kp * p->Error
           + p->Ki * p->ErrorInt
           + p->Kd * (p->Error - p->ErrorPrev);
    
    /*输出限幅*/
    if (p->Out > p->OutMax) {p->Out = p->OutMax;}    //限制输出值最大为结构体指定的OutMax
    if (p->Out < p->OutMin) {p->Out = p->OutMin;}    //限制输出值最小为结构体指定的OutMin
}

PID_HandleTypeDef velocityPID;
PID_HandleTypeDef AnglePID;

// PID_Init
void PID_Init(void)
{
    /*定义PID结构体变量*/
    //速度环PID结构体变量，定义的时候同时给部分成员赋初值
//    velocityPID.Target = 15 * 6.28;              //速度环目标值初值设定
    velocityPID.Kp = 0.2;                         //比例项权重，PID系数的数量级一般由输出范围和输入范围的比值确定，即 Actual / Out
    velocityPID.Ki = 0.005;                         //积分项权重
    velocityPID.Kd = 0;                         //微分项权重
    velocityPID.OutMax = 27;                    //输出限幅的最大值
    velocityPID.OutMin = -27;                   //输出限幅的最小值
    
    //角度环PID结构体变量，定义的时候同时给部分成员赋初值
//    AnglePID.Target = 1 * 6.28;    //角度环目标值初值设定为中心角度值
    AnglePID.Kp = 1;    //比例项权重
    AnglePID.Ki = 0;    //积分项权重
    AnglePID.Kd = 0.01;    //微分项权重
    AnglePID.OutMax = 15 * 6.28;    //输出限幅的最大值
    AnglePID.OutMin = -15 * 6.28;    //输出限幅的最小值
}

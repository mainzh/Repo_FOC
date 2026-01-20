#ifndef __PID_H
#define __PID_H


/* PID结构体变量 */
typedef struct {
    float Target;       /* 目标值 */
    float Actual;       /* 实际值 */
    float Out;          /* 输出值 */
    
    float Kp;           /* 比例环系数 */
    float Ki;           /* 积分环系数 */
    float Kd;           /* 微分环系数 */
    
    float Error;        /* 本次误差值 */
    float ErrorPrev;    /* 上次误差值 */
    float ErrorInt;     /* 误差值积分 */
    
    float OutMax;       /* 输出限幅最大值 */
    float OutMin;       /* 输出限幅最小值 */
} PID_HandleTypeDef;


extern PID_HandleTypeDef velocityPID;
extern PID_HandleTypeDef AnglePID;

void PID_Init(void);

void PID_Update(PID_HandleTypeDef *p);    /* PID计算及结构体变量值更新 */

#endif    /* __PID_H */

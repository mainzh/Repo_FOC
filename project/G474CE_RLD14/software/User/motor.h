#ifndef __MOTOR_H
#define __MOTOR_H

#include "tim.h"
#include "foc_utils.h"
#include "lowpass_filter.h"
#include "pid.h"

#include "encoder.h"

#include "data_format.h"

/*内部使用*/
#define m_pwm_htimx         htim1         /* 电机定时器句柄*/
#define MOS_EN_GPIO_Port    GPIOA         /* MOS使能口 */
#define MOS_EN_Pin          GPIO_PIN_6    /* MOS使能引脚 */

/* 电机MOS使能 */
#define motor_MOS(x)    do{ x ? \
                      HAL_GPIO_WritePin(MOS_EN_GPIO_Port, MOS_EN_Pin, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(MOS_EN_GPIO_Port, MOS_EN_Pin, GPIO_PIN_RESET); \
                  }while(0)                                                          /* x = 1 , IO置高电平；x = 0 , IO置低电平 */

/*设置三相的状态*/
#define PWM_Period (m_pwm_htimx.Init.Period + 1)

#define U0 __HAL_TIM_SET_COMPARE(&m_pwm_htimx , TIM_CHANNEL_1 , 0);
#define V0 __HAL_TIM_SET_COMPARE(&m_pwm_htimx , TIM_CHANNEL_2 , 0);
#define W0 __HAL_TIM_SET_COMPARE(&m_pwm_htimx , TIM_CHANNEL_3 , 0);
#define U1 __HAL_TIM_SET_COMPARE(&m_pwm_htimx , TIM_CHANNEL_1 , PWM_Period/10);
#define V1 __HAL_TIM_SET_COMPARE(&m_pwm_htimx , TIM_CHANNEL_2 , PWM_Period/10);
#define W1 __HAL_TIM_SET_COMPARE(&m_pwm_htimx , TIM_CHANNEL_3 , PWM_Period/10);

/**
 *  Direction structure  方向
 */
typedef enum
{
    CW      = 1,     //clockwise
    CCW     = -1,    //counter clockwise
    UNKNOWN = 0      //not yet known or invalid state
} DirState;

/* 电机构体变量 */
typedef struct {
    float voltage_supply;         /* V，供电电压 */
    float pole_pairs;             /* 磁极对数，按照实际设置，虽然可以上电检测但有失败的概率 */
    
    float angle;                  /* rad，角度 */
    float velocity;               /* rad/s，速度 */
    float electrical_angle;       /* rad，电角度 */
    float zero_electric_angle;    /* rad，初始电角度 */
    
} Motor_HandleTypeDef;

/**
 *  Motion control type  运动控制类型
 */
typedef enum
{
    torque,               //!< Torque control  力矩控制
    velocity,             //!< Velocity motion control  速度运动控制
    angle,                //!< Position/angle motion control  角度运动控制
    velocity_openloop,    //速度开环
    angle_openloop        //角度开环
} MotionControlType;

extern float power_on_pos;  //重上电位置
extern float shaft_angle;    /* 电机角度 */
extern float shaft_velocity;    /* 电机速度 */

extern float shaft_velocity_nolpf;    /* 未滤波的速度 */

extern uint16_t absolute_pos_remaining_cycles; /*绝对位置剩余循环次数*/

/*外部使用*/
void Motor_init(void);    /* 电机初始化 */

void motor_6step(float ms);    /* 电机六步相 */

void setPhaseVoltage(float Uq, float Ud, float angle_el);
void FOC_openLoop_velocity(float Uq, float rate_angle_el);    /* 验证setPhaseVoltage函数 */

void systick_CountMode(void);
float velocityOpenloop(float target_velocity, float Uq, float Ud);    /* 速度开环 */
float angleOpenloop(float target_angle, float target_velocity, float Uq, float Ud);    /* 角度开环 */

float electricalAngle(void);    /* 计算电角度 */
int alignSensor(void);    /* 检测零点偏移量和极对数 */

#endif    /* __MOTOR_H */

#ifndef __ENCODER_H
#define __ENCODER_H


#include "dwt_delay.h"
#include "mu_1sf_driver.h"


#define _abs(x) ((x)>0?(x):-(x))

extern float getAngle_Without_track(void);    /* 编码器单圈值 弧度值 */

float getAngle(void);    /* 编码器多圈值 弧度值 */

#endif    /* __ENCODER_H */

#include "encoder.h"


float angle_prev = 0;
int32_t full_rotations = 0; // full rotation tracking;

/* 编码器多圈值 弧度值 */
float getAngle(void)
{
    float val = getAngle_Without_track();
    
    float d_angle = val - angle_prev;
    
    //计算旋转的总圈数
    //通过判断角度变化是否大于80%的一圈(0.8f*6.28318530718f)来判断是否发生了溢出，如果发生了，则将full_rotations增加1（如果d_angle小于0）或减少1（如果d_angle大于0）。
    if(_abs(d_angle) > (0.8f * 6.28318530718f)) full_rotations += ((d_angle > 0) ? -1 : 1); 
    angle_prev = val;
    return (float)full_rotations * 6.28318530718f + angle_prev;
}

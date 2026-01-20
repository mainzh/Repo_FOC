#include "motor.h"

float pos_len = 65;  //直线模组有效行程，出厂前需要标定从伸极限至缩极限的编码器读数范围
float voltage_power_supply = 48;    /* V，供电电压 */
int pole_pairs = 3;    /* 电机磁极对数，按照实际设置，虽然可以上电检测但有失败的概率 */

float voltage_sensor_align = 3;    /* V  重要参数，航模电机大功率0.5-1，云台电机小功率2-3 */

float shaft_angle;    /* 电机角度 */
float shaft_velocity;    /* 电机速度 */
float electrical_angle;    /* 电机电角度 */
float zero_electric_angle;    /* 初始电角度 */

DirState sensor_direction = CW;    /* 传感器方向,RDC14现象为电机顺时针转编码器值增加， */

/* ***************************************************************** */
/* 电机初始化 */
void Motor_init(void)
{
    printf("MOT: Init\r\n");
//    sensor_direction = UNKNOWN;
    
    /* 定时器PWM输出开始 */
    HAL_TIM_PWM_Start(&m_pwm_htimx , TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&m_pwm_htimx , TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&m_pwm_htimx , TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&m_pwm_htimx , TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&m_pwm_htimx , TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&m_pwm_htimx , TIM_CHANNEL_3);
    
    printf("MOT: Enable driver.\r\n");
}

/* ***************************************************************** */
/* 电机六步相
 * 直流无刷电机换相变频驱动 不可长时间运行
 * ms    关断时间 毫秒
 */
void motor_6step(float ms)
{
    /*U相通*/
    U1;V0;W0;
    HAL_Delay(ms);
    /*V相通*/
    U0;V1;W0;
    HAL_Delay(ms);
    /*W相通*/
    U0;V0;W1;
    HAL_Delay(ms);
}

/* ***************************************************************** */
/* 配置PWM占空比 */
void setPWM(float Ua ,float Ub ,float Uc)
{
    __HAL_TIM_SET_COMPARE(&m_pwm_htimx, TIM_CHANNEL_1, Ua);
    __HAL_TIM_SET_COMPARE(&m_pwm_htimx, TIM_CHANNEL_2, Ub);
    __HAL_TIM_SET_COMPARE(&m_pwm_htimx, TIM_CHANNEL_3, Uc);
}

/* FOC核心函数：输入Ud、Uq和电角度，输出PWM 采用SVPWM */
void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
    float Uout;
    uint32_t sector;
    float T0,T1,T2;
    float Ta,Tb,Tc;
    
    if(Ud) // only if Ud and Uq set 
    {// _sqrt is an approx of sqrt (3-4% error)
        Uout = _sqrt(Ud*Ud + Uq*Uq) / voltage_power_supply;
        // angle normalisation in between 0 and 2pi
        // only necessary if using _sin and _cos - approximation functions
        angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
    }
    else
    {// only Uq available - no need for atan2 and sqrt
        Uout = Uq / voltage_power_supply;
        // angle normalisation in between 0 and 2pi
        // only necessary if using _sin and _cos - approximation functions
        angle_el = _normalizeAngle(angle_el + _PI_2);
    }
    
    /* 六边形的内切圆(SVPWM最大不失真旋转电压矢量赋值)根号3/3 */
    if(Uout > 0.577f) Uout = 0.577f;
    if(Uout < -0.577) Uout = -0.577f;
    
    /* 判断参考电压矢量所在扇区 */
    sector = (angle_el / _PI_3) + 1;
    
    /* 计算相邻电压矢量作用时间 */
    T1 = _SQRT3*_sin(sector*_PI_3 - angle_el) * Uout;
    T2 = _SQRT3*_sin(angle_el - (sector-1.0)*_PI_3) * Uout;
    /* 计算零矢量作用时间 */
    T0 = 1 - T1 - T2;
    
    /* 计算各相桥臂的开关时间 */
    switch(sector)
    {
        case 1:
            Ta = T1 + T2 + T0 / 2;
            Tb = T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 2:
            Ta = T1 +  T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 3:
            Ta = T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T2 + T0 / 2;
            break;
        case 4:
            Ta = T0 / 2;
            Tb = T1+ T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 5:
            Ta = T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 6:
            Ta = T1 + T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T0 / 2;
            break;
        /*possible error state*/
        default:
            Ta = 0;
            Tb = 0;
            Tc = 0;
    }
    
    /* 配置PWM占空比 */
    setPWM(Ta * PWM_Period, Tb * PWM_Period, Tc * PWM_Period);
}

/* ***************************************************************** */
/* 验证setPhaseVoltage函数 */
void FOC_openLoop_velocity(float Uq, float rate_angle_el)
{
    static float angle_el;
    /*估计电角度*/
    angle_el += rate_angle_el;
    
    setPhaseVoltage(Uq, 0, angle_el);
}

/* 0xFFFFFF到0循环计数 */
void systick_CountMode(void)
{
    SysTick->LOAD = 0xFFFFFF-1;      //set reload register
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk; //Enable SysTick Timer
}

unsigned long open_loop_timestamp;

/* 速度开环 */
/* 单位：rad/s */
float velocityOpenloop(float target_velocity, float Uq, float Ud)
{
	unsigned long now_us;
	float Ts;
	
	
	now_us = SysTick->VAL; //_micros();
//    now_us = HAL_GetTick();
    
	if(now_us<open_loop_timestamp)Ts = (float)(open_loop_timestamp - now_us)/9*1e-6;
	else
		Ts = (float)(0xFFFFFF - now_us + open_loop_timestamp)/9*1e-6;
	open_loop_timestamp=now_us;  //save timestamp for next call
  // quick fix for strange cases (micros overflow)
  if(Ts == 0 || Ts > 0.5) Ts = 1e-3; 
	
	// calculate the necessary angle to achieve target velocity
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts); 

	// set the maximal allowed voltage (voltage_limit) with the necessary angle
  setPhaseVoltage(Uq,  Ud, _electricalAngle(shaft_angle, pole_pairs));
	
	return Uq;
}

/* 角度开环 */
float angleOpenloop(float target_angle, float target_velocity, float Uq, float Ud)
{
	unsigned long now_us;
	float Ts;
	
	now_us = SysTick->VAL; //_micros();
//    now_us = HAL_GetTick();
    
	if(now_us<open_loop_timestamp)Ts = (float)(open_loop_timestamp - now_us)/9*1e-6;
	else
		Ts = (float)(0xFFFFFF - now_us + open_loop_timestamp)/9*1e-6;
  open_loop_timestamp = now_us;  //save timestamp for next call
  // quick fix for strange cases (micros overflow)
  if(Ts == 0 || Ts > 0.5) Ts = 1e-3; 
	
	// calculate the necessary angle to move from current position towards target angle
  // with maximal velocity (velocity_limit)
  if(fabs( target_angle - shaft_angle ) > target_velocity*Ts)
	{
    shaft_angle += _sign(target_angle - shaft_angle) * target_velocity * Ts;
    //shaft_velocity = velocity_limit;
  }
	else
	{
    shaft_angle = target_angle;
    //shaft_velocity = 0;
  }
    
	// set the maximal allowed voltage (voltage_limit) with the necessary angle
	setPhaseVoltage(Uq, Ud, _electricalAngle(shaft_angle, pole_pairs));
	
  return Uq;
}

/* ***************************************************************** */
/* 计算电角度 */
float electricalAngle(void)
{
    return  _normalizeAngle((float)(sensor_direction * pole_pairs) * getAngle() - zero_electric_angle);
}

/* ***************************************************************** */
/**
  * 检测零点偏移量和极对数
  * 需要先接通供电，在下载程序才可
  */
int alignSensor(void)
{
//    long i;
//    float angle;
//    float mid_angle, end_angle;
//    float moved;
//    
//    printf("MOT: Align sensor.\r\n");
//    
//    /* 检测传感器方向 */
//    // find natural direction
//    // move one electrical revolution forward
//    for(i = 0; i <= 500; i++)    /*正转2pi电角度，分500次*/
//    {
//        angle = _3PI_2 + _2PI * i / 500.0;
//        setPhaseVoltage(voltage_sensor_align, 0,  angle);
//        HAL_Delay(2);
//    }
//    mid_angle = getAngle();
//    
//    for(i = 500; i >= 0; i--)    /*反转2pi电角度，分500次*/
//    {
//        angle = _3PI_2 + _2PI * i / 500.0 ;
//        setPhaseVoltage(voltage_sensor_align, 0,  angle);
//        HAL_Delay(2);
//    }
//    end_angle = getAngle();
//    setPhaseVoltage(0, 0, 0);
//    HAL_Delay(200);
//    
//    printf("mid_angle = %.4f\r\n", mid_angle);
//    printf("end_angle = %.4f\r\n", end_angle);
//    
//    moved = fabs(mid_angle - end_angle);
//    
//    if((mid_angle == end_angle)||(moved < 0.02))  //相等或者几乎没有动，没接入编码器
//    {
//        printf("MOT: Failed to notice movement.\r\n");
//        motor_MOS(0);    //电机检测不正常，关闭驱动
//        return 0;
//    }
//    else if(mid_angle < end_angle)
//    {
//        printf("MOT: sensor_direction == CCW\r\n");
//        sensor_direction = CCW;
//    }
//    else
//    {
//        printf("MOT: sensor_direction == CW\r\n");
//        sensor_direction = CW;
//    }
    
//    /* 检测磁极对数 */
//    printf("MOT: PP check: ");
//    if(fabs(moved * pole_pairs - _2PI) > 0.5 )    // 0.5 is arbitrary number it can be lower or higher!
//    {
//        printf("fail - estimated pp: ");    //如果不等于设置值，以计算值为准
//        pole_pairs = _2PI / moved + 0.5;     //浮点数转整形，四舍五入，电角度转了2pi，是机械角度的几倍就是磁极对数
//        printf("%d\r\n", pole_pairs);
//    }
//    else
//        printf("PP OK!\r\n");
    
    /* 计算零点偏移角度 */
    setPhaseVoltage(voltage_sensor_align, 0,  _3PI_2);
    HAL_Delay(700);
    zero_electric_angle = _normalizeAngle(_electricalAngle((float)sensor_direction * getAngle(), pole_pairs));
    HAL_Delay(20);
    printf("MOT: zero_electric_angle: ");
    printf("%.4f\r\n",zero_electric_angle);
    
    setPhaseVoltage(0, 0, 0);
    HAL_Delay(200);
    
    return 1;
}

/* ***************************************************************** */
/* 确定输出极性：
* 1.用开环控制电机旋转，当给+Uq时，电机顺时针转为正
* 2.驱动编码器，当电机顺时针转，编码器读数增加为正
*/

/*角度环行程可设置80rad，重上电需重新标定机械位置对应的弧度值*/
/*直线模组禁止堵转，否则电源功率会快速上升，出现啸叫*/

float shaft_angle_prev;    /* 上次电机角度 */
float shaft_velocity_nolpf;    /* 未滤波的速度 */

uint16_t absolute_pos_remaining_cycles; /*绝对位置剩余循环次数*/
float power_on_pos;  //重上电位置

/* ***************************************************************** */
/**
 * @param        htim:定时器句柄指针
 * @note        此函数会被定时器中断函数共同调用的
 * @retval      无
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /*定义静态变量（默认初值为0，函数退出后保留值和存储空间）*/
    static uint16_t Count1, Count2;    //分别用于任务1和任务2的计次分频
    
    if (htim->Instance == TIM4)
    {
        /*每隔1ms，程序执行到这里一次*/
        
        /* 使用以下位置开环程序进行编码器校准，速度限制0.25 * 6.28 rad/s，正反转 15 * 6.28 */
//        angleOpenloop(5 * 6.28, 0.1 * 6.28, 0.5, 0);    /* 角度开环，可移动行程38，导程2.5，可旋转 38/2.5 = 15.2圈，即 15.2 * 6.28 */
        /* 编码器校准通过 */
        
                /* 速度、角度处理 */
//        shaft_angle_prev = shaft_angle;
//        
////        shaft_angle = getAngle() * 180 / _PI / 360;    /* 转，在这里转换单位会放大数据误差波动 */
//        shaft_angle = getAngle();    /* rad */
//        
//        shaft_velocity_nolpf = shaft_angle - shaft_angle_prev;
////        shaft_velocity = LPF_velocity(shaft_angle - shaft_angle_prev) * 1000 * 60;    /* 转每分rpm */
//        shaft_velocity = sensor_direction * LPF_velocity(shaft_velocity_nolpf * 1000.0);    /* rad/s */
//        
//        electrical_angle = electricalAngle();
        
//        /*以下进行速度环PID控制*/
//        velocityPID.Actual = shaft_velocity;    //速度环的实际值
//        PID_Update(&velocityPID);               //调用封装好的函数，一步完成PID计算和更新
//        setPhaseVoltage(velocityPID.Out, 0, electrical_angle);   /* 速度环的输出值给到电机PWM */
        
//        /*角度环计次分频*/
//        Count1 ++;    //计次自增
//        if (Count1 >= 5)    //如果计次5次，则if成立，即if每隔5ms进一次
//        {
//            Count1 = 0;    //计次清零，便于下次计次
//            
//            /*以下进行角度环PID控制*/
//            AnglePID.Actual = shaft_angle;    //外环为角度环，实际值为角度值
//            PID_Update(&AnglePID);    //调用封装好的函数，一步完成PID计算和更新
//            velocityPID.Target = AnglePID.Out;    //外环的输出值作用于内环的目标值，组成串级PID结构
//        }
        
        switch(packet_flag)
        {
            case PACKET_TYPE_CURRENT_OPEN_PARAM:
                switch(motor_flag)
                {
                    case MOTOR_FORWARD:
                        setPhaseVoltage(current_open_params.iq_target, current_open_params.id_target, current_open_params.electrical_angle);
                        break;
                    case MOTOR_REVERSE:
                        //电流开环不需要正反转
                        break;
                    default:
                        break;
                }
                break;
            
            case PACKET_TYPE_SPEED_OPEN_PARAM:
                switch(motor_flag)
                {
                    case MOTOR_FORWARD:
                        velocityOpenloop(speed_open_params.speed_target, speed_open_params.uq, speed_open_params.ud);    /* 速度开环 */
                        break;
                    case MOTOR_REVERSE:
                        velocityOpenloop(-speed_open_params.speed_target, speed_open_params.uq, speed_open_params.ud);    /* 速度开环 */
                        break;
                    default:
                        break;
                }
                break;
                
            case PACKET_TYPE_POSITION_OPEN_PARAM:
                switch(motor_flag)
                {
                    case MOTOR_FORWARD:
                        angleOpenloop(position_open_params.angle_target, position_open_params.speed_target, 
                                position_open_params.uq, position_open_params.ud);    /* 角度开环 */
                        break;
                    case MOTOR_REVERSE:
                        angleOpenloop(-position_open_params.angle_target, position_open_params.speed_target, 
                                position_open_params.uq, position_open_params.ud);    /* 角度开环 */
                        break;
                    default:
                        break;
                }
                break;
                
            case PACKET_TYPE_SPEED_CLOSED_PARAM:
                /* 速度、角度处理 */
                shaft_angle_prev = shaft_angle;
        
//                shaft_angle = getAngle() * 180 / _PI / 360;    /* 转，在这里转换单位会放大数据误差波动 */
                shaft_angle = getAngle();    /* rad */
        
                shaft_velocity_nolpf = shaft_angle - shaft_angle_prev;
//                shaft_velocity = LPF_velocity(shaft_angle - shaft_angle_prev) * 1000 * 60;    /* 转每分rpm */
                shaft_velocity = sensor_direction * LPF_velocity(shaft_velocity_nolpf * 1000.0);    /* rad/s */
                
                electrical_angle = electricalAngle();
                
                switch(motor_flag)
                {
                    case MOTOR_FORWARD:
                        /*以下进行速度环PID控制*/
                        velocityPID.Actual = shaft_velocity;    //速度环的实际值
                        PID_Update(&velocityPID);               //调用封装好的函数，一步完成PID计算和更新
                        setPhaseVoltage(velocityPID.Out, 0, electrical_angle);   /* 速度环的输出值给到电机PWM */
                        break;
                    case MOTOR_REVERSE:
                        motor_flag = MOTOR_FORWARD;
                        break;
                    default:
                        break;
                }
                break;
                
            case PACKET_TYPE_POSITION_CLOSED_PARAM:
                /* 速度、角度处理 */
                shaft_angle_prev = shaft_angle;
        
//                shaft_angle = getAngle() * 180 / _PI / 360;    /* 转，在这里转换单位会放大数据误差波动 */
                shaft_angle = getAngle();    /* rad */
        
                shaft_velocity_nolpf = shaft_angle - shaft_angle_prev;
//                shaft_velocity = LPF_velocity(shaft_angle - shaft_angle_prev) * 1000 * 60;    /* 转每分rpm */
                shaft_velocity = sensor_direction * LPF_velocity(shaft_velocity_nolpf * 1000.0);    /* rad/s */
                
                electrical_angle = electricalAngle();
                
                switch(motor_flag)
                {
                    case MOTOR_FORWARD:
                        /*以下进行速度环PID控制*/
                        velocityPID.Actual = shaft_velocity;    //速度环的实际值
                        PID_Update(&velocityPID);               //调用封装好的函数，一步完成PID计算和更新
                        setPhaseVoltage(velocityPID.Out, 0, electrical_angle);   /* 速度环的输出值给到电机PWM */
                        
                        /*角度环计次分频*/
                        Count1 ++;    //计次自增
                        if (Count1 >= 5)    //如果计次5次，则if成立，即if每隔5ms进一次
                        {
                            Count1 = 0;    //计次清零，便于下次计次
                            
                            /*以下进行角度环PID控制*/
                            AnglePID.Actual = shaft_angle;    //外环为角度环，实际值为角度值
                            PID_Update(&AnglePID);    //调用封装好的函数，一步完成PID计算和更新
                            velocityPID.Target = AnglePID.Out;    //外环的输出值作用于内环的目标值，组成串级PID结构
                        }
                        break;
                    case MOTOR_REVERSE:    //位置闭环不需要正反转
                        break;
                    default:
                        break;
                }
                break;
            
            case PACKET_TYPE_ABSOLUTE_POS_PARAM:
                //对于该直线模组，向外伸为正转，不可超过伸极限；向内缩为反转，不可超过缩极限。
                //伸过程中，编码器的值，由小变大
                
                /* 速度、角度处理 */
                shaft_angle_prev = shaft_angle;
        
//                shaft_angle = getAngle() * 180 / _PI / 360;    /* 转，在这里转换单位会放大数据误差波动 */
                shaft_angle = getAngle();    /* rad */
        
                shaft_velocity_nolpf = shaft_angle - shaft_angle_prev;
//                shaft_velocity = LPF_velocity(shaft_angle - shaft_angle_prev) * 1000 * 60;    /* 转每分rpm */
                shaft_velocity = sensor_direction * LPF_velocity(shaft_velocity_nolpf * 1000.0);    /* rad/s */
                
                electrical_angle = electricalAngle();
                
                
                if(motor_flag == MOTOR_ENABLED)
                {
                    
                    /* 绝对位置循环次数 */
//                    for(absolute_pos_remaining_cycles = absolute_pos_params.cycle_count; 
//                         absolute_pos_remaining_cycles > 0; 
//                         absolute_pos_remaining_cycles--)
//                    {
//                        
//                        if(fabsf(AnglePID.Actual - absolute_pos_params.absolute_position1) <= 0.01f)
//                        {
//                            AnglePID.Target = absolute_pos_params.absolute_position2;
//                        }
//                        else if(fabsf(AnglePID.Actual - absolute_pos_params.absolute_position2) <= 0.01f)
//                        {
//                            AnglePID.Target = absolute_pos_params.absolute_position1;
//                        }
//                    }
                    
                    //1.如果绝对位置设置值1 > 重上电位置 (超出伸极限)，绝对位置设置值1 = 重上电位置
                    if(absolute_pos_params.absolute_position1 > power_on_pos )
                    {
                        absolute_pos_params.absolute_position1 = power_on_pos;  //伸极限
                    }
                    //2.如果绝对位置设置值2 < 重上电位置 - 行程 (超出缩极限)，绝对位置设置值2 = 重上电位置 - 行程
                    if(absolute_pos_params.absolute_position2 < power_on_pos - pos_len) // power_on_pos - 15 有效行程
                    {
                        absolute_pos_params.absolute_position2 = power_on_pos - pos_len;  //缩极限
                    }
                    
                    //从绝对位置设置值1运动到绝对位置设置值2，绝对位置设置值1 > 绝对位置设置值2
                    if(fabsf(AnglePID.Actual - absolute_pos_params.absolute_position1) <= 0.05f)
                    {
                        AnglePID.Target = absolute_pos_params.absolute_position2;
                    }
                    else if(fabsf(AnglePID.Actual - absolute_pos_params.absolute_position2) <= 0.05f)
                    {
                        AnglePID.Target = absolute_pos_params.absolute_position1;
                    }
                    
//                    for(absolute_pos_status_params.remaining_cycles = absolute_pos_params.cycle_count; 
//                         absolute_pos_status_params.remaining_cycles > 0; 
//                         absolute_pos_status_params.remaining_cycles--)
//                    {
//                        
//                        if(fabsf(AnglePID.Actual - absolute_pos_params.absolute_position1) <= 0.1f)
//                        {
//                            AnglePID.Target = absolute_pos_params.absolute_position2;
//                        }
//                        else if(fabsf(AnglePID.Actual - absolute_pos_params.absolute_position2) <= 0.1f)
//                        {
//                            AnglePID.Target = absolute_pos_params.absolute_position1;
//                        }
//                    }
                    
                    /*以下进行速度环PID控制*/
                    velocityPID.Actual = shaft_velocity;    //速度环的实际值
                    PID_Update(&velocityPID);               //调用封装好的函数，一步完成PID计算和更新
                    setPhaseVoltage(velocityPID.Out, 0, electrical_angle);   /* 速度环的输出值给到电机PWM */
                    
                    /*角度环计次分频*/
                    Count1 ++;    //计次自增
                    if (Count1 >= 5)    //如果计次5次，则if成立，即if每隔5ms进一次
                    {
                        Count1 = 0;    //计次清零，便于下次计次
                        
                        /*以下进行角度环PID控制*/
                        AnglePID.Actual = shaft_angle;    //外环为角度环，实际值为角度值
                        PID_Update(&AnglePID);    //调用封装好的函数，一步完成PID计算和更新
                        velocityPID.Target = AnglePID.Out;    //外环的输出值作用于内环的目标值，组成串级PID结构
                    }
                }
                break;
            
            default:
                break;
        }
    }
}

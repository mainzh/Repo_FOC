#ifndef DATA_FORMAT_H
#define DATA_FORMAT_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "bsp_led.h"
#include "in_serial.h"
#include "motor.h"

// 帧头和帧尾定义
#define PACKET_HEADER_HIGH 0xAA
#define PACKET_HEADER_LOW  0x55
#define PACKET_TAIL_HIGH   0x55
#define PACKET_TAIL_LOW    0xAA

// 数据包类型枚举
typedef enum {
    PACKET_TYPE_MOTOR_STATUS       		= 0x01,      // 电机运行状态(伺服状态)参数
    PACKET_TYPE_MOTOR_INFO         		= 0x02,      // 电机信息参数
    PACKET_TYPE_SYSTEM_STATUS      		= 0x03,          // 系统基本运行状态信息
    PACKET_TYPE_SYSTEM_CONTROL     		= 0x04,      // 系统控制命令
    PACKET_TYPE_CURRENT_IQ_PARAM   		= 0x10,      // 电流环Iq参数调节
    PACKET_TYPE_CURRENT_ID_PARAM   		= 0x11,      // 电流环Id参数调节
    PACKET_TYPE_CURRENT_OPEN_PARAM 		= 0x12,      // 电流环开环参数
    PACKET_TYPE_CURRENT_STATUS     		= 0x13,          // 电流环运行状态参数
    PACKET_TYPE_SPEED_CLOSED_PARAM 		= 0x20,      // 速度环闭环参数
    PACKET_TYPE_SPEED_OPEN_PARAM   		= 0x21,      // 速度环开环参数
    PACKET_TYPE_SPEED_STATUS       		= 0x22,          // 速度环运行状态参数
    PACKET_TYPE_POSITION_CLOSED_PARAM 	= 0x30,   // 位置环闭环参数
    PACKET_TYPE_POSITION_OPEN_PARAM 	= 0x31,     // 位置环开环参数
    PACKET_TYPE_POSITION_STATUS    		= 0x32,          // 位置环运行状态参数
	PACKET_TYPE_SIMPLE_SPEED_PARAM = 0x40,      // 简易速度模式参数设置
    PACKET_TYPE_SIMPLE_SPEED_STATUS = 0x41,     // 简易速度模式运行状态
    PACKET_TYPE_ABSOLUTE_POS_PARAM = 0x42,      // 绝对位置模式参数设置
    PACKET_TYPE_ABSOLUTE_POS_STATUS = 0x43,     // 绝对位置模式运行状态
} PacketType;

// 命令字枚举
typedef enum {
    CMD_READ_REQUEST   = 0x01,  // 读请求
    CMD_WRITE_REQUEST  = 0x02,  // 写请求
    CMD_READ_RESPONSE  = 0x03,  // 读响应
    CMD_WRITE_RESPONSE = 0x04,  // 写响应
} CommandType;

// 电机运行状态(伺服状态)枚举（统一用于所有环的运行状态）
typedef enum {
    MOTOR_STOP        = 0x00,  // 电机停止/失能
    MOTOR_ENABLED     = 0x01,  // 电机使能
    MOTOR_FORWARD     = 0x02,  // 正转运行
    MOTOR_REVERSE     = 0x03,  // 反转运行
} MotorRunStatus;

// 控制模式枚举
// typedef enum {
//     CONTROL_MODE_CURRENT_CLOSED = 0x01,  // 电流环-闭环
//     CONTROL_MODE_CURRENT_OPEN   = 0x02,  // 电流环-开环
//     CONTROL_MODE_SPEED_CLOSED   = 0x03,  // 速度环-闭环
//     CONTROL_MODE_SPEED_OPEN     = 0x04,  // 速度环-开环
//     CONTROL_MODE_POSITION_CLOSED = 0x05, // 位置环-闭环
//     CONTROL_MODE_POSITION_OPEN  = 0x06,  // 位置环-开环
// } ControlMode;

// 用户模式枚举
typedef enum {
    USER_MODE_NORMAL    = 0x01,  // 用户模式
    USER_MODE_ENGINEER  = 0x02,  // 工程师模式
} UserMode;

// 系统控制命令枚举
typedef enum {
    SYS_CMD_EMERGENCY_STOP = 0x01,  // 急停
    SYS_CMD_RESET          = 0x02,  // 复位
    SYS_CMD_CLEAR_FAULT    = 0x03,  // 清除故障
} SystemCommand;

// 故障标志位定义
typedef enum {
    FAULT_NONE                     = 0x0000, // 正常
    FAULT_OVER_VOLTAGE             = 0x0001, // 过压
    FAULT_OVER_CURRENT             = 0x0002, // 过流
    FAULT_MOTOR_OVERLOAD           = 0x0004, // 电机过载
    FAULT_UNDER_VOLTAGE            = 0x0008, // 欠压
    FAULT_MOTOR_OVERHEAT           = 0x0010, // 电机过热
    FAULT_MOTOR_STALL              = 0x0020, // 电机堵转
    FAULT_STATE_TRANSITION_ERROR   = 0x0040, // 状态切换异常
    FAULT_MODE_CONFLICT            = 0x0080, // 模式控制冲突
    FAULT_ENABLE_ERROR             = 0x0100, // 使能状态异常
    FAULT_SAFETY_TRIGGER           = 0x0200, // 安全状态触发
    FAULT_SPEED_OVERSHOOT          = 0x0400, // 速度超调
    FAULT_CURRENT_LOOP_ERROR       = 0x0800, // 电流环异常
    FAULT_POSITION_TRACKING_ERROR  = 0x1000, // 位置跟踪误差过大
} FaultFlags;

#pragma pack(push, 1)  // 保存当前对齐设置，设置为1字节对齐

// 电机运行状态(伺服状态)参数结构体
typedef struct {
    uint8_t motor_status;  // 电机运行状态
} MotorStatusParams;

// 电机信息参数结构体
typedef struct {
    char motor_model[32];      // 电机型号
    char motor_name[64];       // 电机名称
    float rated_current;       // 电机额定电流
} MotorInfoParams;


// 系统基本运行状态信息参数结构体
typedef struct {
    uint16_t fault_flags;      // 故障显示
    float bus_voltage;         // 母线电压
    float bus_current;         // 母线电流
    float actual_speed;        // 实际速度
    float actual_position;     // 实际位置
} SystemStatusParams;

// 系统控制命令参数结构体
typedef struct {
    uint8_t system_command;    // 系统控制命令
} SystemControlParams;

// 电流环闭环Iq参数调节结构体
typedef struct {
    float iq_target;           // Iq目标值
    float iq_min_output;       // Iq最小输出值
    float iq_max_output;       // Iq最大输出值
    float iq_kp;               // Iq_kp
    float iq_ki;               // Iq_ki
    float iq_kd;               // Iq_kd
} CurrentIqParams;

// 电流环闭环Id参数调节结构体
typedef struct {
    float id_target;           // Id目标值
    float id_min_output;       // Id最小输出值
    float id_max_output;       // Id最大输出值
    float id_kp;               // Id_kp
    float id_ki;               // Id_ki
    float id_kd;               // Id_kd
} CurrentIdParams;

// 电流环开环参数结构体
typedef struct {
    float electrical_angle;    // 电角度
    float id_target;           // Id目标值
    float iq_target;           // Iq目标值
} CurrentOpenParams;

// 电流环运行状态参数结构体
typedef struct {
    float iq_actual;           // Iq实际值
    float iq_output;           // Iq输出值
    float id_actual;           // Id实际值
    float id_output;           // Id输出值
    uint8_t motor_status;      // 电机运行状态(伺服状态)
    uint16_t system_fault;     // 系统故障
} CurrentStatusParams;

// 速度环闭环参数结构体
typedef struct {
    const float speed_target;        // 速度目标值
    const float speed_min_output;    // 速度最小输出值
    const float speed_max_output;    // 速度最大输出值
    const float kp;                  // kp
    const float ki;                  // ki
    const float kd;                  // kd
} SpeedClosedParams;

// 速度环开环参数结构体
typedef struct {
    float speed_target;        // 速度目标值
    float ud;                  // Ud
    float uq;                  // Uq
} SpeedOpenParams;

// 速度环运行状态参数结构体
typedef struct {
    float speed_actual;        // 速度实际值
    float speed_output;        // 速度输出值
    uint8_t motor_status;      // 电机运行状态(伺服状态)
    uint16_t system_fault;     // 系统故障
} SpeedStatusParams;

// 位置环闭环参数结构体
typedef struct {
    float angle_target;        // 角度目标值
    float angle_min_output;    // 角度最小输出值
    float angle_max_output;    // 角度最大输出值
    float kp;                  // kp
    float ki;                  // ki
    float kd;                  // kd
} PositionClosedParams;

// 位置环开环参数结构体
typedef struct {
    float angle_target;        // 角度目标值
    float speed_target;        // 速度目标值
    float ud;                  // Ud
    float uq;                  // Uq
} PositionOpenParams;

// 位置环运行状态参数结构体
typedef struct {
    float angle_actual;        // 角度实际值
    float angle_output;        // 角度输出值
    uint8_t motor_status;      // 电机运行状态(伺服状态)
    uint16_t system_fault;     // 系统故障
} PositionStatusParams;

// 简易速度模式参数设置结构体
    typedef struct {
    float speed_target;        // 速度目标设定
    float rise_time;           // 上升时间设定
    float fall_time;           // 下降时间设定
    float max_speed;           // 速度最大值
} SimpleSpeedParams;

// 简易速度模式运行状态结构体
typedef struct {
    float position_actual;     // 位置实际值
    float speed_feedback;      // 速度反馈值
    float torque_actual;       // 出力实际值（力矩/扭矩实际值）
    uint8_t servo_status;      // 伺服状态
    uint16_t system_fault;     // 系统故障
} SimpleSpeedStatusParams;

// 绝对位置模式参数设置结构体
typedef struct {
    float run_speed;           // 运行速度设定
    float acceleration_time;   // 运行加减速时间
    float wait_time;           // 运行等待时间
    float absolute_position1;  // 绝对位置目标值1
    float absolute_position2;  // 绝对位置目标值2
    uint16_t cycle_count;      // 循环次数设定
} AbsolutePosParams;

// 绝对位置模式运行状态结构体
typedef struct {
    uint16_t remaining_cycles; // 剩余循环次数
    float position_actual;     // 位置实际值
    float speed_feedback;      // 速度反馈值
    float torque_actual;       // 出力实际值（力矩/扭矩实际值）
    uint8_t servo_status;      // 伺服状态
    uint16_t system_fault;     // 系统故障
} AbsolutePosStatusParams;

// 恢复之前的对齐设置
#pragma pack(pop)

// 数据包结构体
typedef struct {
    uint8_t header[2];         // 帧头(2B): 0xAA, 0x55
    uint8_t data_length;       // 数据长度(1B): 数据域的长度
    uint8_t packet_type;       // 数据包类型(1B)
    uint8_t command;           // 命令字(1B)
    uint8_t* data_field;       // 数据域(NB): 可变长度数据
    uint8_t crc;               // CRC校验(1B)
    uint8_t tail[2];           // 帧尾(2B): 0x55, 0xAA
} DataPacket;

// 函数声明
#ifdef __cplusplus
extern "C" {
#endif

// 计算CRC校验
uint8_t calculate_crc(const uint8_t* data, uint8_t length);

// 创建数据包
DataPacket* create_packet(uint8_t packet_type, uint8_t command, const void* data, uint8_t data_size);

// 释放数据包内存
void free_packet(DataPacket* packet);

// 验证数据包
int validate_packet(const DataPacket* packet);

// 序列化数据包到字节流
uint8_t* serialize_packet(const DataPacket* packet, uint8_t* total_length);

// 从字节流反序列化数据包
DataPacket* deserialize_packet(const uint8_t* buffer, uint8_t length);

	
void process_uart_receive(void);//串口数据接收处理
void process_command(uint8_t* received_packet);// 在process_command函数中处理特定命令

DataPacket* create_system_status_response(void);	// 创建系统基本运行状态响应包
DataPacket* create_current_status_response(void);	// 创建电流环运行状态响应包	
DataPacket* create_speed_status_response(void);		// 创建速度环运行状态响应包
DataPacket* create_position_status_response(void);	// 创建位置环运行状态响应包
// 函数声明中添加
DataPacket* create_simple_speed_status_response(void);    // 创建简易速度模式运行状态响应包
DataPacket* create_absolute_pos_status_response(void);    // 创建绝对位置模式运行状态响应包

//DataPacket* create_motor_status_response(void);


void process_command(uint8_t* received_packet);// 在process_command函数中处理特定命令




DataPacket* handle_read_request(uint8_t packet_type);// 处理读请求
void handle_write_command(uint8_t packet_type, uint8_t* data_field, uint8_t data_length);// 处理写请求 - 将接收到的数据解析到对应的结构体
DataPacket* create_write_response(uint8_t packet_type);// 创建写响应包

// 全局结构体变量声明
extern CurrentIqParams current_iq_params;
extern CurrentIdParams current_id_params;
extern CurrentOpenParams current_open_params;
extern SpeedClosedParams speed_closed_params;
extern SpeedOpenParams speed_open_params;
extern SpeedStatusParams speed_status_params;
extern PositionClosedParams position_closed_params;
extern PositionOpenParams position_open_params;
extern PositionStatusParams position_status_params;
extern MotorStatusParams motor_status_params;
extern SystemStatusParams system_status_params;

// 添加新的全局变量声明
extern SimpleSpeedParams        simple_speed_params;
extern SimpleSpeedStatusParams  simple_speed_status_params;
extern AbsolutePosParams        absolute_pos_params;
extern AbsolutePosStatusParams  absolute_pos_status_params;

extern uint8_t rx_buffer[];

extern uint8_t packet_flag;
extern uint8_t motor_flag;

#ifdef __cplusplus
}
#endif

#endif // DATA_FORMAT_H

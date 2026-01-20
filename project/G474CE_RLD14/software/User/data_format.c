#include "data_format.h"

// 全局结构体变量定义
CurrentIqParams current_iq_params = {0};
CurrentIdParams current_id_params = {0};
CurrentOpenParams current_open_params = {0};
CurrentStatusParams current_status_params= {0};
SpeedClosedParams speed_closed_params = {0};
SpeedOpenParams speed_open_params = {0};
SpeedStatusParams speed_status_params = {0};
PositionClosedParams position_closed_params = {0};
PositionOpenParams position_open_params = {0};
PositionStatusParams position_status_params = {0};
MotorStatusParams motor_status_params = {0};
SystemStatusParams system_status_params = {0};

SimpleSpeedParams        simple_speed_params = {0};
SimpleSpeedStatusParams  simple_speed_status_params = {0};
AbsolutePosParams        absolute_pos_params = {0};
AbsolutePosStatusParams  absolute_pos_status_params = {0};

uint8_t packet_flag;
uint8_t motor_flag;

typedef enum {
    STATE_IDLE = 0,
    STATE_HEADER_RECEIVED,
    STATE_PACKET_COMPLETE
} uart_state_t;

uart_state_t uart_state = STATE_IDLE;
uint8_t rx_buffer[32] = {0};
uint8_t expected_length = 0;
uint8_t received_count = 0;


/**
 * @brief 计算CRC8校验值
 * @param data 待校验数据指针
 * @param length 数据长度
 * @return CRC8校验值
 */
 //CRC8多项式: x8+x2+x+1 (0x07)
uint8_t calculate_crc(const uint8_t* data, uint8_t length)
{
    uint8_t crc = 0x00;
    uint8_t i;
    
    while(length--)
    {
        crc ^= *data++;
        for(i = 0; i < 8; i++)
        {
            if(crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// 创建数据包
DataPacket* create_packet(uint8_t packet_type, uint8_t command, const void* data, uint8_t data_size) {
    DataPacket* packet = (DataPacket*)malloc(sizeof(DataPacket));
    if (!packet) return NULL;
    
    // 设置帧头帧尾
    packet->header[0] = PACKET_HEADER_HIGH;
    packet->header[1] = PACKET_HEADER_LOW;
    packet->tail[0] = PACKET_TAIL_HIGH;
    packet->tail[1] = PACKET_TAIL_LOW;
    
    // 设置包类型和命令
    packet->packet_type = packet_type;
    packet->command = command;
    packet->data_length = data_size;
    
    // 分配并拷贝数据域
    if (data_size > 0 && data != NULL) {
        packet->data_field = (uint8_t*)malloc(data_size);
        if (packet->data_field) {
            memcpy(packet->data_field, data, data_size);
        }
    } else {
        packet->data_field = NULL;
    }
    
    // 计算CRC（从packet_type开始到data_field结束）
    uint8_t crc_data_length = 2 + data_size; // packet_type + command + data_field
    uint8_t* crc_data = (uint8_t*)malloc(crc_data_length);
    if (crc_data) {
        crc_data[0] = packet_type;
        crc_data[1] = command;
        if (data_size > 0 && packet->data_field) {
            memcpy(&crc_data[2], packet->data_field, data_size);
        }
        packet->crc = calculate_crc(crc_data, crc_data_length);
        free(crc_data);
    }
    
    return packet;
}

// 释放数据包内存
void free_packet(DataPacket* packet) {
    if (packet) {
        if (packet->data_field) {
            free(packet->data_field);
        }
        free(packet);
    }
}

// 验证数据包
int validate_packet(const DataPacket* packet) {
    if (!packet) return 0;
    
    // 检查帧头帧尾
    if (packet->header[0] != PACKET_HEADER_HIGH || 
        packet->header[1] != PACKET_HEADER_LOW ||
        packet->tail[0] != PACKET_TAIL_HIGH || 
        packet->tail[1] != PACKET_TAIL_LOW) {
        return 0;
    }
    
    // 计算并验证CRC
    uint8_t crc_data_length = 2 + packet->data_length; // packet_type + command + data_field
    uint8_t* crc_data = (uint8_t*)malloc(crc_data_length);
    if (!crc_data) return 0;
    
    crc_data[0] = packet->packet_type;
    crc_data[1] = packet->command;
    if (packet->data_length > 0 && packet->data_field) {
        memcpy(&crc_data[2], packet->data_field, packet->data_length);
    }
    
    uint8_t calculated_crc = calculate_crc(crc_data, crc_data_length);
    free(crc_data);
    
    return (calculated_crc == packet->crc);
}

// 序列化数据包到字节流(组包)
uint8_t* serialize_packet(const DataPacket* packet, uint8_t* total_length) {
    // 计算总长度: header(2) + data_length(1) + packet_type(1) + command(1) + data_field(N) + crc(1) + tail(2)
    uint8_t total_len = 8 + packet->data_length;
    uint8_t* buffer = (uint8_t*)malloc(total_len);
    
    if(buffer)
    {
        uint8_t index = 0;
        
        // 帧头
        buffer[index++] = packet->header[0];
        buffer[index++] = packet->header[1];
        
        // 数据长度
        buffer[index++] = packet->data_length;
        
        // 数据包类型
        buffer[index++] = packet->packet_type;
        
        // 命令字
        buffer[index++] = packet->command;
        
        // 数据域
        if(packet->data_length > 0 && packet->data_field)
        {
            memcpy(&buffer[index], packet->data_field, packet->data_length);
            index += packet->data_length;
        }
        
        // CRC校验
        buffer[index++] = packet->crc;
        
        // 帧尾
        buffer[index++] = packet->tail[0];
        buffer[index++] = packet->tail[1];
        
        *total_length = total_len;
    }
    
    return buffer;
}

// 从字节流反序列化数据包(拆包)
DataPacket* deserialize_packet(const uint8_t* buffer, uint8_t length) {
    if (length < 8) return NULL; // 最小包长度
    
    DataPacket* packet = (DataPacket*)malloc(sizeof(DataPacket));
    if (!packet) return NULL;
    
    uint8_t index = 0;
    
    // 帧头
    packet->header[0] = buffer[index++];
    packet->header[1] = buffer[index++];
    
    // 数据长度
    packet->data_length = buffer[index++];
    
    // 包类型和命令
    packet->packet_type = buffer[index++];
    packet->command = buffer[index++];
    
    // 数据域
    if (packet->data_length > 0) {
        packet->data_field = (uint8_t*)malloc(packet->data_length);
        if (packet->data_field) {
            memcpy(packet->data_field, &buffer[index], packet->data_length);
            index += packet->data_length;
        }
    } else {
        packet->data_field = NULL;
    }
    
    // CRC
    packet->crc = buffer[index++];
    
    // 帧尾
    packet->tail[0] = buffer[index++];
    packet->tail[1] = buffer[index++];
    
    return packet;
}

/* 串口数据接收处理 */
void process_uart_receive(void)
{
    switch(uart_state)
    {
        case STATE_IDLE:
            // 等待接收包头
        RS485DIR_RX
            if(HAL_OK == HAL_UART_Receive(&huart1, rx_buffer, 5,200))
            {
                if(rx_buffer[0] == 0xAA && rx_buffer[1] == 0x55)
                {
                    uint8_t data_field_length = rx_buffer[2];
                    expected_length = 8 + data_field_length; // 总包长 = 8 + 数据域长度
                    received_count = 5;
                    uart_state = STATE_HEADER_RECEIVED;
                }
            }
            break;
            
        case STATE_HEADER_RECEIVED:
            // 接收剩余数据
        RS485DIR_RX
            if(HAL_OK == HAL_UART_Receive(&huart1, &rx_buffer[received_count], 
                                          expected_length - received_count,200))
            {
                received_count = expected_length;
                uart_state = STATE_PACKET_COMPLETE;
            }
            break;
            
        case STATE_PACKET_COMPLETE:
            // 验证帧尾并处理数据包
            if(rx_buffer[expected_length-2] == 0x55 && rx_buffer[expected_length-1] == 0xAA)
            {
				//验证CRC
				uint8_t 	data_field_length = rx_buffer[2];
				uint8_t 	crc_data_length = 2+data_field_length;// packet_type + command + data_field
				uint8_t *	crc_data = &rx_buffer[3];//从packet_type开始
				uint8_t		calculated_crc = calculate_crc(crc_data,crc_data_length);
				uint8_t		received_crc = rx_buffer[5+data_field_length];
				
				if(calculated_crc == received_crc)
				{
//					printf("111");
					// 发送回显到串口调试助手
//					HAL_UART_Transmit(&huart1, rx_buffer, expected_length, 100);
					
					// 根据数据包类型处理不同命令
					process_command(rx_buffer);
//					printf("222");
				}
				else
				{
					// CRC验证失败
//                    printf("CRC check failed! Calculated: 0x%02X, Received: 0x%02X\n", 
//                           calculated_crc, received_crc);
				}
				
            }
            
            // 重置状态，准备接收下一个数据包
            uart_state = STATE_IDLE;
            memset(rx_buffer, 0, sizeof(rx_buffer));
            break;
    }
}

// 在process_command函数中处理特定命令
void process_command(uint8_t* received_packet)
{
    // 解析接收到的数据包
    uint8_t packet_type = received_packet[3];
    uint8_t command = received_packet[4];
    uint8_t data_field_length = received_packet[2];
    uint8_t* data_field = &received_packet[5]; // 数据域起始位置
    
    DataPacket* response_packet = NULL;
    uint8_t* serialized_data = NULL;
    uint8_t serialized_length = 0;
    
    // 根据命令类型处理
    switch(command)
    {
        case CMD_READ_REQUEST:
			
            // 处理读请求
            response_packet = handle_read_request(packet_type);
			
            break;
            
        case CMD_WRITE_REQUEST:
            // 处理写请求
            handle_write_command(packet_type, data_field, data_field_length);
            // 可以发送写响应确认
            response_packet = create_write_response(packet_type);
            break;
            
        default:
            // 其他命令暂不处理
            break;
    }
    
    // 如果创建了响应包，则序列化并发送
    if(response_packet)
    {
        serialized_data = serialize_packet(response_packet, &serialized_length);
        if(serialized_data)
        {
//			printf("555");
            // 通过串口发送响应
            RS485DIR_TX
            HAL_UART_Transmit(&huart1, serialized_data, serialized_length, 100);
            free(serialized_data);
        }
        free_packet(response_packet);
    }
}

// 处理读请求
DataPacket* handle_read_request(uint8_t packet_type)
{
    switch(packet_type)
    {
        case PACKET_TYPE_CURRENT_STATUS: // 0x13 - 电流环运行状态
			
            return create_current_status_response();
            
        case PACKET_TYPE_SYSTEM_STATUS: // 0x03 - 系统基本运行状态
			
            return create_system_status_response();
            
        case PACKET_TYPE_SPEED_STATUS: // 0x22 - 速度环运行状态
            return create_speed_status_response();
            
        case PACKET_TYPE_POSITION_STATUS: // 0x32 - 位置环运行状态
            return create_position_status_response();
            
		case PACKET_TYPE_SIMPLE_SPEED_STATUS: // 0x41 - 简易速度模式运行状态
            return create_simple_speed_status_response();
            
        case PACKET_TYPE_ABSOLUTE_POS_STATUS: // 0x43 - 绝对位置模式运行状态
            return create_absolute_pos_status_response();
		
//        case PACKET_TYPE_MOTOR_STATUS: // 0x01 - 电机运行状态
//            return create_motor_status_response();
            
        default:
            return NULL;
    }
}

// 处理写请求 - 将接收到的数据解析到对应的结构体
void handle_write_command(uint8_t packet_type, uint8_t* data_field, uint8_t data_length)
{
    switch(packet_type)
    {
        case PACKET_TYPE_CURRENT_IQ_PARAM: // 0x10 - 电流环Iq参数
            packet_flag = PACKET_TYPE_CURRENT_IQ_PARAM;
            if(data_length == sizeof(CurrentIqParams))
            {
                memcpy(&current_iq_params, data_field, sizeof(CurrentIqParams));
                // 这里可以添加参数应用逻辑
                printf("Current Iq params updated.Iq_target = %f,  iq_kp = %f\n",current_iq_params.iq_target,current_iq_params.iq_kp);
            }
            break;
            
        case PACKET_TYPE_CURRENT_ID_PARAM: // 0x11 - 电流环Id参数
            packet_flag = PACKET_TYPE_CURRENT_ID_PARAM;
            if(data_length == sizeof(CurrentIdParams))
            {
                memcpy(&current_id_params, data_field, sizeof(CurrentIdParams));
                printf("Current Id params updated.\n");
            }
            break;
            
        case PACKET_TYPE_CURRENT_OPEN_PARAM: // 0x12 - 电流环开环参数
            packet_flag = PACKET_TYPE_CURRENT_OPEN_PARAM;
            if(data_length == sizeof(CurrentOpenParams))
            {
                memcpy(&current_open_params, data_field, sizeof(CurrentOpenParams));
                printf("Current open params updated.\n");
            }
            break;
            
        case PACKET_TYPE_SPEED_CLOSED_PARAM: // 0x20 - 速度环闭环参数
            packet_flag = PACKET_TYPE_SPEED_CLOSED_PARAM;
            if(data_length == sizeof(SpeedClosedParams))
            {
                memcpy(&speed_closed_params, data_field, sizeof(SpeedClosedParams));
                
                /* 上位机参数赋值 */
                velocityPID.Target = speed_closed_params.speed_target;              //速度环目标值初值设定
                
                velocityPID.Kp = speed_closed_params.kp;                         //比例项权重，PID系数的数量级一般由输出范围和输入范围的比值确定，即 Actual / Out
                velocityPID.Ki = speed_closed_params.ki;                         //积分项权重
                velocityPID.Kd = speed_closed_params.kd;                         //微分项权重
                
                velocityPID.OutMax = speed_closed_params.speed_max_output;                    //输出限幅的最大值
                velocityPID.OutMin = speed_closed_params.speed_min_output;                   //输出限幅的最小值
                
                printf("Speed closed params updated.\n");
            }
            break;
            
        case PACKET_TYPE_SPEED_OPEN_PARAM: // 0x21 - 速度环开环参数
            packet_flag = PACKET_TYPE_SPEED_OPEN_PARAM;
            if(data_length == sizeof(SpeedOpenParams))
            {
                memcpy(&speed_open_params, data_field, sizeof(SpeedOpenParams));
                printf("Speed open params updated.\n");
            }
            break;
            
        case PACKET_TYPE_POSITION_CLOSED_PARAM: // 0x30 - 位置环闭环参数
            packet_flag = PACKET_TYPE_POSITION_CLOSED_PARAM;
            if(data_length == sizeof(PositionClosedParams))
            {
                memcpy(&position_closed_params, data_field, sizeof(PositionClosedParams));
                
                /* 上位机参数赋值 */
                AnglePID.Target = position_closed_params.angle_target;
                
                AnglePID.Kp = position_closed_params.kp;
                AnglePID.Ki = position_closed_params.ki;
                AnglePID.Kd = position_closed_params.kd;
                
                AnglePID.OutMax = position_closed_params.angle_max_output;
                AnglePID.OutMin = position_closed_params.angle_min_output;
                
                printf("Position closed params updated.\n");
            }
            break;
            
        case PACKET_TYPE_POSITION_OPEN_PARAM: // 0x31 - 位置环开环参数
            packet_flag = PACKET_TYPE_POSITION_OPEN_PARAM;
            if(data_length == sizeof(PositionOpenParams))
            {
                memcpy(&position_open_params, data_field, sizeof(PositionOpenParams));
                printf("Position open params updated.\n");
            }
            break;
            
        case PACKET_TYPE_MOTOR_STATUS: // 0x01 - 电机运行状态
            
            if(data_length == sizeof(MotorStatusParams))
            {
                memcpy(&motor_status_params, data_field, sizeof(MotorStatusParams));
                
                if(motor_status_params.motor_status == MOTOR_STOP)
                {
                    motor_flag = MOTOR_STOP;
                    
//                    memset(&current_status_params, 0, sizeof(current_status_params));
//					memset(&speed_status_params, 0, sizeof(speed_status_params));
//					memset(&position_status_params, 0, sizeof(position_status_params));
//					memset(&system_status_params, 0, sizeof(system_status_params));
//					memset(&simple_speed_status_params, 0, sizeof(simple_speed_status_params));
//                    memset(&absolute_pos_status_params, 0, sizeof(absolute_pos_status_params));
                    
                    motor_MOS(0); /* 电机MOS失能 */
                    
                }
                else if(motor_status_params.motor_status == MOTOR_ENABLED)
                {
                    motor_flag = MOTOR_ENABLED;
                    
                    motor_MOS(1); /* 电机MOS使能 */
                    
                }
                else if(motor_status_params.motor_status == MOTOR_FORWARD)
                {
                    motor_flag = MOTOR_FORWARD;
                    
                    switch(packet_flag)
                    {
                        case PACKET_TYPE_SPEED_CLOSED_PARAM:
                            velocityPID.Target = speed_closed_params.speed_target;              //速度环目标值初值设定
                        break;
                    }
                }
                else if(motor_status_params.motor_status == MOTOR_REVERSE)
                {
                    motor_flag = MOTOR_REVERSE;
                    
                    switch(packet_flag)
                    {
                        case PACKET_TYPE_SPEED_CLOSED_PARAM:
                            velocityPID.Target = - speed_closed_params.speed_target;              //速度环目标值初值设定
                        break;
                    }
                }
                
                printf("Motor status updated: 0x%02X\n", motor_status_params.motor_status);
                
            }
            break;
            
            case PACKET_TYPE_SIMPLE_SPEED_PARAM: // 0x40 - 简易速度模式参数设置
            if(data_length == sizeof(SimpleSpeedParams))
            {
                memcpy(&simple_speed_params, data_field, sizeof(SimpleSpeedParams));
                // 这里可以添加参数应用逻辑，比如设置速度环的目标值
            }
            break;
            
        case PACKET_TYPE_ABSOLUTE_POS_PARAM: // 0x42 - 绝对位置模式参数设置
            packet_flag = PACKET_TYPE_ABSOLUTE_POS_PARAM;
            if(data_length == sizeof(AbsolutePosParams))
            {
                memcpy(&absolute_pos_params, data_field, sizeof(AbsolutePosParams));
                // 这里可以添加参数应用逻辑，比如启动位置控制
                
                /* 上位机参数赋值 */
//                AnglePID.OutMax = absolute_pos_params.run_speed;
//                AnglePID.OutMin = - absolute_pos_params.run_speed;
//                
            }
			break;
            
        default:
            // 未知的写命令类型
            break;
    }
}
// 创建写响应包
DataPacket* create_write_response(uint8_t packet_type)
{
    // 简单的写响应，通常只包含成功标志
    uint8_t success_flag = 0x01; // 1表示成功
    
    return create_packet(packet_type, 
                        CMD_WRITE_RESPONSE, 
                        &success_flag, 
                        sizeof(success_flag));
}

uint8_t fault_flag = 0;

// 创建电流环运行状态响应包
DataPacket* create_current_status_response(void)
{
    fault_flag = 1;
    
    // 设置参数值（根据您的实际数据）
    current_status_params.iq_actual = 1.23f;
    current_status_params.iq_output = 1.24f;
    current_status_params.id_actual = 1.25f;
    current_status_params.id_output = 1.26f;
    current_status_params.motor_status = motor_status_params.motor_status; // 0x01
    current_status_params.system_fault = FAULT_NONE;    // 0x0000
//    printf("333");
    // 创建数据包
    return create_packet(PACKET_TYPE_CURRENT_STATUS, 
                        CMD_READ_RESPONSE, 
                        &current_status_params, 
                        sizeof(CurrentStatusParams));
}

// 创建系统基本运行状态响应包
DataPacket* create_system_status_response(void)
{
    
    
    // 设置参数值（根据您的实际数据）
//    system_status_params .fault_flags = 0x0000;      // 故障显示
	if(fault_flag == 1)system_status_params .fault_flags = current_status_params.system_fault;      // 故障显示
	if(fault_flag == 2)system_status_params .fault_flags = speed_status_params.system_fault;      // 故障显示
	if(fault_flag == 3)system_status_params .fault_flags = position_status_params.system_fault;      // 故障显示
	if(fault_flag == 4)system_status_params .fault_flags = simple_speed_status_params.system_fault;
	if(fault_flag == 5)system_status_params .fault_flags = absolute_pos_status_params.system_fault;
    
    system_status_params .bus_voltage = 2.5f;        // 母线电压
    system_status_params .bus_current = 2.6f;        // 母线电流
    system_status_params .actual_speed = shaft_velocity;     // 实际速度
    system_status_params .actual_position = shaft_angle;  // 实际位置
//    printf("444");
    // 创建数据包
    return create_packet(PACKET_TYPE_SYSTEM_STATUS, 
                        CMD_READ_RESPONSE, 
                        &system_status_params , 
                        sizeof(SystemStatusParams));
}
//创建速度环运行状态响应包
DataPacket* create_speed_status_response(void)
{
	fault_flag = 2;
    
	speed_status_params.speed_actual = shaft_velocity;//实际速度
	speed_status_params.speed_output = velocityPID.Out;//速度输出值
	speed_status_params.motor_status = motor_status_params.motor_status;//
	speed_status_params.system_fault = FAULT_NONE;//无故障
	
	return create_packet(PACKET_TYPE_SPEED_STATUS,
						CMD_READ_RESPONSE,
						&speed_status_params,
						sizeof(SpeedStatusParams));
}


// 创建位置环运行状态响应包
DataPacket* create_position_status_response(void)
{
    fault_flag = 3;
    
    position_status_params.angle_actual = shaft_angle;     // 实际角度
    position_status_params.angle_output = AnglePID.Out;     // 角度输出值
    position_status_params.motor_status = motor_status_params.motor_status; // 
    position_status_params.system_fault = FAULT_NONE;    // 无故障
    
    return create_packet(PACKET_TYPE_POSITION_STATUS, 
                        CMD_READ_RESPONSE, 
                        &position_status_params, 
                        sizeof(PositionStatusParams));
}

// 创建简易速度模式运行状态响应包
DataPacket* create_simple_speed_status_response(void)
{
    // 更新状态数据（这里需要根据实际系统状态更新）
    simple_speed_status_params.position_actual = shaft_angle;    // 位置实际值
    simple_speed_status_params.speed_feedback = shaft_velocity;       // 速度反馈值
    simple_speed_status_params.torque_actual = simple_speed_status_params.torque_actual+0.5f;         // 力矩实际值（需要根据实际系统）
    simple_speed_status_params.servo_status = motor_status_params.motor_status;         // 伺服状态
    simple_speed_status_params.system_fault = FAULT_NONE ;//过压         // 系统故障
    
    
    return create_packet(PACKET_TYPE_SIMPLE_SPEED_STATUS, 
                        CMD_READ_RESPONSE, 
                        &simple_speed_status_params, 
                        sizeof(SimpleSpeedStatusParams));
}

// 创建绝对位置模式运行状态响应包
DataPacket* create_absolute_pos_status_response(void)
{

    absolute_pos_status_params.remaining_cycles = absolute_pos_status_params.remaining_cycles;	//剩余循环次数
    absolute_pos_status_params.position_actual = shaft_angle;	// 位置实际值
    absolute_pos_status_params.speed_feedback = AnglePID.Out;		// 速度反馈值
    absolute_pos_status_params.torque_actual = absolute_pos_status_params.torque_actual+0.5f;		// 力矩实际值
    absolute_pos_status_params.servo_status = motor_status_params.motor_status;       				// 伺服状态
    absolute_pos_status_params.system_fault = FAULT_NONE ;    // 过流         						// 系统故障

    
    return create_packet(PACKET_TYPE_ABSOLUTE_POS_STATUS, 
                        CMD_READ_RESPONSE, 
                        &absolute_pos_status_params, 
                        sizeof(AbsolutePosStatusParams));
}

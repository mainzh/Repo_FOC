/**
 ******************************************************************************
 * @file    mu_1sf_driver.c
 * @author  iC-Haus GmbH
 * @version 1.1.0
 * @note Designed according to iC-MU datasheet release F2 for chip revision Y2/Y2H.
 * @note Designed according to iC-MU150 datasheet release D2 for chip revision 1.
 * @note Designed according to iC-MU200 datasheet release B2 for chip revision 0.
 ******************************************************************************
 * @attention
 *
 *	Software and its documentation is provided by iC-Haus GmbH or contributors "AS IS" and is
 *	subject to the ZVEI General Conditions for the Supply of Products and Services with iC-Haus
 *	amendments and the ZVEI Software clause with iC-Haus amendments (http://www.ichaus.de/EULA).
 *
 ******************************************************************************
 */

#include "mu_1sf_driver.h"

/* iC-MU opcodes 操作码*/
enum MU_OPCODE {
	MU_OPCODE_ACTIVATE = 0xB0,
	MU_OPCODE_SDAD_TRANSMISSION = 0xA6,
	MU_OPCODE_SDAD_STATUS = 0xF5,
	MU_OPCODE_READ_REGISTER = 0x97,
	MU_OPCODE_WRITE_REGISTER = 0xD2,
	MU_OPCODE_REGISTER_STATUS_DATA = 0xAD,
};

/* iC-MU parameters （iC-MU参数）*/
const struct mu_param MU_GF_M = { .bank = 0x00, .addr = { 0x00 }, .pos = 5, .len = 6 };
const struct mu_param MU_GC_M = { .bank = 0x00, .addr = { 0x00 }, .pos = 7, .len = 2 };
const struct mu_param MU_GX_M =  { .bank = 0x00, .addr = { 0x01 }, .pos = 6, .len = 7 };
const struct mu_param MU_VOSS_M = { .bank = 0x00, .addr = { 0x02 }, .pos = 6, .len = 7 };
const struct mu_param MU_VOSC_M = { .bank = 0x00, .addr = { 0x03 }, .pos = 6, .len = 7 };
const struct mu_param MU_PH_M = { .bank = 0x00, .addr = { 0x04 }, .pos = 6, .len = 7 };
const struct mu_param MU_CIBM = { .bank = 0x00, .addr = { 0x05 }, .pos = 3, .len = 4 };
const struct mu_param MU_ENAC = { .bank = 0x00, .addr = { 0x05 }, .pos = 7, .len = 1 };
const struct mu_param MU_GF_N = { .bank = 0x00, .addr = { 0x06 }, .pos = 5, .len = 6 };
const struct mu_param MU_GC_N = { .bank = 0x00, .addr = { 0x06 }, .pos = 7, .len = 2 };
const struct mu_param MU_GX_N = { .bank = 0x00, .addr = { 0x07 }, .pos = 6, .len = 7 };
const struct mu_param MU_VOSS_N = { .bank = 0x00, .addr = { 0x08 }, .pos = 6, .len = 7 };
const struct mu_param MU_VOSC_N = { .bank = 0x00, .addr = { 0x09 }, .pos = 6, .len = 7 };
const struct mu_param MU_PH_N = { .bank = 0x00, .addr = { 0x0A }, .pos = 6, .len = 7 };
const struct mu_param MU_MODEA = { .bank = 0x00, .addr = { 0x0B }, .pos = 2, .len = 3 };
const struct mu_param MU_MODEB = { .bank = 0x00, .addr = { 0x0B }, .pos = 6, .len = 3 };
const struct mu_param MU_CFGEW = { .bank = 0x00, .addr = { 0x0C }, .pos = 7, .len = 8 };
const struct mu_param MU_EMTD = { .bank = 0x00, .addr = { 0x0D }, .pos = 2, .len = 3 };
const struct mu_param MU_ACRM_RES = { .bank = 0x00, .addr = { 0x0D }, .pos = 4, .len = 1 };
const struct mu_param MU_NCHK_NON = { .bank = 0x00, .addr = { 0x0D }, .pos = 5, .len = 1 };
const struct mu_param MU_NCHK_CRC = { .bank = 0x00, .addr = { 0x0D }, .pos = 6, .len = 1 };
const struct mu_param MU_ACC_STAT = { .bank = 0x00, .addr = { 0x0D }, .pos = 7, .len = 1 };
const struct mu_param MU_FILT = { .bank = 0x00, .addr = { 0x0E }, .pos = 2, .len = 3 };
const struct mu_param MU_LIN = { .bank = 0x00, .addr = { 0x0E }, .pos = 4, .len = 1 };
const struct mu_param MU_ROT_MT = { .bank = 0x00, .addr = { 0x0E }, .pos = 5, .len = 1 };
const struct mu_param MU_ESSI_MT = { .bank = 0x00, .addr = { 0x0E }, .pos = 7, .len = 2 };
const struct mu_param MU_MPC = { .bank = 0x00, .addr = { 0x0F }, .pos = 3, .len = 4 };
const struct mu_param MU_SPO_MT = { .bank = 0x00, .addr = { 0x0F }, .pos = 7, .len = 4 };
const struct mu_param MU_MODE_MT = { .bank = 0x00, .addr = { 0x10 }, .pos = 3, .len = 4 };
const struct mu_param MU_SBL_MT = { .bank = 0x00, .addr = { 0x10 }, .pos = 5, .len = 2 };
const struct mu_param MU_CHK_MT = { .bank = 0x00, .addr = { 0x10 }, .pos = 6, .len = 1 };
const struct mu_param MU_GET_MT = { .bank = 0x00, .addr = { 0x10 }, .pos = 7, .len = 1 };
const struct mu_param MU_OUT_MSB = { .bank = 0x00, .addr = { 0x11 }, .pos = 4, .len = 5 };
const struct mu_param MU_OUT_ZERO = { .bank = 0x00, .addr = { 0x11 }, .pos = 7, .len = 3 };
const struct mu_param MU_OUT_LSB = { .bank = 0x00, .addr = { 0x12 }, .pos = 3, .len = 4 };
const struct mu_param MU_MODE_ST = { .bank = 0x00, .addr = { 0x12 }, .pos = 5, .len = 2 };
const struct mu_param MU_RSSI = { .bank = 0x00, .addr = { 0x12 }, .pos = 6, .len = 1 };
const struct mu_param MU_GSSI = { .bank = 0x00, .addr = { 0x12 }, .pos = 7, .len = 1 };
const struct mu_param MU_RESABZ = { .bank = 0x00, .addr = { 0x14, 0x13 }, .pos = 7, .len = 16 };
const struct mu_param MU_FRQAB = { .bank = 0x00, .addr = { 0x15 }, .pos = 2, .len = 3 };
const struct mu_param MU_ENIF_AUTO = { .bank = 0x00, .addr = { 0x15 }, .pos = 3, .len = 1 };
const struct mu_param MU_SS_AB = { .bank = 0x00, .addr = { 0x15 }, .pos = 5, .len = 2 };
const struct mu_param MU_ROT_ALL = { .bank = 0x00, .addr = { 0x15 }, .pos = 7, .len = 1 };
const struct mu_param MU_INV_Z = { .bank = 0x00, .addr = { 0x16 }, .pos = 0, .len = 1 };
const struct mu_param MU_INV_B = { .bank = 0x00, .addr = { 0x16 }, .pos = 1, .len = 1 };
const struct mu_param MU_INV_A = { .bank = 0x00, .addr = { 0x16 }, .pos = 2, .len = 1 };
const struct mu_param MU_PP60UVW =  { .bank = 0x00,.addr = { 0x16 }, .pos = 3, .len = 1 };
const struct mu_param MU_CHYS_AB = { .bank = 0x00, .addr = { 0x16 }, .pos = 5, .len = 2 };
const struct mu_param MU_LENZ = { .bank = 0x00, .addr = { 0x16 }, .pos = 7, .len = 2 };
const struct mu_param MU_PPUVW = { .bank = 0x00, .addr = { 0x17 }, .pos = 5, .len = 6 };
const struct mu_param MU_RPL = { .bank = 0x00, .addr = { 0x17 }, .pos = 7, .len = 2 };
const struct mu_param MU_TEST = { .bank = 0x00,  .addr = { 0x18 }, .pos = 7, .len = 8 };
const struct mu_param MU_OFF_ABZ = { .bank = 0x00, .addr = { 0x4A, 0x49, 0x48, 0x1F, 0x1E }, .pos = 7, .len = 36 };
const struct mu_param MU_OFF_POS = { .bank = 0x00, .addr = { 0x22, 0x21, 0x20 }, .pos = 7, .len = 24 };
const struct mu_param MU_OFF_COM = { .bank = 0x00, .addr = { 0x24, 0x23 }, .pos= 7, .len = 12 };
const struct mu_param MU_PA0_CONF = { .bank = 0x00, .addr = { 0x25 }, .pos = 7, .len = 8 };
const struct mu_param MU_AFGAIN_M = { .bank = 0x00, .addr = { 0x2B }, .pos = 2, .len = 3 };
const struct mu_param MU_ACGAIN_M = { .bank = 0x00, .addr = { 0x2B }, .pos = 4, .len = 2 };
const struct mu_param MU_AFGAIN_N = { .bank = 0x00, .addr = { 0x2F }, .pos = 2, .len = 3 };
const struct mu_param MU_ACGAIN_N = { .bank = 0x00, .addr = { 0x2F }, .pos = 4, .len = 2 };
const struct mu_param MU_BANKSEL = { .addr = { 0x40 }, .pos = 4, .len = 5 };
const struct mu_param MU_EDSBANK = { .addr = { 0x41 }, .pos = 7, .len = 8 };
const struct mu_param MU_PROFILE_ID = { .addr = { 0x42, 0x43 }, .pos = 7, .len = 16 };
const struct mu_param MU_SERIAL = { .addr = { 0x44, 0x45, 0x46, 0x47 }, .pos = 7, .len = 32 };
const struct mu_param MU_OFF_UVW = { .addr = { 0x4C, 0x4B }, .pos = 7, .len = 12 };
const struct mu_param MU_PRES_POS = { .addr = { 0x51, 0x50, 0x4F, 0x4E, 0x4D }, .pos = 7, .len = 36 };
const struct mu_param MU_SPO_BASE = { .addr = { 0x52 }, .pos = 3, .len = 4 };
const struct mu_param MU_SPO_0 = { .addr = { 0x52 }, .pos = 7, .len = 4 };
const struct mu_param MU_SPO_1 = { .addr = { 0x53 }, .pos = 3, .len = 4 };
const struct mu_param MU_SPO_2 = { .addr = { 0x53 }, .pos = 7, .len = 4 };
const struct mu_param MU_SPO_3 = { .addr = { 0x54 }, .pos = 3, .len = 4 };
const struct mu_param MU_SPO_4 = { .addr = { 0x54 }, .pos = 7, .len = 4 };
const struct mu_param MU_SPO_5 = { .addr = { 0x55 }, .pos = 3, .len = 4 };
const struct mu_param MU_SPO_6 = { .addr = { 0x55 }, .pos = 7, .len = 4 };
const struct mu_param MU_SPO_7 = { .addr = { 0x56 }, .pos = 3, .len = 4 };
const struct mu_param MU_SPO_8 = { .addr = { 0x56 }, .pos = 7, .len = 4 };
const struct mu_param MU_SPO_9 = { .addr = { 0x57 }, .pos = 3, .len = 4 };
const struct mu_param MU_SPO_10 = { .addr = { 0x57 }, .pos = 7, .len = 4 };
const struct mu_param MU_SPO_11 = { .addr = { 0x58 }, .pos = 3, .len = 4 };
const struct mu_param MU_SPO_12 = { .addr = { 0x58 }, .pos = 7, .len = 4 };
const struct mu_param MU_SPO_13 = { .addr = { 0x59 }, .pos = 3, .len = 4 };
const struct mu_param MU_SPO_14 = { .addr = { 0x59 }, .pos = 7, .len = 4 };
const struct mu_param MU_RPL_RESET = { .addr = { 0x5A }, .pos = 7, .len = 8 };
const struct mu_param MU_I2C_DEV_START =  { .addr = { 0x5B }, .pos = 7, .len = 8 };
const struct mu_param MU_I2C_RAM_START =  { .addr = { 0x5C }, .pos = 7, .len = 8 };
const struct mu_param MU_I2C_RAM_END =  { .addr = { 0x5D }, .pos = 7, .len = 8 };
const struct mu_param MU_I2C_DEVID =  { .addr = { 0x5E }, .pos = 7, .len = 8 };
const struct mu_param MU_I2C_RETRY =  { .addr = { 0x5F }, .pos = 7, .len = 8 };
const struct mu_param MU_EVENT_COUNT =  { .addr = { 0x73 }, .pos = 7, .len = 8 };
const struct mu_param MU_HARD_REV = { .addr = { 0x74 }, .pos = 7, .len = 8 };
const struct mu_param MU_CMD_MU = { .addr = { 0x75 }, .pos = 7, .len = 8 };
const struct mu_param MU_STATUS0 = { .addr = { 0x76 }, .pos = 7, .len = 8 };
const struct mu_param MU_STATUS1 = { .addr = { 0x77 }, .pos = 7, .len = 8 };
const struct mu_param MU_DEV_ID = { .addr = { 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D }, .pos = 7, .len = 48 };
const struct mu_param MU_MFG_ID = { .addr = { 0x7E, 0x7F }, .pos = 7, .len = 16 };

/* SPI RAM access exclusive parameters （SPI RAM访问独占参数）*/
const struct mu_param MU_CRC16 =  { .bank =0x00, .addr = { 0x81, 0x80 }, .pos = 7, .len = 16 };
const struct mu_param MU_CRC8 = { .addr = { 0x82 }, .pos = 7, .len = 8 };

/* iC-MU150/iC-MU200 exclusive parameters （iC-MU150/iC-MU200专属参数）*/
const struct mu_param MU_PHR_M = { .bank = 0x00, .addr = { 0x04 }, .pos = 7, .len = 1 };
const struct mu_param MU_PHR_N = { .bank = 0x00, .addr = { 0x0A }, .pos = 7, .len = 1 };
const struct mu_param MU_NTOA = { .bank = 0x00, .addr = { 0x0B }, .pos = 3, .len = 1 };
const struct mu_param MU_ROT_POS = { .bank = 0x00, .addr = { 0x1E }, .pos = 0, .len = 1 };

/* iC-MU defines （iC-MU定义）*/
#define MU_REGISTER_SIZE	8

/* local function declarations （局部函数声明）*/
static uint8_t get_start_bit_number(uint8_t bit_pos, uint8_t bit_len);

/* globals */
static uint8_t buf_tx[0xFF + 1] = {0};
static uint8_t buf_rx[0xFF + 1] = {0};
static uint16_t bufsize;

#define pi  3.1415926f
/**
 * @brief  SPI 全双工数据传输函数
 * @param  data_tx: 发送数据缓冲区（NULL 表示仅接收）
 * @param  data_rx: 接收数据缓冲区（NULL 表示仅发送）
 * @param  datasize: 传输数据长度（字节数）
 * @note   1. 基于 STM32 HAL 库 SPI 阻塞式传输接口
 *        2. 需确保 SPI 外设已初始化（hspi3 句柄配置正确）
 *        3. 全双工模式下，发送和接收同步进行；单工模式可传 NULL 忽略对应缓冲区
 */
uint16_t mu_spi_transfer(uint8_t *data_tx, uint8_t *data_rx, uint16_t datasize)
{

		// 1. 参数合法性检查
    if (datasize == 0 || (data_tx == NULL && data_rx == NULL))
    {
        return HAL_ERROR; // 无效参数，直接返回
    }
 
//    // 2. 等待 SPI 外设空闲
//    while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);

    // 4. 处理全双工传输（“仅发送”“仅接收”“双向传输”）
    HAL_StatusTypeDef mu150_spi_status;
	
    uint8_t mu150_dummy_buf[8] = {0}; // 仅接收时的 dummy 发送缓冲区
//	printf("  HAL_SPI_data_tx = 0x%02X\r\n", (data_tx[0]<<24)|(data_tx[1]<<16)|(data_tx[2]<<8)|data_tx[3]);
		if (data_tx == NULL && data_rx != NULL)
		{
			// 场景1：仅接收（iC-MU150 输出数据，MCU 仅读取）
			// 需发送 dummy 数据（0x00），确保 SPI 时钟正常生成
			if (datasize > sizeof(mu150_dummy_buf))
			{
				return HAL_ERROR; // 超出 dummy 缓冲区最大长度，防止溢出
			}
			mu150_spi_status = HAL_SPI_TransmitReceive(&hspi1, mu150_dummy_buf, data_rx, datasize, 100);
//			printf("  HAL_SPI_data_rx = 0x%02X\r\n", (data_rx[0]<<24)|(data_rx[1]<<16)|(data_rx[2]<<8)|data_rx[3]);
		}
		else if (data_tx != NULL && data_rx == NULL)
		{
			// 场景2：仅发送（MCU 向 iC-MU150 写数据，忽略接收）
			mu150_spi_status = HAL_SPI_TransmitReceive(&hspi1, data_tx, mu150_dummy_buf, datasize, 100);
//			printf("  HAL_SPI_data_rx = 0x%02X\r\n", (data_rx[0]<<24)|(data_rx[1]<<16)|(data_rx[2]<<8)|data_rx[3]);
		}
		else
		{
			// 场景3：双向传输（MCU 发送指令/数据，同时接收 iC-MU150 响应）
			mu150_spi_status = HAL_SPI_TransmitReceive(&hspi1, data_tx, data_rx, datasize, 100);
//			printf("  HAL_SPI_data_rx = 0x%02X\r\n", (data_rx[0]<<24)|(data_rx[1]<<16)|(data_rx[2]<<8)|data_rx[3]);
		}
//	spi_status = HAL_SPI_TransmitReceive(&hspi3, data_tx, data_rx, datasize, 100);
//    printf("  HAL_SPI_data_rx = 0x%02X\r\n", (data_rx[0]<<24)|(data_rx[1]<<16)|(data_rx[2]<<8)|data_rx[3]);
	if (mu150_spi_status != HAL_OK)
    {
        // 错误处理：重启 SPI 外设，恢复通信能力
        HAL_SPI_DeInit(&hspi1);
        MX_SPI1_Init(); // 调用自动生成的 SPI3 初始化函数
//		printf("SPI传输错误\r\n");
//		printf("\r\n");
    }
	return mu150_spi_status;

	
//				if ((data_tx == NULL && data_rx == NULL) || datasize == 0) 
//				{
//						return;  // 空指针或无数据时直接返回
//				}

//    // -------------------------- 1. 拉低NCS（片选使能）--------------------------
//    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
//		DWT_Delay_us(1); // 1μs 延时，远大于 50ns 要求，兼容不同硬件环境

    // -------------------------- 2. SPI数据双向传输 --------------------------
//    // 循环传输每1字节数据（发送与接收同步进行）
//				for (uint16_t i = 0; i < datasize; i++) 
//				{
//						uint8_t tx_byte = (data_tx != NULL) ? data_tx[i] : 0x00;  // 无发送数据时填0
//						uint8_t rx_byte = 0x00;

//						// 等待SPI发送缓冲区空
//						while (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_TXE) == RESET);
//						// 发送1字节数据
//						HAL_SPI_Transmit(&hspi3, &tx_byte, 1, HAL_MAX_DELAY);

//						// 等待SPI接收缓冲区非空
//						while (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_RXNE) == RESET);
//						// 接收1字节数据
//						HAL_SPI_Receive(&hspi3, &rx_byte, 1, HAL_MAX_DELAY);

//						// 存储接收数据（若data_rx非空）
//						if (data_rx != NULL) 
//						{
//								data_rx[i] = rx_byte;
//						}
//				}

//    // -------------------------- 3. 拉高NCS（片选失能）--------------------------
//    DWT_Delay_us(1); // 1μs 延时，远大于 500ns 要求
//		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
//    DWT_Delay_us(1); // 1μs 延时，远大于 500ns 要求
}

void mu_wait_us(uint16_t time_us)
{
	DWT_Delay_us(time_us);
}
/**
 * @brief This function can be used to switch the register and sensor data channels of the connected slaves on and off.
 *			(该功能可用于切换连接的从机的寄存器和传感器数据通道的打开和关闭。)
 * @note After startup of iC-MU RACTIVE and PACTIVE are set to 1.(iC-MU启动后，RACTIVE和PACTIVE设置为1。)
 *
 * @param active_vector is a pointer to a buffer containing the ractive/pactive vector to be transmitted.
			(Active_vector是指向缓冲区的指针，其中包含要传输的ractive/pactive向量。)
 * @param vector_size is the length of the vector in byte.(Vector_size是vector的长度，单位为byte)
 * @retval None
 */
void mu_activate(const uint8_t *active_vector, uint8_t vector_size) {
	bufsize = vector_size + 1;
	buf_tx[0] = MU_OPCODE_ACTIVATE;

	for (uint8_t i = 0; i < vector_size; ++i) {
		buf_tx[i + 1] = active_vector[i];
	}
	printf("  active_CMD = 0x%02X\r\n", (buf_tx[0]<<24)|(buf_tx[1]<<16)|(buf_tx[2]<<8)|buf_tx[3]);
	SPI3_IC_MU_CS_LOW();
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET)
	{
//		printf("CS信号被拉低：\r\n");
	}
	mu_wait_us(10);
	mu_spi_transfer(buf_tx, buf_rx, bufsize);
	mu_wait_us(10);
	SPI3_IC_MU_CS_HIGH();
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET)
	{
//		printf("CS信号被拉高：\r\n");
	}
}
/**
 * @brief This function directly transmits SDAD data.(该函数直接传输SDAD数据。)
 *
 * @param data_rx is a pointer to a buffer the received SDAD data is written to.(data_rx是一个指向缓冲区的指针，接收到的SDAD数据将写入该缓冲区。)
 * @param datasize is the number of bytes transmitted.(Datasize是传输的字节数)
 * @retval None
 */
void mu_sdad_transmission(uint8_t *data_rx, uint8_t datasize) {
	bufsize = datasize + 1;
	buf_tx[0] = MU_OPCODE_SDAD_TRANSMISSION;

	for (uint16_t i = 1; i < bufsize; i++) {
		buf_tx[i] = 0x00;
	}
	SPI3_IC_MU_CS_LOW();
	mu_wait_us(1);
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET)
	{
//		printf("CS信号被拉低：\r\n");
	}
	mu_spi_transfer(buf_tx, buf_rx, bufsize);
	
	mu_wait_us(1);
	SPI3_IC_MU_CS_HIGH();
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET)
	{
//		printf("CS信号被拉高：\r\n");
	}
	for (uint8_t i = 0; i < datasize; i++) {
		data_rx[i] = buf_rx[i + 1];
	}
}

/**
 * @brief This function can be used to request sensor data.(此函数可用于请求传感器数据。)
 *
 * @param svalid_vector is a pointer to a buffer the received svalid vector is written to.(Svalid_vector是一个指向缓冲区的指针，接收到的Svalid_vector将写入该缓冲区。)
 * @param vector_size is the length of the vector in byte.(Vector_size是vector的长度，单位为byte。)
 * @retval None
 */
void mu_sdad_status(uint8_t *svalid_vector, uint8_t vector_size) {
	bufsize = vector_size + 1;
	buf_tx[0] = MU_OPCODE_SDAD_STATUS;

	for (uint8_t i = 1; i < bufsize; i++) {
		buf_tx[i] = 0x00;
	}
	
	SPI3_IC_MU_CS_LOW();
	mu_wait_us(10);
	
	mu_spi_transfer(buf_tx, buf_rx, bufsize);

	mu_wait_us(10);
	SPI3_IC_MU_CS_HIGH();
	
	for (uint8_t i = 0; i < (bufsize - 1); i++) {
		svalid_vector[i] = buf_rx[i + 1];
	}
}

/**
 * @brief This function enables register data to be read out from the slave byte by byte.(这个函数使寄存器的数据可以一个字节一个字节地从从机读出)
 *
 * @param addr is the address of the register to be read.(DDR是要读取的寄存器的地址。)
 * @retval None
 */
uint16_t mu_read_register(uint8_t addr) {
	uint8_t status;
	uint8_t number_of_times = 0;
	bufsize = 2;
	buf_tx[0] = MU_OPCODE_READ_REGISTER;
	buf_tx[1] = addr;
	while(number_of_times< 3)
	{
		SPI3_IC_MU_CS_LOW();
		mu_wait_us(10);
		status = mu_spi_transfer(buf_tx, buf_rx, bufsize);
		mu_wait_us(10);
		SPI3_IC_MU_CS_HIGH();
		if(status == HAL_OK)
		{
//			printf("  read_register_value = 0x%02X\r\n", (buf_rx[0]<<24)|(buf_rx[1]<<16)|(buf_rx[2]<<8)|(buf_rx[3]));
			return status;
		}
		else
		{
			if(number_of_times<3)
			{
				number_of_times++;
				break;
			}
			else
			{
				return HAL_ERROR;
			}
		}
	}
	return status;
}


/**
 * @brief This function enables data to be written to the slave byte by byte.(该函数允许数据一个字节一个字节地写入从机。)
 *
 * @param addr is the address of the register to be written to.（地址是要写入的寄存器的地址。）
 * @param data_tx is the byte written to the register.（Data_tx是写入寄存器的字节）
 * @retval None
 */
void mu_write_register(uint8_t addr, uint8_t data_tx) {
	bufsize = 3;
	buf_tx[0] = MU_OPCODE_WRITE_REGISTER;
	buf_tx[1] = addr;
	buf_tx[2] = data_tx;
	
	SPI3_IC_MU_CS_LOW();
	mu_wait_us(10);
	
	mu_spi_transfer(buf_tx, buf_rx, bufsize);
	
	mu_wait_us(10);
	SPI3_IC_MU_CS_HIGH();
}

/**
 * @brief This function can be used to request the status of the last register communication and/or the last data transmission.
 *			（此功能可用于请求最后一次寄存器通信和/或最后一次数据传输的状态。）
 * @note It can be used to poll until the validity of the DATA following the SPI-STATUS byte is signaled via SPI-STATUS.
 *			（它可以用于轮询，直到SPI-STATUS字节后面的DATA通过SPI-STATUS发出有效信号为止。）
 * @param status_rx is a pointer the received status byte is written to.（Status_rx是将接收到的状态字节写入的指针。）
 * @param data_rx is a pointer the received data byte is written to.（Data_rx是将接收到的数据字节写入的指针。）
 * @retval None
 */
void mu_register_status_data(uint8_t *status_rx, uint8_t *data_rx) {
	bufsize = 3;
	buf_tx[0] = MU_OPCODE_REGISTER_STATUS_DATA;
	buf_tx[1] = 0x00;
	buf_tx[2] = 0x00;
	
	SPI3_IC_MU_CS_LOW();
	mu_wait_us(10);
	
	mu_spi_transfer(buf_tx, buf_rx, bufsize);
	
	mu_wait_us(10);
	SPI3_IC_MU_CS_HIGH();
	*status_rx = buf_rx[1];
	*data_rx = buf_rx[2];
}


/**
 * @brief This function is used to execute a command.（该功能用于执行命令。）
 *
 * @param command has to be one element of @ref CMD_MU.（命令必须是@ref CMD_MU的一个元素。）
 * @retval None
 */
void mu_write_command(CMD_MU command) {
	SPI3_IC_MU_CS_LOW();
	mu_wait_us(10);
	
	mu_write_register(MU_CMD_MU.addr[0], (uint8_t) command);
	
	mu_wait_us(10);
	SPI3_IC_MU_CS_HIGH();
}

/**
 * @brief This function switches the active memory bank.（此功能切换活动内存块。）
 *
 * @param bank number to be switched to.（要切换到的内存块编号。）
 * @retval None
 */
void mu_switch_bank(uint8_t bank) {
	SPI3_IC_MU_CS_LOW();
	mu_wait_us(10);
	
	mu_write_register(MU_BANKSEL.addr[0], bank);
	
	mu_wait_us(10);
	SPI3_IC_MU_CS_HIGH();
}

/**
 * @brief This function reads a specific chip parameter.（这个函数读取一个特定的芯片参数。）
 *
 * @note A sequence of SPI communications is executed that will increase transmission time compared to direct register access.
 *		（与直接寄存器访问相比，执行SPI通信序列将增加传输时间。）
 * @param param has to be one of the parameters defined in @ref MU_Parameters_List.（param必须是@ref MU_Parameters_List中定义的参数之一。）
 * @retval Value of the parameter read.（读取参数的值。）
 */
uint64_t mu_read_param(const struct mu_param *param) {
	uint8_t datasize = 0;

	if (param->len <= 8) {
		datasize = 1;
	}
	else if (param->len <= 16) {
		datasize = 2;
	}
	else if (param->len <= 24) {
		datasize = 3;
	}
	else if (param->len <= 32) {
		datasize = 4;
	}
	else if (param->len <= 40) {
		datasize = 5;
	}
	else {
		datasize = 6;
	}

	uint8_t startbit = get_start_bit_number(param->pos, param->len);
	uint8_t status = 0;
	uint8_t data_rx = 0;
	uint64_t param_val = 0;
	uint64_t param_mask = 0;

	if (param->addr[datasize - 1] < 0x40) {
		mu_switch_bank(param->bank);
	}

	for (uint8_t i = 0; i < datasize; i++) {
		mu_read_register(param->addr[i]);
		mu_wait_us(1);
		mu_register_status_data(&status, &data_rx);
		mu_wait_us(1);
		param_val |= ((uint64_t) data_rx << ((datasize - 1 - i) * 8));
	}

	param_val >>= startbit;

	for (uint8_t i = 0; i < param->len; i++) {
		param_mask |= ((uint64_t) 1 << i);
	}

	param_val &= param_mask;

	return param_val;
}

/**
 * @brief This function writes a specific chip parameter.（这个函数写一个特定的芯片参数。）
 *
 * @note A sequence of SPI communications is executed that will increase transmission time compared to direct register access.
 *			(与直接寄存器访问相比，执行SPI通信序列将增加传输时间。)
 * @param param has to be one of the parameters defined in @ref MU_Parameters_List.(param必须是@ref MU_Parameters_List中定义的参数之一。)
 * @param param_val is the value to be written to the parameter.(param_val是要写入参数的值。)
 * @retval None
 */
void mu_write_param(const struct mu_param *param, uint64_t param_val) {
	uint8_t datasize = 0;

	if (param->len <= 8) {
		datasize = 1;
	}
	else if (param->len <= 16) {
		datasize = 2;
	}
	else if (param->len <= 24) {
		datasize = 3;
	}
	else if (param->len <= 32) {
		datasize = 4;
	}
	else if (param->len <= 40) {
		datasize = 5;
	}
	else {
		datasize = 6;
	}

	uint8_t startbit = get_start_bit_number(param->pos, param->len);
	uint8_t status = 0;
	uint8_t data_rx = 0;
	uint64_t reg_val = 0;
	uint64_t param_mask = 0;

	if (param->addr[datasize - 1] < 0x40) {
		mu_switch_bank(param->bank);
	}

	for (uint8_t i = 0; i < datasize; i++) {
		mu_read_register(param->addr[i]);
		mu_wait_us(1);
		mu_register_status_data(&status, &data_rx);
		mu_wait_us(1);
		reg_val |= ((uint64_t) data_rx << ((datasize - 1 - i) * 8));
	}

	for (uint8_t i = 0; i < (datasize * 8); i++) {
		if ((i < startbit) ^ (i > (startbit + param->len - 1))) {
			param_mask |= ((uint64_t) 0 << i);
		} else {
			param_mask |= ((uint64_t) 1 << i);
		}
	}

	reg_val &= ~param_mask;

	param_val <<= startbit;
	param_val &= param_mask;
	param_val |= reg_val;

	for (uint8_t i = 0; i < datasize; i++) {
		uint8_t data_tx = ((param_val >> (datasize - 1 - i) * 8)) & 0xFF;
		mu_write_register(param->addr[i], data_tx);
		mu_wait_us(1);
	}
}



/* local function definitions (局部函数定义)*/
static uint8_t get_start_bit_number(uint8_t bit_pos, uint8_t bit_len) {
	bit_len = (bit_len - 1) % MU_REGISTER_SIZE;

	return (bit_pos - bit_len);
}

//得到一圈内的绝对弧度值（0~2π）
float getAngle_Without_track(void)
{
	uint8_t mu150_pos_buf[3];
	uint32_t mu150_pos = 0;//绝对位置值
	float_t mu150_angle = 0;//绝对角度值
	float_t mu150_Radian = 0.0;//绝对弧度值
	
	mu_sdad_transmission(mu150_pos_buf,3);
	
	mu150_pos = (mu150_pos_buf[0]<<16) | (mu150_pos_buf[1] << 8) | (mu150_pos_buf[2]);
	mu150_pos = mu150_pos>>5;
	
	mu150_angle = mu150_pos * ((float_t)360/524228);
	
//	printf("mu150_angle:%f\r\n",mu150_angle);
	
	mu150_Radian =(float_t)(mu150_pos * ((2*pi)/524228));
	
	
//	printf("%f,%f\r\n",mu150_angle,mu150_Radian);
	
	return mu150_Radian;
}



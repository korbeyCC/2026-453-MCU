#ifndef __MID_MODBUS_H
#define __MID_MODBUS_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// Modbus 状态机状态
typedef enum {
    MODBUS_STATE_IDLE = 0,
    MODBUS_STATE_WAIT_READ_RESP,
    MODBUS_STATE_WAIT_WRITE_RESP
} ModbusState;

// 读写回调函数定义
typedef void (*modbus_read_callback_t)(uint16_t *pData, uint8_t success);
typedef void (*modbus_write_callback_t)(uint8_t success);

// Modbus 主站控制结构体 (支持多实例并行通讯)
typedef struct {
    UART_HandleTypeDef *huart;     // 关联的底层串口句柄
    uint8_t slave_addr;            // 电机驱动器 Modbus 从站地址
    ModbusState state;             // 控制器当前状态
    
    uint8_t tx_buf[64];            // 发送缓冲区
    uint8_t rx_buf[64];            // 接收缓冲区
    volatile uint16_t rx_count;    // 实际接收到的字节数
    volatile uint8_t rx_complete;  // 接收完成标志位
    
    uint16_t timeout_cnt;          // 超时计数器 (ms)
    uint16_t expected_reg_cnt;     // 期望读取的寄存器数
    uint16_t temp_buf[64];         // 临时寄存器解析缓冲区
    
    modbus_read_callback_t read_cb;   // 读完成后的应用层异步回调
    modbus_write_callback_t write_cb; // 写完成后的应用层异步回调
} Modbus_Master_t;

// 4路 RS485 点对点主站实例
extern Modbus_Master_t modbus_masters[4];

// --- API 接口声明 ---
void MID_Modbus_Init(void);
void MID_Modbus_Process_1ms(void);
uint16_t MID_Modbus_CRC16(uint8_t *pData, uint16_t len);

bool MID_Modbus_ReadRegs(Modbus_Master_t *master, uint16_t start_addr, uint16_t reg_count, modbus_read_callback_t callback);
bool MID_Modbus_WriteSingleReg(Modbus_Master_t *master, uint16_t reg_addr, uint16_t value, modbus_write_callback_t callback);
bool MID_Modbus_WriteMultipleRegs(Modbus_Master_t *master, uint16_t start_addr, uint16_t reg_count, uint16_t *data, modbus_write_callback_t callback);
void MID_Modbus_SetBaudRate(uint32_t baudrate);

#endif // __MID_MODBUS_H

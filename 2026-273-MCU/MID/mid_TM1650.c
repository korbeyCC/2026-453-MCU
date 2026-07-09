#include "mid_TM1650.h"
#include "FreeRTOS.h"
#include "task.h"

static void TM1650_Delay(void)
{
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
}

static void TM1650_I2C_Start(void)
{
    HAL_GPIO_WritePin(TM1650_SCL_GPIO_Port, TM1650_SCL_Pin, GPIO_PIN_SET); // CLK_H
    HAL_GPIO_WritePin(TM1650_SDA_GPIO_Port, TM1650_SDA_Pin, GPIO_PIN_SET); // DIO_H
    TM1650_Delay();
    HAL_GPIO_WritePin(TM1650_SDA_GPIO_Port, TM1650_SDA_Pin, GPIO_PIN_RESET); // DIO_L
}

static void TM1650_I2C_Stop(void)
{
    HAL_GPIO_WritePin(TM1650_SCL_GPIO_Port, TM1650_SCL_Pin, GPIO_PIN_SET);   // CLK_H
    HAL_GPIO_WritePin(TM1650_SDA_GPIO_Port, TM1650_SDA_Pin, GPIO_PIN_RESET); // DIO_L
    TM1650_Delay();
    HAL_GPIO_WritePin(TM1650_SDA_GPIO_Port, TM1650_SDA_Pin, GPIO_PIN_SET); // DIO_H
}

static void TM1650_I2C_Ack(void)
{
    uint8_t timeout = 1;
    HAL_GPIO_WritePin(TM1650_SCL_GPIO_Port, TM1650_SCL_Pin, GPIO_PIN_SET); // CLK_H
    TM1650_Delay();
    HAL_GPIO_WritePin(TM1650_SCL_GPIO_Port, TM1650_SCL_Pin, GPIO_PIN_RESET); // CLK_L

    // 等待应答 (DIO_READ 为 0 代表 ACK)
    while (HAL_GPIO_ReadPin(TM1650_SDA_GPIO_Port, TM1650_SDA_Pin) && (timeout <= 100)) {
        timeout++;
    }
    TM1650_Delay();
    HAL_GPIO_WritePin(TM1650_SCL_GPIO_Port, TM1650_SCL_Pin, GPIO_PIN_RESET); // CLK_L
}

static void TM1650_I2C_WriteByte(uint8_t oneByte)
{
    uint8_t i;
    HAL_GPIO_WritePin(TM1650_SCL_GPIO_Port, TM1650_SCL_Pin, GPIO_PIN_RESET); // CLK_L
    __NOP();
    for (i = 0; i < 8; i++) {
        if (oneByte & 0x80) {
            HAL_GPIO_WritePin(TM1650_SDA_GPIO_Port, TM1650_SDA_Pin, GPIO_PIN_SET); // DIO_H
        } else {
            HAL_GPIO_WritePin(TM1650_SDA_GPIO_Port, TM1650_SDA_Pin, GPIO_PIN_RESET); // DIO_L
        }
        oneByte = oneByte << 1;

        HAL_GPIO_WritePin(TM1650_SCL_GPIO_Port, TM1650_SCL_Pin, GPIO_PIN_RESET); // CLK_L
        TM1650_Delay();
        HAL_GPIO_WritePin(TM1650_SCL_GPIO_Port, TM1650_SCL_Pin, GPIO_PIN_SET); // CLK_H
        TM1650_Delay();
        HAL_GPIO_WritePin(TM1650_SCL_GPIO_Port, TM1650_SCL_Pin, GPIO_PIN_RESET); // CLK_L
    }
}

void TM1650_Write(uint8_t addr, uint8_t data)
{
    taskENTER_CRITICAL(); // 进入临界区，保护软件I2C时序不被任务切换打断
    TM1650_I2C_Start();
    TM1650_I2C_WriteByte(addr);
    TM1650_I2C_Ack();
    TM1650_I2C_WriteByte(data);
    TM1650_I2C_Ack();
    TM1650_I2C_Stop();
    taskEXIT_CRITICAL(); // 退出临界区
}

uint8_t MID_TM1650_ReadKey(void)
{
    uint8_t i, rekey = 0;

    taskENTER_CRITICAL(); // 临界区保护
    TM1650_I2C_Start();
    TM1650_I2C_WriteByte(0x49); // 读按键命令
    TM1650_I2C_Ack();

    // 释放SDA总线（输入拉高）
    HAL_GPIO_WritePin(TM1650_SDA_GPIO_Port, TM1650_SDA_Pin, GPIO_PIN_SET);
    TM1650_Delay();

    for (i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(TM1650_SCL_GPIO_Port, TM1650_SCL_Pin, GPIO_PIN_SET); // CLK_H
        TM1650_Delay();
        rekey = rekey << 1;
        if (HAL_GPIO_ReadPin(TM1650_SDA_GPIO_Port, TM1650_SDA_Pin) == GPIO_PIN_SET) {
            rekey++;
        }
        HAL_GPIO_WritePin(TM1650_SCL_GPIO_Port, TM1650_SCL_Pin, GPIO_PIN_RESET); // CLK_L
        TM1650_Delay();
    }
    
    TM1650_I2C_Ack();
    TM1650_I2C_Stop();
    taskEXIT_CRITICAL(); // 退出临界区

    return rekey;
}

void MID_TM1650_Init(void)
{
    // 初始化命令：发送 0x48 作为控制命令地址。
    // 写入控制值 0x11: 8级亮度(0x01为1级，0x11对应低3位001代表1级亮度)，显示开(最低位为1代表开启)
    TM1650_Write(0x48, 0x01);
}

void MID_TM1650_DisplayWrite(uint8_t *seg_data, uint8_t len)
{
    if (len > 4) len = 4;
    // TM1650 的 DIG1, DIG2, DIG3, DIG4 显存地址分别是 0x68, 0x6A, 0x6C, 0x6E
    for (uint8_t i = 0; i < len; i++) {
        TM1650_Write(0x68 + i * 2, seg_data[i]);
    }
}

#include "mid_modbus.h"
#include <string.h>

// 导入底层串口配置的外部句柄
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;

// 4路主站控制结构体实例
Modbus_Master_t modbus_masters[4];

// 时间配置常数 (单位: ms)
#define MODBUS_READ_TIMEOUT_MS  30
#define MODBUS_WRITE_TIMEOUT_MS 30
#define MODBUS_INTERVAL_MS      1

// Modbus 功能码
#define MB_READ_HOLDING_REGS   0x03
#define MB_WRITE_SINGLE_REG    0x06
#define MB_WRITE_MULTIPLE_REGS 0x10

/* CRC高字节校验表 */
static const uint8_t auchCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40};

/* CRC低字节校验表 */
static const uint8_t auchCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40};

// ========================== CRC 计算函数 ==========================
uint16_t MID_Modbus_CRC16(uint8_t *pData, uint16_t len)
{
    uint8_t uchCRCHi = 0xFF;
    uint8_t uchCRCLo = 0xFF;
    uint16_t uIndex;
    while (len--) {
        uIndex   = uchCRCHi ^ *pData++;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];
    }
    return (uchCRCHi << 8 | uchCRCLo);
}

// ========================== 485 接收空闲中断强回调函数 ==========================
/**
 * @brief  重写 HAL 库串口空闲中断接收回调函数 (RxEvent)
 * @note   中断接收到空闲(IDLE)或缓冲区满时，HAL自动调用此函数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (int i = 0; i < 4; i++) {
        if (huart == modbus_masters[i].huart) {
            modbus_masters[i].rx_count    = Size;
            modbus_masters[i].rx_complete = 1;

            // 接收已完成，不在此处重新挂载以防止总线杂波和重入锁死，改在下发发送命令后按需开启
            break;
        }
    }
}

// ========================== 内部：解析响应报文 ==========================
static bool parse_response(Modbus_Master_t *master, uint16_t *destBuf, uint16_t expectedCount)
{
    uint16_t crc_recv, crc_calc;
    uint16_t i, byteCount;

    if (master->rx_count < 5) return false;
    if (master->rx_buf[0] != master->slave_addr) return false;

    // 检查异常响应
    if (master->rx_buf[1] & 0x80) return false;

    // 校验 CRC
    crc_recv = (master->rx_buf[master->rx_count - 2] << 8) | master->rx_buf[master->rx_count - 1];
    crc_calc = MID_Modbus_CRC16(master->rx_buf, master->rx_count - 2);
    if (crc_recv != crc_calc) return false;

    byteCount = master->rx_buf[2];
    if (byteCount != expectedCount * 2) return false;

    // 解析寄存器数据 (大端序转成小端主机序)
    for (i = 0; i < expectedCount; i++) {
        destBuf[i] = (master->rx_buf[3 + i * 2] << 8) | master->rx_buf[4 + i * 2];
    }
    return true;
}

// ========================== 内部：组包发送读命令 ==========================
static void send_read_cmd(Modbus_Master_t *master, uint16_t startAddr, uint16_t regCount)
{
    uint16_t crc;

    // 发送前重置 DMA 状态并清空完成标志，彻底杜绝 HAL_BUSY 锁死及脏数据拼接
    HAL_UART_DMAStop(master->huart);
    master->rx_count    = 0;
    master->rx_complete = 0;
    master->timeout_cnt = 0;

    master->tx_buf[0] = master->slave_addr;
    master->tx_buf[1] = MB_READ_HOLDING_REGS;
    master->tx_buf[2] = (startAddr >> 8) & 0xFF;
    master->tx_buf[3] = startAddr & 0xFF;
    master->tx_buf[4] = (regCount >> 8) & 0xFF;
    master->tx_buf[5] = regCount & 0xFF;

    crc               = MID_Modbus_CRC16(master->tx_buf, 6);
    master->tx_buf[6] = (crc >> 8) & 0xFF;
    master->tx_buf[7] = crc & 0xFF;

    // 调用 HAL 库串口发送 (阻塞 100ms 最大超时保护)
    HAL_UART_Transmit(master->huart, master->tx_buf, 8, 100);

    // 发送完毕，立刻重新挂载 DMA 接收监听本次应答，保障缓冲区的纯净
    HAL_UARTEx_ReceiveToIdle_DMA(master->huart, master->rx_buf, sizeof(master->rx_buf));
}

// ========================== 内部：仅重配波特率，不拆 MSP/DMA/NVIC ==========================
static void apply_uart_baud(UART_HandleTypeDef *huart, uint32_t baudrate)
{
    HAL_UART_DMAStop(huart);
    huart->Init.BaudRate = baudrate;
    /* gState 非 RESET 时 HAL_UART_Init 只会重配寄存器，不会再走 MspDeInit/MspInit */
    HAL_UART_Init(huart);
}

static void reset_master_rx(Modbus_Master_t *master)
{
    memset(master->rx_buf, 0, sizeof(master->rx_buf));
    master->rx_complete = 0;
    master->rx_count    = 0;
}

// ========================== 内部：组包发送写单个寄存器命令 ==========================
static void send_write_single_cmd(Modbus_Master_t *master, uint16_t regAddr, uint16_t value, bool start_rx)
{
    uint16_t crc;

    HAL_UART_DMAStop(master->huart);
    master->rx_count    = 0;
    master->rx_complete = 0;
    master->timeout_cnt = 0;

    master->tx_buf[0] = master->slave_addr;
    master->tx_buf[1] = MB_WRITE_SINGLE_REG;
    master->tx_buf[2] = (regAddr >> 8) & 0xFF;
    master->tx_buf[3] = regAddr & 0xFF;
    master->tx_buf[4] = (value >> 8) & 0xFF;
    master->tx_buf[5] = value & 0xFF;

    crc               = MID_Modbus_CRC16(master->tx_buf, 6);
    master->tx_buf[6] = (crc >> 8) & 0xFF;
    master->tx_buf[7] = crc & 0xFF;

    // 清除 TC 标志位，确保后续能够精确捕获当前帧的物理发送完成
    __HAL_UART_CLEAR_FLAG(master->huart, UART_FLAG_TC);

    HAL_UART_Transmit(master->huart, master->tx_buf, 8, 100);

    if (start_rx) {
        HAL_UARTEx_ReceiveToIdle_DMA(master->huart, master->rx_buf, sizeof(master->rx_buf));
    }
}

// ========================== 内部：组包发送写多个寄存器命令 ==========================
static void send_write_multiple_cmd(Modbus_Master_t *master, uint16_t startAddr, uint16_t regCount, uint16_t *data)
{
    uint16_t crc;
    uint16_t i;
    uint16_t tx_len;

    HAL_UART_DMAStop(master->huart);
    master->rx_count    = 0;
    master->rx_complete = 0;
    master->timeout_cnt = 0;

    master->tx_buf[0] = master->slave_addr;
    master->tx_buf[1] = MB_WRITE_MULTIPLE_REGS;
    master->tx_buf[2] = (startAddr >> 8) & 0xFF;
    master->tx_buf[3] = startAddr & 0xFF;
    master->tx_buf[4] = (regCount >> 8) & 0xFF;
    master->tx_buf[5] = regCount & 0xFF;
    master->tx_buf[6] = regCount * 2; // 字节数

    // 拷贝并转换大端字节序
    for (i = 0; i < regCount; i++) {
        master->tx_buf[7 + i * 2] = (data[i] >> 8) & 0xFF;
        master->tx_buf[8 + i * 2] = data[i] & 0xFF;
    }

    tx_len                     = 7 + regCount * 2;
    crc                        = MID_Modbus_CRC16(master->tx_buf, tx_len);
    master->tx_buf[tx_len]     = (crc >> 8) & 0xFF;
    master->tx_buf[tx_len + 1] = crc & 0xFF;

    HAL_UART_Transmit(master->huart, master->tx_buf, tx_len + 2, 100);

    HAL_UARTEx_ReceiveToIdle_DMA(master->huart, master->rx_buf, sizeof(master->rx_buf));
}

// ========================== 对外接口：主站初始化 ==========================
void MID_Modbus_Init(void)
{
    // 1. 绑定底层串口并设定从站地址 (由于是点对点485，各物理线上驱动器默认地址均为 0x01)
    modbus_masters[0].huart      = &huart1;
    modbus_masters[0].slave_addr = 0x01;
    modbus_masters[0].state      = MODBUS_STATE_IDLE;

    modbus_masters[1].huart      = &huart2;
    modbus_masters[1].slave_addr = 0x01;
    modbus_masters[1].state      = MODBUS_STATE_IDLE;

    modbus_masters[2].huart      = &huart3;
    modbus_masters[2].slave_addr = 0x01;
    modbus_masters[2].state      = MODBUS_STATE_IDLE;

    modbus_masters[3].huart      = &huart4;
    modbus_masters[3].slave_addr = 0x01;
    modbus_masters[3].state      = MODBUS_STATE_IDLE;

    // 2. 软件手动开启串口全局中断及 NVIC 配置 (防止CubeMX未开)
    IRQn_Type irqs[4] = {USART1_IRQn, USART2_IRQn, USART3_IRQn, UART4_IRQn};
    for (int i = 0; i < 4; i++) {
        HAL_NVIC_SetPriority(irqs[i], 5, 0); // 抢占优先级5 (允许FreeRTOS中断API调用)
        HAL_NVIC_EnableIRQ(irqs[i]);
    }

    // 3. 重置各串口 DMA 为就绪，此时不挂载接收，发送命令时才触发挂载
    for (int i = 0; i < 4; i++) {
        memset(modbus_masters[i].rx_buf, 0, sizeof(modbus_masters[i].rx_buf));
        modbus_masters[i].rx_complete = 0;
        modbus_masters[i].rx_count    = 0;

        HAL_UART_DMAStop(modbus_masters[i].huart);
    }
}

// ========================== 对外接口：非阻塞读保持寄存器 ==========================
bool MID_Modbus_ReadRegs(Modbus_Master_t *master, uint16_t start_addr, uint16_t reg_count, modbus_read_callback_t callback)
{
    if (master == NULL) return false;
    if (master->state != MODBUS_STATE_IDLE) return false;
    if (reg_count == 0 || reg_count > 64) return false;

    send_read_cmd(master, start_addr, reg_count);
    master->state            = MODBUS_STATE_WAIT_READ_RESP;
    master->expected_reg_cnt = reg_count;
    master->read_cb          = callback;
    return true;
}

// ========================== 对外接口：非阻塞写单寄存器 ==========================
bool MID_Modbus_WriteSingleReg(Modbus_Master_t *master, uint16_t reg_addr, uint16_t value, modbus_write_callback_t callback)
{
    if (master == NULL) return false;
    if (master->state != MODBUS_STATE_IDLE) return false;

    send_write_single_cmd(master, reg_addr, value, true);
    master->state    = MODBUS_STATE_WAIT_WRITE_RESP;
    master->write_cb = callback;
    return true;
}

// ========================== 对外接口：单向写单寄存器 (等待 TC 发送完，不等待应答) ==========================
void MID_Modbus_WriteSingleRegNoWait(Modbus_Master_t *master, uint16_t reg_addr, uint16_t value)
{
    if (master == NULL) return;

    send_write_single_cmd(master, reg_addr, value, false);
    while (__HAL_UART_GET_FLAG(master->huart, UART_FLAG_TC) == RESET) {
    }
    master->state = MODBUS_STATE_IDLE;
}

// ========================== 对外接口：写单寄存器后立刻切波特率再听应答 ==========================
bool MID_Modbus_WriteSingleRegThenSwitchBaud(Modbus_Master_t *master, uint16_t reg_addr, uint16_t value, uint32_t baudrate, modbus_write_callback_t callback)
{
    if (master == NULL) return false;
    if (master->state != MODBUS_STATE_IDLE) return false;

    /* 1. 先按当前 19200 波特率下发写指令 */
    send_write_single_cmd(master, reg_addr, value, false);

    /* 2. 严格等待物理移位寄存器将 8 字节完完全全吐到总线上（TC 置位），绝不在传输中途截断断电/重置波特率 */
    while (__HAL_UART_GET_FLAG(master->huart, UART_FLAG_TC) == RESET) {
    }

    /* 3. 物理帧发送完毕，立刻切换本地波特率至 115200，准备接收驱动器以 115200 发回的模式 B 应答 */
    apply_uart_baud(master->huart, baudrate);
    reset_master_rx(master);
    master->timeout_cnt = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(master->huart, master->rx_buf, sizeof(master->rx_buf));

    master->state    = MODBUS_STATE_WAIT_WRITE_RESP;
    master->write_cb = callback;
    return true;
}

// ========================== 对外接口：非阻塞写多寄存器 ==========================
bool MID_Modbus_WriteMultipleRegs(Modbus_Master_t *master, uint16_t start_addr, uint16_t reg_count, uint16_t *data, modbus_write_callback_t callback)
{
    if (master == NULL) return false;
    if (master->state != MODBUS_STATE_IDLE) return false;
    if (reg_count == 0 || reg_count > 64) return false;

    send_write_multiple_cmd(master, start_addr, reg_count, data);
    master->state    = MODBUS_STATE_WAIT_WRITE_RESP;
    master->write_cb = callback;
    return true;
}

// ========================== 1ms 周期状态机推进函数 ==========================
/**
 * @brief  主站超时管理及回调响应函数，需每 1ms 周期性调用推进
 */
void MID_Modbus_Process_1ms(void)
{
    uint8_t status_ok;

    for (int i = 0; i < 4; i++) {
        Modbus_Master_t *m = &modbus_masters[i];

        switch (m->state) {
            case MODBUS_STATE_IDLE:
                break;

            case MODBUS_STATE_WAIT_READ_RESP:
                m->timeout_cnt += MODBUS_INTERVAL_MS;
                if (m->timeout_cnt >= MODBUS_READ_TIMEOUT_MS) {
                    HAL_UART_DMAStop(m->huart); // 超时强制关闭本次 DMA，防止污染
                    m->state       = MODBUS_STATE_IDLE;
                    m->rx_complete = 0;
                    if (m->read_cb != NULL) {
                        m->read_cb(NULL, 0); // 传入 success = 0 代表超时失败
                        m->read_cb = NULL;
                    }
                    break;
                }

                if (m->rx_complete) {
                    m->rx_complete = 0;
                    if (parse_response(m, m->temp_buf, m->expected_reg_cnt)) {
                        status_ok = 1;
                    } else {
                        status_ok = 0;
                    }
                    m->state = MODBUS_STATE_IDLE;
                    if (m->read_cb != NULL) {
                        m->read_cb(m->temp_buf, status_ok);
                        m->read_cb = NULL;
                    }
                }
                break;

            case MODBUS_STATE_WAIT_WRITE_RESP:
                m->timeout_cnt += MODBUS_INTERVAL_MS;
                if (m->timeout_cnt >= MODBUS_WRITE_TIMEOUT_MS) {
                    HAL_UART_DMAStop(m->huart); // 超时强制关闭 DMA
                    m->state       = MODBUS_STATE_IDLE;
                    m->rx_complete = 0;
                    if (m->write_cb != NULL) {
                        m->write_cb(0); // 传入 success = 0
                        m->write_cb = NULL;
                    }
                    break;
                }

                if (m->rx_complete) {
                    m->rx_complete = 0;
                    // 写命令回执：正常响应应为 8 字节，并且功能码匹配
                    if (m->rx_count >= 8 && m->rx_buf[1] == m->tx_buf[1]) {
                        status_ok = 1;
                    } else {
                        status_ok = 0;
                    }
                    m->state = MODBUS_STATE_IDLE;
                    if (m->write_cb != NULL) {
                        m->write_cb(status_ok);
                        m->write_cb = NULL;
                    }
                }
                break;

            default:
                m->state = MODBUS_STATE_IDLE;
                break;
        }
    }
}

/**
 * @brief  重新设置单个主站串口波特率，并清空接收/状态机
 * @param  master   目标主站
 * @param  baudrate 目标波特率 (如 19200 / 115200)
 */
void MID_Modbus_SetMasterBaudRate(Modbus_Master_t *master, uint32_t baudrate)
{
    if (master == NULL || master->huart == NULL) {
        return;
    }

    apply_uart_baud(master->huart, baudrate);
    reset_master_rx(master);
    master->timeout_cnt = 0;
    master->state       = MODBUS_STATE_IDLE;
    master->read_cb     = NULL;
    master->write_cb    = NULL;
}

/**
 * @brief  重新设置 4 路 Modbus RS485 串口的通信波特率
 * @param  baudrate 目标波特率 (如 115200)
 */
void MID_Modbus_SetBaudRate(uint32_t baudrate)
{
    for (int i = 0; i < 4; i++) {
        MID_Modbus_SetMasterBaudRate(&modbus_masters[i], baudrate);
    }
}

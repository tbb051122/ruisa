/**
 * @file n4aia04_current.c
 * @brief N4AIA04第三通道电流读取驱动源文件
 * @author Your Name
 * @date 2025
 */

#include "driver_current.h"

/* 静态变量定义 */
static uint8_t rx_buffer[RX_BUFFER_SIZE];     /**< 接收缓冲区 */
static volatile uint8_t rx_index = 0;         /**< 接收索引 */
static volatile bool rx_complete = false;     /**< 接收完成标志 */

/**
 * @brief CRC16校验计算（Modbus标准）
 * @param data 数据指针
 * @param length 数据长度
 * @return CRC16校验值
 */
static uint16_t calculate_crc16(uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint8_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief 清空接收缓冲区
 */
static void clear_rx_buffer(void)
{
    memset(rx_buffer, 0, sizeof(rx_buffer));
    rx_index = 0;
    rx_complete = false;
}

/**
 * @brief 等待响应
 * @param timeout_ms 超时时间
 * @return true:收到响应 false:超时
 */
static bool wait_for_response(uint32_t timeout_ms)
{
    uint32_t timeout = timeout_ms;

    while (!rx_complete && timeout > 0)
    {
        R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);
        timeout--;
    }

    return rx_complete;
}

/**
 * @brief 初始化N4AIA04模块
 */
bool n4aia04_init(void)
{
    fsp_err_t err = FSP_SUCCESS;

    // 打开UART9
    err = R_SCI_UART_Open(&g_uart9_ctrl, &g_uart9_cfg);
    if (FSP_SUCCESS != err)
    {
        printf("UART9初始化失败: %d\r\n", err);
        return false;
    }

    // 注册回调函数
    err = R_SCI_UART_CallbackSet(&g_uart9_ctrl, current_uart_callback, NULL, NULL);
    if (FSP_SUCCESS != err)
    {
        printf("UART9回调设置失败: %d\r\n", err);
        return false;
    }

    printf("N4AIA04模块初始化成功 (9600,N,8,1)\r\n");
    return true;
}

/**
 * @brief 读取第三通道电流值
 */
bool n4aia04_read_channel3_current(float *current_value)
{
    uint8_t request_frame[8];
    uint16_t raw_current = 0;

    if (current_value == NULL)
    {
        printf("参数错误\r\n");
        return false;
    }

    // 清空接收缓冲区
    clear_rx_buffer();

    // 构建请求帧: 01 03 00 02 00 01 25 CA
    request_frame[0] = N4AIA04_SLAVE_ADDR;        // 地址码
    request_frame[1] = FUNC_CODE_READ;            // 功能码
    request_frame[2] = (CHANNEL3_CURRENT_REG >> 8) & 0xFF;  // 寄存器地址高字节
    request_frame[3] = CHANNEL3_CURRENT_REG & 0xFF;          // 寄存器地址低字节
    request_frame[4] = (READ_COUNT >> 8) & 0xFF;             // 读取数量高字节
    request_frame[5] = READ_COUNT & 0xFF;                    // 读取数量低字节

    // 计算CRC16校验
    uint16_t crc = calculate_crc16(request_frame, 6);
    request_frame[6] = crc & 0xFF;                // CRC低字节
    request_frame[7] = (crc >> 8) & 0xFF;         // CRC高字节

    // 发送请求
    fsp_err_t err = R_SCI_UART_Write(&g_uart9_ctrl, request_frame, 8);
    if (FSP_SUCCESS != err)
    {
        printf("发送失败: %d\r\n", err);
        return false;
    }

    // 等待响应
    if (!wait_for_response(TIMEOUT_MS))
    {
        printf("接收超时\r\n");
        return false;
    }

    // 解析响应: 01 03 02 xx xx CRC CRC
    // 检查最小长度
    if (rx_index < 7)
    {
        printf("响应帧长度错误: %d\r\n", rx_index);
        return false;
    }

    // 检查地址码和功能码
    if (rx_buffer[0] != N4AIA04_SLAVE_ADDR || rx_buffer[1] != FUNC_CODE_READ)
    {
        printf("响应帧格式错误\r\n");
        return false;
    }

    // 检查数据长度
    uint8_t data_length = rx_buffer[2];
    if (data_length != 2)
    {
        printf("数据长度错误: %d\r\n", data_length);
        return false;
    }

    // 验证CRC
    uint16_t received_crc = rx_buffer[5] | (rx_buffer[6] << 8);
    uint16_t calculated_crc = calculate_crc16(rx_buffer, 5);

    if (received_crc != calculated_crc)
    {
        printf("CRC校验错误\r\n");
        return false;
    }

    // 提取电流值（高字节在前）
    raw_current = (rx_buffer[3] << 8) | rx_buffer[4];

    // 转换为实际电流值 (单位: mA)
    *current_value = (float)raw_current * CURRENT_SCALE;

    printf("第三通道电流: %.1f mA (原始值: %d)\r\n", *current_value, raw_current);

    return true;
}

/**
 * @brief 串口回调函数
 */
void current_uart_callback(uart_callback_args_t *p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
            if (rx_index < RX_BUFFER_SIZE)
            {
                rx_buffer[rx_index++] = (uint8_t)p_args->data;

                // 判断帧结束：读取响应固定7字节
                if (rx_index >= 7)
                {
                    rx_complete = true;
                }
            }
            break;

        case UART_EVENT_TX_COMPLETE:
            // 发送完成
            break;

        default:
            break;
    }
}

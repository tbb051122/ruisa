/**
 * @file modbus_drive_control.c
 * @brief 瑞萨RA6M5 串口6 Modbus RTU电机控制实现
 */

#include "motor.h"
#include <string.h>

/* 外部串口6句柄 */
extern const uart_instance_t g_uart6;

/* 静态变量 */
static uint16_t heartbeat_counter = 0U;
static volatile bool uart_tx_complete = false;

/**
 * @brief 串口6回调函数
 */
void motor_uart_callback(uart_callback_args_t *p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_TX_COMPLETE:
            uart_tx_complete = true;
            break;

        case UART_EVENT_ERR_PARITY:
        case UART_EVENT_ERR_FRAMING:
        case UART_EVENT_ERR_OVERFLOW:
        case UART_EVENT_BREAK_DETECT:
            /* 错误处理 */
            break;

        default:
            break;
    }
}

/**
 * @brief 计算CRC16校验
 */
static uint16_t calculate_crc16(uint8_t* data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i, j;

    for (i = 0U; i < length; i++)
    {
        crc ^= data[i];
        for (j = 0U; j < 8U; j++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1) ^ 0xA001U);
            }
            else
            {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}

/**
 * @brief 通过串口6发送数据
 */
static fsp_err_t uart6_send_data(uint8_t* data, uint16_t length)
{
    fsp_err_t err;
    uint32_t timeout = 1000U;

    uart_tx_complete = false;

    err = R_SCI_UART_Write(&g_uart6_ctrl, data, (uint32_t)length);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    /* 等待发送完成 */
    while ((false == uart_tx_complete) && (timeout > 0U))
    {
        timeout--;
        R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MILLISECONDS);
    }

    if (0U == timeout)
    {
        return FSP_ERR_TIMEOUT;
    }

    /* Modbus RTU帧间隔 */
    R_BSP_SoftwareDelay(2U, BSP_DELAY_UNITS_MILLISECONDS);

    return FSP_SUCCESS;
}

/**
 * @brief 写单个寄存器
 */
static fsp_err_t modbus_write_single_register(uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_value)
{
    uint8_t frame[8];
    uint16_t crc;

    /* 构造Modbus RTU帧 */
    frame[0] = slave_addr;
    frame[1] = 0x06U;  /* 功能码06：写单个寄存器 */
    frame[2] = (uint8_t)((reg_addr >> 8) & 0xFFU);
    frame[3] = (uint8_t)(reg_addr & 0xFFU);
    frame[4] = (uint8_t)((reg_value >> 8) & 0xFFU);
    frame[5] = (uint8_t)(reg_value & 0xFFU);

    /* 计算CRC */
    crc = calculate_crc16(frame, 6U);
    frame[6] = (uint8_t)(crc & 0xFFU);
    frame[7] = (uint8_t)((crc >> 8) & 0xFFU);

    return uart6_send_data(frame, 8U);
}

/**
 * @brief 初始化电机控制系统
 */
fsp_err_t init_duty_cycle_control(void)
{
    fsp_err_t err;

    /* 打开串口6 */
    err = R_SCI_UART_Open(&g_uart6_ctrl, &g_uart6_cfg);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    /* 设置回调函数 */
    err = R_SCI_UART_CallbackSet(&g_uart6_ctrl, motor_uart_callback, NULL, NULL);
    if (FSP_SUCCESS != err)
    {
        (void)R_SCI_UART_Close(&g_uart6_ctrl);
        return err;
    }

    /* 重置心跳计数器 */
    heartbeat_counter = 0U;

    /* 等待串口稳定 */
    R_BSP_SoftwareDelay(100U, BSP_DELAY_UNITS_MILLISECONDS);

    /* 发送初始心跳 */
    err = send_heartbeat(1U);
    if (FSP_SUCCESS != err)
    {
        (void)R_SCI_UART_Close(&g_uart6_ctrl);
        return err;
    }

    /* 短暂延时 */
    R_BSP_SoftwareDelay(50U, BSP_DELAY_UNITS_MILLISECONDS);

    /* 设置为占空比控制模式 */
    err = set_duty_cycle_control(DUTY_CYCLE_STOP);
    if (FSP_SUCCESS != err)
    {
        (void)R_SCI_UART_Close(&g_uart6_ctrl);
        return err;
    }

    return FSP_SUCCESS;
}

/**
 * @brief 发送心跳包
 */
fsp_err_t send_heartbeat(uint16_t heartbeat_value)
{
    return modbus_write_single_register(DRIVE_ADDRESS, REG_HEARTBEAT, heartbeat_value);
}

/**
 * @brief 设置占空比控制
 */
fsp_err_t set_duty_cycle_control(int16_t duty_cycle)
{
    fsp_err_t err;

    /* 参数检查 */
    if ((duty_cycle < DUTY_CYCLE_MIN) || (duty_cycle > DUTY_CYCLE_MAX))
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    /* 设置占空比值 */
    err = modbus_write_single_register(DRIVE_ADDRESS, REG_DUTY_CYCLE, (uint16_t)duty_cycle);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    /* 延时确保模式设置生效 */
    R_BSP_SoftwareDelay(10U, BSP_DELAY_UNITS_MILLISECONDS);

    /* 设置控制模式 */
    err = modbus_write_single_register(DRIVE_ADDRESS, REG_CTRL_MODE, MODE_DUTY_CYCLE);

    return err;
}

/**
 * @brief 心跳任务
 */
void heartbeat_task(void)
{
    heartbeat_counter++;
    (void)send_heartbeat(heartbeat_counter);
}

/**
 * @brief 停止电机
 */
fsp_err_t stop_motor(void)
{
    return set_duty_cycle_control(DUTY_CYCLE_STOP);
}

/**
 * @brief 紧急停止电机
 */
fsp_err_t emergency_stop_motor(void)
{
    fsp_err_t err;

    /* 设置为停止模式 */
    err = modbus_write_single_register(DRIVE_ADDRESS, REG_CTRL_MODE, MODE_STOP);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    R_BSP_SoftwareDelay(10U, BSP_DELAY_UNITS_MILLISECONDS);

    /* 确保占空比为0 */
    err = modbus_write_single_register(DRIVE_ADDRESS, REG_DUTY_CYCLE, (uint16_t)DUTY_CYCLE_STOP);

    return err;
}

/**
 * @brief 电机正转
 */
fsp_err_t motor_forward(uint8_t speed_percent)
{
    int16_t duty_cycle;

    if ((speed_percent < 1U) || (speed_percent > 100U))
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    /* 转换百分比为占空比 */
    duty_cycle = (int16_t)((uint16_t)speed_percent * 10U);

    return set_duty_cycle_control(duty_cycle);
}

/**
 * @brief 电机反转
 */
fsp_err_t motor_reverse(uint8_t speed_percent)
{
    int16_t duty_cycle;

    if ((speed_percent < 1U) || (speed_percent > 100U))
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    /* 转换百分比为负占空比 */
    duty_cycle = -(int16_t)((uint16_t)speed_percent * 10U);

    return set_duty_cycle_control(duty_cycle);
}

/**
 * @brief 获取心跳计数器值
 */
uint16_t get_heartbeat_counter(void)
{
    return heartbeat_counter;
}

/**
 * @brief 重置心跳计数器
 */
void reset_heartbeat_counter(void)
{
    heartbeat_counter = 0U;
}

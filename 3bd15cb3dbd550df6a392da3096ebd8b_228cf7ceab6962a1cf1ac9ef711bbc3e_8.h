/**
 * @file n4aia04_current.h
 * @brief N4AIA04第三通道电流读取驱动头文件
 * @author Your Name
 * @date 2025
 */

#ifndef N4AIA04_CURRENT_H
#define N4AIA04_CURRENT_H

#include "hal_data.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

/* 模块配置参数 */
#define N4AIA04_SLAVE_ADDR    0x01    /**< 模块地址 */
#define FUNC_CODE_READ        0x03    /**< 读取功能码 */
#define CHANNEL3_CURRENT_REG  0x0002  /**< 第三通道电流寄存器 */
#define READ_COUNT            0x0001  /**< 读取数量 */

/* 通信参数 */
#define RX_BUFFER_SIZE        10      /**< 接收缓冲区大小 */
#define TIMEOUT_MS            1000    /**< 超时时间(毫秒) */
#define CURRENT_SCALE         0.1f    /**< 电流转换系数 0.1mA */

/**
 * @brief 初始化N4AIA04模块
 * @return true:成功 false:失败
 */
bool n4aia04_init(void);

/**
 * @brief 读取第三通道电流值
 * @param current_value 返回的电流值指针 (mA)
 * @return true:成功 false:失败
 */
bool n4aia04_read_channel3_current(float *current_value);

/**
 * @brief 串口回调函数（需要在UART配置中注册）
 * @param p_args 回调参数
 */
void current_uart_callback(uart_callback_args_t *p_args);

#endif /* N4AIA04_CURRENT_H */
